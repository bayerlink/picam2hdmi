/* Copyright 2026 Serge Rabyking
 * SPDX-License-Identifier: MIT
 *
 * The Pi as a thin bridge: sensor lines land on the wire in LINES.
 *
 * Unicam DMAs each sensor line straight into the KMS scanout buffer
 * (stride = display pitch), the HVS scans the same bytes out behind
 * the write beam, and the CPU's whole job per frame is 48 bytes of
 * bayerlink header and one control write. No libcamera: the kernel's
 * sensor driver owns the init, and the FPGA on the far end is the ISP.
 *
 * C because the kernel ABI structs come from the kernel's own headers
 * instead of hand-built copies, because a single static-ish binary is the
 * appliance shape, and because SCHED_FIFO + mlockall bound the tails an
 * interpreter only makes unlikely. `make check` holds the header it packs
 * byte-identical to bayerlink's own codec, the format's one owner.
 *
 * THE GENLOCK LOOP. The sensor and the display run on crystals that owe
 * each other nothing;
 * unsteered, the write beam laps the read beam and a tear seam
 * drifts through the picture. The loop is a plain PLL -- hblank sets
 * the rate once, vblank trims the phase continuously -- and the
 * setpoint, the write beam's lead over the read beam, IS the Pi's
 * whole contribution to end-to-end latency.
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

#include <linux/media-bus-format.h>
#include "bridge.h"
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

/* The RASTER is the link's: what the sink advertises it can carry,
 * and the size of everything that gets scanned out. The FRAME is the
 * sensor's, discovered from the sensor at startup. They were one
 * number for as long as this bridge only ever met one camera, and
 * conflating them is why a different sensor, a crop or a mode change
 * each had to be refused. A frame smaller than the raster simply sits
 * inside it -- the header states the true width and height, and a
 * receiver that takes its geometry from the stream crops correctly. */
static const struct bridge_raster *RASTER;  /* the link's, chosen at start */
static uint32_t WIDTH, HEIGHT;         /* the sensor's frame, as it says */
static int EXP_BOUND;                  /* the longest integration the lock allows */
static uint32_t VDISP;                 /* the raster's active + the header row */
#define BL_MAGIC  0x4B4C5942u          /* "BYLK"                      */

/* The header's fourcc DECLARES the mosaic order. For ov5647 1080p
 * the driver's GBRG is CORRECT -- confirmed the expensive way,
 * 2026-08-19, by walking the claim through all four orders against
 * REAL LIGHT: BGGR mixed (one axis off), GRBG swapped red and blue
 * (diagonal), GBRG rendered true. The morning had "proven" BGGR
 * from the sensor's colour-bar pattern, and that proof was the
 * mistake worth recording: the TEST PATTERN is digital, injected at
 * the readout with the GENERATOR'S own parity, one column off the
 * physical array's crop parity here. Bars validate transport and
 * demosaic; they are NO ORACLE for the mosaic phase of real light.
 * The only parity witness is a real scene through the real filters.
 * (Transport row-exactness was proven separately, 2026-08-12.) */
/* ONE row per format, three vocabularies as columns.
 *
 * The same format has three names: the kernel's mediabus code (what the
 * sensor reports), the V4L2 pixelformat (what unicam must deliver), and
 * bayerlink's fourcc (what the receiver is told to unpack). Three names
 * are inherent -- three separate tables were not, and the two that
 * existed had already drifted apart in depth coverage.
 *
 * The bayerlink column is NOT typed from memory: it is what
 * bayerlink.protocol.fourcc_for() returns, and --selftest re-proves it
 * against that codec at every start-up. Note the 8-bit row for BGGR is
 * 'BA81' where its siblings are plain order strings -- a V4L2 heirloom,
 * and exactly the kind of irregularity that gets guessed wrong once. */
static const struct {
    const char *order;
    int bits;
    uint32_t mbus, v4l2, bayerlink;
} FORMATS[] = {
    {"RGGB",  8, MEDIA_BUS_FMT_SRGGB8_1X8,   V4L2_PIX_FMT_SRGGB8,
     v4l2_fourcc('R', 'G', 'G', 'B')},
    {"GRBG",  8, MEDIA_BUS_FMT_SGRBG8_1X8,   V4L2_PIX_FMT_SGRBG8,
     v4l2_fourcc('G', 'R', 'B', 'G')},
    {"GBRG",  8, MEDIA_BUS_FMT_SGBRG8_1X8,   V4L2_PIX_FMT_SGBRG8,
     v4l2_fourcc('G', 'B', 'R', 'G')},
    {"BGGR",  8, MEDIA_BUS_FMT_SBGGR8_1X8,   V4L2_PIX_FMT_SBGGR8,
     v4l2_fourcc('B', 'A', '8', '1')},
    {"RGGB", 10, MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10P,
     v4l2_fourcc('p', 'R', 'A', 'A')},
    {"GRBG", 10, MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10P,
     v4l2_fourcc('p', 'g', 'A', 'A')},
    {"GBRG", 10, MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10P,
     v4l2_fourcc('p', 'G', 'A', 'A')},
    {"BGGR", 10, MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10P,
     v4l2_fourcc('p', 'B', 'A', 'A')},
    {"RGGB", 12, MEDIA_BUS_FMT_SRGGB12_1X12, V4L2_PIX_FMT_SRGGB12P,
     v4l2_fourcc('p', 'R', 'C', 'C')},
    {"GRBG", 12, MEDIA_BUS_FMT_SGRBG12_1X12, V4L2_PIX_FMT_SGRBG12P,
     v4l2_fourcc('p', 'g', 'C', 'C')},
    {"GBRG", 12, MEDIA_BUS_FMT_SGBRG12_1X12, V4L2_PIX_FMT_SGBRG12P,
     v4l2_fourcc('p', 'G', 'C', 'C')},
    {"BGGR", 12, MEDIA_BUS_FMT_SBGGR12_1X12, V4L2_PIX_FMT_SBGGR12P,
     v4l2_fourcc('p', 'B', 'C', 'C')},
    /* No mosaic: samples that are not light through a colour filter -- a
     * time-of-flight module's phase correlations. */
    {"MONO", 12, MEDIA_BUS_FMT_Y12_1X12,     V4L2_PIX_FMT_Y12P,
     v4l2_fourcc('Y', '1', '2', 'P')},
};

/* The row in force: set by discovery, overridden by --order, and the
 * single place the three names are read from. */
static int fmt_row = 6;                     /* GBRG 10-bit, this bench */

/* The mosaic order after a CROP.
 *
 * A crop moves the origin, so an odd offset flips that axis's parity
 * and the mosaic appears shifted: crop one column into an RGGB array
 * and the first pixel is G, on a GRBG lattice. bayerlink states the
 * order as two bits -- bit 1 the row parity of R, bit 0 the column
 * parity -- which makes the whole rule an XOR rather than a table of
 * sixteen cases.
 *
 * This is the fact that made a 1080p crop of a BGGR array report GBRG
 * and look, for one expensive morning, like a broken sensor. Written
 * once, here, over the same rows everything else reads. */
static int order_phase(const char *order) {
    if (!strcmp(order, "RGGB")) return 0;
    if (!strcmp(order, "GRBG")) return 1;
    if (!strcmp(order, "GBRG")) return 2;
    return 3;                                 /* BGGR */
}

static int format_after_crop(int row, unsigned x0, unsigned y0) {
    if (!strcmp(FORMATS[row].order, "MONO")) return row;   /* no phase */
    int phase = order_phase(FORMATS[row].order)
                ^ (int)((y0 & 1u) << 1) ^ (int)(x0 & 1u);
    for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
        if (FORMATS[i].bits == FORMATS[row].bits &&
            strcmp(FORMATS[i].order, "MONO") &&
            order_phase(FORMATS[i].order) == phase)
            return (int)i;
    return row;                               /* unreachable: 4 per depth */
}

static int format_row(const char *order, int bits) {
    for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
        if (FORMATS[i].bits == bits && !strcmp(FORMATS[i].order, order))
            return (int)i;
    return -1;
}
static int order_given;                     /* --order beats discovery */
/* What the header stamps: the bayerlink column of the row in force.
 * A function, not a variable, so it cannot be set once and then
 * disagree with the format actually being captured. */
#define bl_fourcc (FORMATS[fmt_row].bayerlink)
/* The table above is the 10-bit family, which is what this link has
 * ever carried. A sensor reporting another depth is not silently
 * relabelled as ten -- the fourcc is the receiver's ONLY statement of
 * how to unpack, so a wrong one is unreadable data that looks like a
 * broken picture. */

/* source_id indexes the SOURCES on this device -- 0 here, because
 * there is one sensor. It is not an identity: two cameras of the same
 * model are two sources, and calibration reaches the receiver over the
 * link's register channel, not by being guessed from this number. */
#define SOURCE_ID 0

/* The modeline scanned out: the chosen raster VERBATIM, claiming one
 * more active line than it has -- the header row -- inside its own
 * vertical blanking (kms.py's container: both ends are ours, and the
 * receiver counts data-enable rather than reading mode names). */
static drmModeModeInfo MODE;
static struct bridge_mode BMODE;

/* The LINK raster is the sensor's, carried over: one sensor line a
 * link line, one sensor frame a link frame, at the link's pixel clock --
 * the one fact the far end fixes (its receiver is built for it). Nothing
 * chooses a frame rate: the sensor's line time and frame length are its
 * mode's, and the raster follows them. The line is rounded UP, and to an
 * even count, so the link line is never shorter than the sensor's; the
 * vertical-blanking trim of the line lock takes up the rest. A link line
 * is as wide as the sensor's PACKED line, three bytes a link pixel -- the
 * receiver reads that many bytes from each and ignores the rest -- and no
 * narrower than the header's 48 bytes, at a stride unicam keeps: the
 * sensor writes the scanned buffer directly, so the two strides are one
 * (link_width asks unicam, it does not assume its alignment); it lasts at least the sensor's
 * width in link clocks, since the receiver puts out one sample a clock
 * (bayerlink's rate rule), and main stretches the sensor's line to
 * that where its own is shorter. The frame
 * carries LOCK_SPARE lines past the sensor's own, so the lock can trim
 * the sensor's blanking either way: with the raster at the sensor's
 * minimum it could only slow it, and acquired by waiting out the drift.
 * The syncs sit in the blanking, which the sensor's minimum holds. */
#define LOCK_SPARE 2
static struct bridge_raster LINK = {.name = "sensor"};
#define HEADER_BYTES 48
/* The link line's width in link pixels: an even count, three bytes each,
 * holding the packed line, at the first such stride unicam keeps for a
 * w x h frame of this pixel format. 0 where it keeps none nearby. */
static uint32_t link_width(const char *video, uint32_t w, uint32_t h,
                           int bits, uint32_t pixfmt) {
    uint32_t bytes = w * (uint32_t)bits / 8u;
    if (bytes < HEADER_BYTES) bytes = HEADER_BYTES;
    if (!video) {                       /* no unicam: whole link pixels */
        uint32_t cw = (bytes + 2u) / 3u;
        return cw + (cw & 1u);
    }
    int fd = open(video, O_RDWR | O_NONBLOCK);
    if (fd < 0) return 0;
    uint32_t cw = 0;
    for (uint32_t p = (bytes + 5u) / 6u * 6u; p < bytes + 6u * 64u; p += 6u) {
        struct v4l2_format f = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
        f.fmt.pix.width = w; f.fmt.pix.height = h;
        f.fmt.pix.pixelformat = pixfmt;
        f.fmt.pix.field = V4L2_FIELD_NONE;
        f.fmt.pix.bytesperline = p;
        if (ioctl(fd, VIDIOC_TRY_FMT, &f) == 0 && f.fmt.pix.bytesperline == p) {
            cw = p / 3u;
            break;
        }
    }
    close(fd);
    return cw;
}
static int derive_raster(uint32_t w, uint32_t h, uint32_t cw,
                         double pixel_rate, int hblank, int vblank,
                         uint32_t link_khz) {
    double line_s = (w + hblank) / pixel_rate;
    /* rounded up, robust to the floating point a whole count carries --
     * the panel rounds the same way, so a plan and a run agree */
    uint32_t ht = (uint32_t)ceil(line_s * link_khz * 1000.0 - 1e-6);
    ht += ht & 1u;
    if (!cw || ht < cw + 48 || ht < w || vblank < 8) return -1;
    LINK.pixel_khz = link_khz;
    LINK.h_active = cw;
    LINK.hs_beg = cw + 8;
    LINK.hs_end = cw + 40;
    LINK.h_total = ht;
    LINK.v_active = h;
    LINK.vs_beg = h + 2;
    LINK.vs_end = h + 6;
    LINK.v_total = h + (uint32_t)vblank + LOCK_SPARE;
    LINK.hsync_positive = LINK.vsync_positive = 1;
    LINK.hz = (uint32_t)(link_khz * 1000.0 / ((double)ht * LINK.v_total)
                         + 0.5);
    return 0;
}

static void set_raster(const struct bridge_raster *r) {
    RASTER = r;
    VDISP = r->v_active + 1;
    memset(&MODE, 0, sizeof MODE);
    MODE.clock = r->pixel_khz;
    MODE.hdisplay = r->h_active;
    MODE.hsync_start = r->hs_beg;
    MODE.hsync_end = r->hs_end;
    MODE.htotal = r->h_total;
    MODE.vdisplay = VDISP;
    MODE.vsync_start = r->vs_beg;
    MODE.vsync_end = r->vs_end;
    MODE.vtotal = r->v_total;
    MODE.vrefresh = r->hz;
    MODE.flags = (r->hsync_positive ? DRM_MODE_FLAG_PHSYNC
                                    : DRM_MODE_FLAG_NHSYNC)
               | (r->vsync_positive ? DRM_MODE_FLAG_PVSYNC
                                    : DRM_MODE_FLAG_NVSYNC);
    MODE.type = DRM_MODE_TYPE_USERDEF;
    snprintf(MODE.name, sizeof MODE.name, "bayerlink%u", (unsigned)VDISP);
}

static volatile sig_atomic_t running = 1;
static void on_sig(int sig) { (void)sig; running = 0; }

static void die(const char *what) { perror(what); exit(1); }

/* The default hooks do nothing; a module in modules/ overrides them
 * (weak symbols) and is compiled in by the on-device rebuild. */
__attribute__((weak)) void bridge_frame_hook(const struct bridge_frame *f) {
    (void)f;
}
__attribute__((weak)) void bridge_link_hook(const struct bridge_link *l) {
    (void)l;
}
__attribute__((weak)) void bridge_command_hook(const char *cmd, char *reply,
                                               size_t cap) {
    (void)cmd;
    snprintf(reply, cap, "err no installed module speaks commands");
}
__attribute__((weak)) void bridge_source_hook(struct bridge_source *s) {
    (void)s;
}

/* ---------------------------------------------------------------- */
/* The far-end register channel: the display cable's own I2C bus.   */
/* The receiver answers at 0x37 (16-bit register address, 32-bit    */
/* big-endian value, auto-increment). The bus NUMBER is a fact      */
/* about this Pi's wiring, so it is probed, not configured: the     */
/* first /dev/i2c-* where 0x37 answers a read is the cable.         */
/* ---------------------------------------------------------------- */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static int ddc_fd = -1;

static int ddc_try_bus(int n) {
    char path[32];
    snprintf(path, sizeof path, "/dev/i2c-%d", n);
    int fd = open(path, O_RDWR);
    if (fd < 0) return -1;
    uint8_t reg[2] = {0, 0}, val[4];
    struct i2c_msg m[2] = {
        { .addr = 0x37, .flags = 0,        .len = 2, .buf = reg },
        { .addr = 0x37, .flags = I2C_M_RD, .len = 4, .buf = val },
    };
    struct i2c_rdwr_ioctl_data x = { .msgs = m, .nmsgs = 2 };
    if (ioctl(fd, I2C_RDWR, &x) == 2) return fd;
    close(fd);
    return -1;
}

/* The connector DIR's `ddc` symlink names the one bus that is this
 * cable -- never scan: 0x37 is also DDC/CI, and a blind probe could
 * find a real monitor on the other port. */
static void ddc_probe(const char *conn_dir) {
    if (ddc_fd >= 0) { close(ddc_fd); ddc_fd = -1; }
    char link[160], tgt[160];
    snprintf(link, sizeof link, "%s/ddc", conn_dir);
    ssize_t n = readlink(link, tgt, sizeof tgt - 1);
    if (n <= 0) return;
    tgt[n] = 0;
    const char *base = strrchr(tgt, '/');
    base = base ? base + 1 : tgt;          /* "i2c-N" */
    int bus = -1;
    if (sscanf(base, "i2c-%d", &bus) != 1) return;
    ddc_fd = ddc_try_bus(bus);
    if (ddc_fd >= 0)
        printf("bridge: far-end registers answering on %s\n", base);
}

int bridge_ddc_write(uint16_t r, uint32_t value) {
    if (ddc_fd < 0) { errno = ENODEV; return -1; }
    uint8_t buf[6] = { r >> 8, r & 0xFF,
                       value >> 24, value >> 16, value >> 8, value };
    struct i2c_msg m = { .addr = 0x37, .flags = 0, .len = 6, .buf = buf };
    struct i2c_rdwr_ioctl_data x = { .msgs = &m, .nmsgs = 1 };
    return ioctl(ddc_fd, I2C_RDWR, &x) == 1 ? 0 : -1;
}

int bridge_ddc_read(uint16_t r, uint32_t *value) {
    if (ddc_fd < 0) { errno = ENODEV; return -1; }
    uint8_t reg[2] = { r >> 8, r & 0xFF }, val[4];
    struct i2c_msg m[2] = {
        { .addr = 0x37, .flags = 0,        .len = 2, .buf = reg },
        { .addr = 0x37, .flags = I2C_M_RD, .len = 4, .buf = val },
    };
    struct i2c_rdwr_ioctl_data x = { .msgs = m, .nmsgs = 2 };
    if (ioctl(ddc_fd, I2C_RDWR, &x) != 2) return -1;
    *value = (uint32_t)val[0] << 24 | (uint32_t)val[1] << 16
           | (uint32_t)val[2] << 8 | val[3];
    return 0;
}

/* ---------------------------------------------------------------- */
/* The link watch: HPD via sysfs, cheap enough for once a second.    */
/* On every edge the DDC bus is re-probed, the sink's EDID re-read   */
/* (the kernel already keeps it in sysfs), and the link hook runs -- */
/* which is where a module writes the far end's registers back:      */
/* a receiver forgets on power loss, and the sensor's owner is the   */
/* one who knows what to restore.                                    */
/* ---------------------------------------------------------------- */
static uint8_t sink_edid[256];

static char conn_dir[128];

static int link_read_state(uint8_t **edid_out) {
    static char status_path[144], edid_path[144];
    *edid_out = NULL;
    if (!status_path[0]) {
        for (int card = 0; card < 2 && !status_path[0]; card++)
            for (int c = 0; c < 4; c++) {
                char p[160];
                snprintf(p, sizeof p,
                         "/sys/class/drm/card%d-HDMI-A-%d/status", card, c);
                FILE *f = fopen(p, "r");
                if (!f) continue;
                char st[16] = {0};
                if (fgets(st, sizeof st, f) && !strncmp(st, "connected", 9)) {
                    snprintf(conn_dir, sizeof conn_dir,
                             "/sys/class/drm/card%d-HDMI-A-%d", card, c);
                    snprintf(status_path, sizeof status_path, "%.140s", p);
                    snprintf(edid_path, sizeof edid_path,
                             "%.128s/edid", conn_dir);
                }
                fclose(f);
                if (status_path[0]) break;
            }
        if (!status_path[0]) return 0;
    }
    FILE *f = fopen(status_path, "r");
    if (!f) return 0;
    char st[16] = {0};
    int up = fgets(st, sizeof st, f) && !strncmp(st, "connected", 9);
    fclose(f);
    if (up) {
        FILE *e = fopen(edid_path, "rb");
        if (e) {
            size_t n = fread(sink_edid, 1, sizeof sink_edid, e);
            fclose(e);
            if (n >= 128) *edid_out = sink_edid;
        }
    }
    return up;
}

/* ---------------------------------------------------------------- */
/* The control socket: the web panel's path to the module. One      */
/* short text command per connection; the bridge owns the transport */
/* and forwards the line to bridge_command_hook, because only the   */
/* module knows what its far end's registers mean. Lives in /run so */
/* a stale socket dies with the boot, mode 0666 because the panel   */
/* daemon is not root and a bench instrument's localhost is its     */
/* console.                                                         */
/* ---------------------------------------------------------------- */
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#define CTL_PATH "/run/bridge.sock"
static int ctl_fd = -1;

static void ctl_open(void) {
    unlink(CTL_PATH);
    ctl_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (ctl_fd < 0) return;
    struct sockaddr_un a = { .sun_family = AF_UNIX };
    snprintf(a.sun_path, sizeof a.sun_path, "%s", CTL_PATH);
    if (bind(ctl_fd, (struct sockaddr *)&a, sizeof a) < 0 ||
        listen(ctl_fd, 4) < 0) {
        fprintf(stderr, "bridge: no control socket (%s); commands off\n",
                strerror(errno));
        close(ctl_fd);
        ctl_fd = -1;
        return;
    }
    chmod(CTL_PATH, 0666);
}

/* Nonblocking on the frame path: the client connects, sends its
 * whole line, then reads, so by the time accept() hands over the
 * socket the command is normally already buffered and one read gets
 * it. If it is not, ONE bounded 5 ms grace -- a command arrives at
 * human cadence and may cost a frame's slack; it may never cost the
 * loop an unbounded wait. */
static int read_ctrl(int fd, uint32_t id, int32_t *out) {
    struct v4l2_control c = {.id = id};
    if (ioctl(fd, VIDIOC_G_CTRL, &c) < 0) return -1;
    *out = c.value;
    return 0;
}

/* A control's DEFAULT, as the driver states it for the current mode --
 * not the value in force, which is whatever the last caller left. */
static int ctrl_default(int fd, uint32_t id, int32_t *out) {
    struct v4l2_queryctrl q = {.id = id};
    if (ioctl(fd, VIDIOC_QUERYCTRL, &q) < 0) return -1;
    *out = q.default_value;
    return 0;
}

/* What the loop knows, published for the control socket. Written once
 * per frame by the loop and read between frames by the same thread, so
 * it needs no lock. */
/* PAUSE holds the picture without stopping the camera. The display
 * plane scans the payload buffer continuously and does not care
 * whether anyone is writing it, so freezing is simply a matter of
 * sending the sensor's DMA somewhere else -- a scratch buffer of the
 * same size. The loop keeps getting frame-end events, so the genlock
 * stays LOCKED throughout and resuming costs nothing: the next frame
 * lands in the displayed buffer already in phase. Stopping the camera
 * instead would drop the lock and pay for it on every unpause. */
static const uint8_t *base_map_ro;   /* the header row, for capture */
static int paused;
static int scratch_fd = -1;
/* A clip run (--clip): no sensor at all, the link timed by the clip's
 * own line and frame lengths and paced by the display's vblank. */
static int CLIP_RUN;
/* A loop run (--loop N): a sensor whose frames come in loops of N, each
 * frame carried one a link frame and paced by the display's vblank. N,
 * or 0. */
static unsigned LOOP_RUN;
/* the clip a clip run plays: its frames mapped, and how many */
static const uint8_t *CLIP;
static size_t CLIP_SIZE, CLIP_FRAMES, CLIP_FBYTES;
static int CLIP_NEW;                    /* drawn from its first frame next */

/* Map clip `path` for this run's geometry; 0, or -1 with `why` said. */
static int clip_map_file(const char *path, char *why, size_t cap) {
    int fd = open(path, O_RDONLY);
    struct stat st;
    if (fd < 0 || fstat(fd, &st) < 0) {
        snprintf(why, cap, "%s: %s", path, strerror(errno));
        if (fd >= 0) close(fd);
        return -1;
    }
    size_t frames = (size_t)st.st_size / CLIP_FBYTES;
    if (!frames || (size_t)st.st_size % CLIP_FBYTES) {
        snprintf(why, cap, "%s: %lld bytes is not whole frames of %zu",
                 path, (long long)st.st_size, CLIP_FBYTES);
        close(fd);
        return -1;
    }
    const uint8_t *m = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) {
        snprintf(why, cap, "%s: %s", path, strerror(errno));
        return -1;
    }
    if (CLIP) munmap((void *)CLIP, CLIP_SIZE);
    CLIP = m; CLIP_SIZE = st.st_size; CLIP_FRAMES = frames; CLIP_NEW = 1;
    return 0;
}
/* How many captures V4L2 holds at once: unicam wants the next slot queued
 * before the current frame ends. */
#define CAPTURE_BUFS 2
static uint8_t *scratch_map;
static uint32_t scratch_pitch;

static struct {
    uint32_t seq;
    double phase, margin;
    int locked, vblank, sensor_fd;
    time_t started;
    const uint8_t *payload_map;
    uint32_t payload_pitch;
} bstate = {.sensor_fd = -1};

/* The bridge's own vocabulary. `status` is the loop's view of itself,
 * which until now existed only as a line in the journal every five
 * seconds; the rest are the sensor's actuators, which the bridge holds
 * open and nobody else can reach while it runs. */
static void bridge_builtin(const char *cmd, char *reply, size_t cap) {
    if (!strcmp(cmd, "status")) {
        int32_t exposure = -1, gain = -1;
        if (bstate.sensor_fd >= 0) {
            read_ctrl(bstate.sensor_fd, V4L2_CID_EXPOSURE, &exposure);
            read_ctrl(bstate.sensor_fd, V4L2_CID_ANALOGUE_GAIN, &gain);
        }
        snprintf(reply, cap,
                 "ok seq=%u locked=%d paused=%d source=%s "
                 "width=%u height=%u bits=%d order=%s link_khz=%u "
                 "h_total=%u v_total=%u hz=%u lock_spare=%d exposure_max=%d "
                 "phase=%.2f margin=%.0f "
                 "vblank=%d uptime=%ld exposure=%d gain=%d sensor=%d",
                 bstate.seq, bstate.locked, paused,
                 CLIP_RUN ? "file" : LOOP_RUN ? "loop" : "sensor", WIDTH,
                 HEIGHT,
                 FORMATS[fmt_row].bits, FORMATS[fmt_row].order,
                 RASTER ? RASTER->pixel_khz : 0, RASTER ? RASTER->h_total : 0,
                 RASTER ? RASTER->v_total : 0, RASTER ? RASTER->hz : 0,
                 LOCK_SPARE, EXP_BOUND,
                 bstate.phase,
                 bstate.margin,
                 bstate.vblank,
                 bstate.started ? (long)(time(NULL) - bstate.started) : 0L,
                 exposure, gain, !CLIP_RUN);
        return;
    }
    char cpath[160];
    if (sscanf(cmd, "capture %159s", cpath) == 1) {
        /* The DISPLAYED container, header row and all -- what the wire
         * is carrying, which is what a capture is for. Take it while
         * paused and it is a whole frame by construction: the sensor is
         * writing the scratch buffer, so nothing is racing this copy.
         * Unpaused it would tear, because the next frame starts the
         * instant the last one lands. */
        if (!bstate.payload_map) {
            snprintf(reply, cap, "err no payload buffer yet");
            return;
        }
        FILE *f = fopen(cpath, "wb");
        if (!f) {
            snprintf(reply, cap, "err %s: %s", cpath, strerror(errno));
            return;
        }
        size_t row = (size_t)LINK.h_active * 3;
        int ok = fwrite(base_map_ro, 1, row, f) == row;
        for (uint32_t r = 0; ok && r < HEIGHT; r++)
            ok = fwrite(bstate.payload_map + (size_t)r * bstate.payload_pitch,
                        1, row, f) == row;
        fclose(f);
        if (!ok) {
            snprintf(reply, cap, "err %s: short write", cpath);
            return;
        }
        snprintf(reply, cap, "ok %s %u %u %s", cpath, LINK.h_active, VDISP,
                 paused ? "steady" : "live");
        return;
    }
    if (!strcmp(cmd, "preview")) {
        /* A decimated grey thumbnail of what the SENSOR is currently
         * writing -- the scratch buffer while paused, the displayed one
         * otherwise. The payload is packed 10-bit Bayer (4 pixels in 5
         * bytes, the four MSBs first), so byte p*5/4 is an 8-bit
         * approximation of pixel p. Stepping by 8 in both axes keeps
         * the CFA phase constant, so every sample is the same colour
         * and the thumbnail has no checkerboard. */
        const uint8_t *src = (paused && scratch_map) ? scratch_map
                                                     : bstate.payload_map;
        uint32_t pitch = (paused && scratch_map) ? scratch_pitch
                                                 : bstate.payload_pitch;
        if (!src) { snprintf(reply, cap, "err no payload buffer"); return; }
        /* Step 4, not 8: this preview is what a crop gets chosen
         * from, so detail matters more than the (already negligible)
         * cost of reading 480x270 bytes. Even steps only -- an odd one
         * would walk the CFA phase and stripe the image. */
        const int step = 4, w = WIDTH / step, h = HEIGHT / step;
        FILE *f = fopen("/run/bridge-preview.pgm.new", "wb");
        if (!f) {
            snprintf(reply, cap, "err preview: %s", strerror(errno));
            return;
        }
        fprintf(f, "P5\n%d %d\n255\n", w, h);
        for (int y = 0; y < h; y++) {
            const uint8_t *line = src + (size_t)(y * step) * pitch;
            for (int x = 0; x < w; x++)
                fputc(line[(x * step) * 5 / 4], f);
        }
        fclose(f);
        rename("/run/bridge-preview.pgm.new", "/run/bridge-preview.pgm");
        snprintf(reply, cap, "ok /run/bridge-preview.pgm %d %d", w, h);
        return;
    }
    if (!strcmp(cmd, "pause")) {
        if (scratch_fd < 0) {
            snprintf(reply, cap, "err no scratch buffer; cannot pause "
                                 "without disturbing the picture");
            return;
        }
        paused = 1;
        snprintf(reply, cap, "ok paused -- the display holds the last "
                             "complete frame, the sensor keeps running");
        return;
    }
    if (!strcmp(cmd, "resume")) {
        paused = 0;
        snprintf(reply, cap, "ok resumed");
        return;
    }
    snprintf(reply, cap, "err the bridge speaks: bridge status. The "
                         "sensor's exposure and gain are the AE loop's "
                         "to write, not a caller's.");
}

/* ---------------------------------------------------------------- */
/* THE CONFIGURATION: what a run streams -- the sensor in a mode, or  */
/* a clip -- in the command line's words. `bridge reconfigure` takes  */
/* the same words, parsed by the same function, so the two cannot     */
/* drift; the panel sends what it would have put in BRIDGE_ARGS.      */
/* ---------------------------------------------------------------- */
/* The longest loop of frames a sensor may come in (--loop), and the
 * most driver controls a run sets (--ctrl). */
#define LOOP_MAX 16
#define CTRLS_MAX 8

struct bridge_args {
    int seconds, margin_lines, test_pattern, selftest, order_given;
    char order[8];
    unsigned want_w, want_h, link_khz, frame_lines, line_ns;
    char video[288], subdev[128];
    char clip[160];                     /* "": the sensor */
    unsigned clip_w, clip_h, clip_bits, line_clocks;
    /* a clip of whole bayerlink containers -- each frame a header row
     * and its lines, this many link pixels wide -- not payload rows */
    unsigned containers;
    /* a sensor whose frames come in loops of this many -- a
     * measurement's, a time-of-flight module's reference phases -- each
     * sent as it comes; 0: one picture a frame */
    unsigned loop;
    /* driver controls set before it streams, by the names V4L2 tools
     * print */
    char ctrl_name[CTRLS_MAX][48];
    int32_t ctrl_value[CTRLS_MAX];
    int n_ctrls;
};

/* 0; -1 with `why` said; -2 for a word it does not know. */
static int parse_args(int argc, char **argv, struct bridge_args *a,
                      char *why, size_t cap) {
    memset(a, 0, sizeof *a);
    a->margin_lines = 10;
    a->clip_bits = 10;
    snprintf(a->subdev, sizeof a->subdev, "/dev/v4l-subdev0");
    for (int i = 0; i < argc; i++) {
        const char *k = argv[i];
        if (!strcmp(k, "--selftest")) { a->selftest = 1; continue; }
        if (!strcmp(k, "--test-pattern")) { a->test_pattern = 1; continue; }
        if (i + 1 >= argc) {
            snprintf(why, cap, "%s wants a value", k);
            return -1;
        }
        const char *v = argv[++i];
        char tail;
        if (!strcmp(k, "--seconds")) a->seconds = atoi(v);
        else if (!strcmp(k, "--margin")) a->margin_lines = atoi(v);
        else if (!strcmp(k, "--order")) {
            a->order_given = 1;
            snprintf(a->order, sizeof a->order, "%s", v);
        }
        else if (!strcmp(k, "--mode")) {
            if (sscanf(v, "%ux%u%c", &a->want_w, &a->want_h, &tail) != 2) {
                snprintf(why, cap, "--mode wants the sensor's WxH, e.g. "
                         "1296x972 -- its rate is the sensor's own");
                return -1;
            }
        }
        else if (!strcmp(k, "--link-khz")) a->link_khz = atoi(v);
        else if (!strcmp(k, "--clip"))
            snprintf(a->clip, sizeof a->clip, "%s", v);
        else if (!strcmp(k, "--size")) {
            if (sscanf(v, "%ux%u%c", &a->clip_w, &a->clip_h, &tail) != 2) {
                snprintf(why, cap, "--size wants the clip's WxH");
                return -1;
            }
        }
        else if (!strcmp(k, "--bits")) a->clip_bits = atoi(v);
        else if (!strcmp(k, "--containers")) a->containers = atoi(v);
        /* the clip's link line, in link clocks: at least its width */
        else if (!strcmp(k, "--line-clocks")) a->line_clocks = atoi(v);
        /* The sensor's frame length, in its own lines: a setting OF THE
         * SENSOR -- its vertical blanking -- which the link then follows
         * like any other part of its timing. 0: the mode's own. A clip's,
         * in link lines. */
        else if (!strcmp(k, "--frame-lines")) a->frame_lines = atoi(v);
        /* The sensor's line time, in ns: its horizontal blanking, another
         * setting of the sensor the link follows. 0: the mode's own. */
        else if (!strcmp(k, "--line-ns")) a->line_ns = atoi(v);
        else if (!strcmp(k, "--video"))
            snprintf(a->video, sizeof a->video, "%s", v);
        else if (!strcmp(k, "--subdev"))
            snprintf(a->subdev, sizeof a->subdev, "%s", v);
        else if (!strcmp(k, "--loop")) a->loop = (unsigned)atoi(v);
        else if (!strcmp(k, "--ctrl")) {
            const char *eq = strchr(v, '=');
            if (a->n_ctrls == CTRLS_MAX || !eq || eq == v ||
                (size_t)(eq - v) >= sizeof a->ctrl_name[0]) {
                snprintf(why, cap, "--ctrl wants name=value, %d at most",
                         CTRLS_MAX);
                return -1;
            }
            snprintf(a->ctrl_name[a->n_ctrls], sizeof a->ctrl_name[0],
                     "%.*s", (int)(eq - v), v);
            a->ctrl_value[a->n_ctrls++] = (int32_t)strtol(eq + 1, NULL, 0);
        }
        else {
            snprintf(why, cap, "unknown arg %s", k);
            return -2;
        }
    }
    if (a->clip[0] && !a->containers &&
        (format_row(a->order_given ? a->order : "GBRG",
                    (int)a->clip_bits) < 0 || !a->clip_w || !a->clip_h)) {
        snprintf(why, cap, "a clip needs --size WxH and a mosaic order and "
                 "depth bayerlink names (--order, --bits)");
        return -1;
    }
    if (a->clip[0] && a->loop) {
        snprintf(why, cap, "--clip and --loop are two sources; name one");
        return -1;
    }
    if (a->loop && (a->loop < 2 || a->loop > LOOP_MAX)) {
        snprintf(why, cap, "--loop wants 2 to %d frames", LOOP_MAX);
        return -1;
    }
    return 0;
}

/* A reconfiguration asked for and not yet made: the configuration, and
 * the caller's connection, held open until the new run streams or is
 * refused -- so the answer is the outcome, not an acknowledgement. */
static struct bridge_args PENDING;
static int reconfig_pending;
static int reconfig_fd = -1;

static void reconfig_answer(const char *fmt, ...) {
    if (reconfig_fd < 0) return;
    char reply[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(reply, sizeof reply, fmt, ap);
    va_end(ap);
    ssize_t w = write(reconfig_fd, reply, strlen(reply));
    (void)w;
    close(reconfig_fd);
    reconfig_fd = -1;
}

/* `bridge reconfigure <args>`: 1 if taken, the connection kept for the
 * answer; 0 with `reply` said. */
static int reconfigure(int c, char *words, char *reply, size_t cap) {
    if (reconfig_pending || reconfig_fd >= 0) {
        snprintf(reply, cap, "err a reconfiguration is already under way");
        return 0;
    }
    char *argv[64], *save = NULL;
    int argc = 0;
    for (char *t = strtok_r(words, " \t", &save); t && argc < 64;
         t = strtok_r(NULL, " \t", &save))
        argv[argc++] = t;
    char why[256];
    if (parse_args(argc, argv, &PENDING, why, sizeof why) < 0) {
        snprintf(reply, cap, "err %s", why);
        return 0;
    }
    if (PENDING.selftest || PENDING.seconds) {
        snprintf(reply, cap, "err --selftest and --seconds are the "
                 "process's, not a configuration's");
        return 0;
    }
    reconfig_pending = 1;
    reconfig_fd = c;
    return 1;
}

static void ctl_poll(void) {
    if (ctl_fd < 0) return;
    int c = accept4(ctl_fd, NULL, NULL, SOCK_NONBLOCK);
    if (c < 0) return;
    /* One line, read to its newline: a configuration is hundreds of
     * bytes, and one read() of a socket returns what has arrived, not
     * what was sent. Bounded twice -- by the buffer and by the wait. */
    char cmd[2048], reply[2048];
    ssize_t n = 0;
    for (int waits = 0; n < (ssize_t)sizeof cmd - 1 && waits < 20;) {
        ssize_t got = read(c, cmd + n, sizeof cmd - 1 - n);
        if (got > 0) {
            n += got;
            if (memchr(cmd, '\n', n)) break;
            continue;
        }
        if (got == 0) break;                    /* the caller is done */
        struct pollfd p = {.fd = c, .events = POLLIN};
        if (poll(&p, 1, 5) <= 0) waits++;
    }
    if (n > 0) {
        while (n && (cmd[n - 1] == '\n' || cmd[n - 1] == '\r')) n--;
        cmd[n] = 0;
        reply[0] = 0;
        /* The bridge answers for what the BRIDGE owns -- its own
         * telemetry and the sensor it holds -- and delegates everything
         * else to the module. Two namespaces, no collision: a module's
         * far-end registers are reached by its own verbs, and nothing
         * here needs to know what those are. */
        if (!strncmp(cmd, "bridge reconfigure", 18)
            && (cmd[18] == ' ' || !cmd[18])) {
            if (reconfigure(c, cmd + 18, reply, sizeof reply))
                return;                 /* answered by the next run */
        }
        else if (!strncmp(cmd, "bridge ", 7))
            bridge_builtin(cmd + 7, reply, sizeof reply);
        else
            bridge_command_hook(cmd, reply, sizeof reply);
        ssize_t w = write(c, reply, strlen(reply));
        (void)w;
    }
    close(c);
}

static void link_watch(int force) {
    static int last = -1;
    uint8_t *edid;
    int up = link_read_state(&edid);
    if (up == last && !force) return;
    last = up;
    if (up) ddc_probe(conn_dir);
    BMODE = (struct bridge_mode){ .raster = RASTER, .width = WIDTH,
                                  .height = HEIGHT, .lines = VDISP,
                                  /* a time-of-flight module is the
                                   * bridge's to set: no module speaks
                                   * for it */
                                  .sensor = !CLIP_RUN && !LOOP_RUN };
    struct bridge_link l = { .up = up, .edid = up ? edid : NULL,
                             .mode = &BMODE };
    printf("bridge: link %s\n", up ? "up" : "down");
    bridge_link_hook(&l);
}

/* ---------------------------------------------------------------- */
/* bayerlink v2 header: 44 bytes + CRC32. Layout owned by            */
/* bayerlink/protocol.py (_HEADER_FMT "<IHHIIIIIBBBBIII"); this copy */
/* is held byte-identical to it by `make check`, not by care.        */
/* ---------------------------------------------------------------- */

struct __attribute__((packed)) bl_header {
    uint32_t magic, /*u16*/ version_hb;   /* version u16 + header_bytes u16 */
    uint32_t fourcc, width, height, frame_seq, flags;
    uint8_t  source_id, stripe_index, stripe_count, pad0;
    uint32_t stripe_offset, full_height, pad1, crc;
};

static uint32_t crc_table[256];
static void crc_init(void) {
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++)
            c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        crc_table[n] = c;
    }
}
static uint32_t crc32z(const uint8_t *p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    while (n--) c = crc_table[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

static void pack_header(uint8_t out[48], uint32_t seq, uint8_t source_id) {
    struct bl_header h = {
        .magic = BL_MAGIC,
        .version_hb = 2u | (48u << 16),
        .fourcc = bl_fourcc, .width = WIDTH, .height = HEIGHT,
        .frame_seq = seq, .flags = 0,
        .source_id = source_id, .stripe_index = 0, .stripe_count = 1,
        .pad0 = 0, .stripe_offset = 0, .full_height = HEIGHT, .pad1 = 0,
    };
    memcpy(out, &h, 44);
    uint32_t crc = crc32z(out, 44);
    memcpy(out + 44, &crc, 4);
}

/* ---------------------------------------------------------------- */
/* DRM: mode, two planes, PRIME                                      */
/* ---------------------------------------------------------------- */

struct dumb { uint32_t handle, pitch, fb_id; uint64_t size; uint8_t *map; };

static struct dumb make_fb(int fd, uint32_t w, uint32_t h) {
    struct dumb d = {0};
    struct drm_mode_create_dumb c = {.width = w, .height = h, .bpp = 24};
    if (drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &c)) die("CREATE_DUMB");
    d.handle = c.handle; d.pitch = c.pitch; d.size = c.size;
    uint32_t handles[4] = {c.handle}, pitches[4] = {c.pitch}, offs[4] = {0};
    if (drmModeAddFB2(fd, w, h, DRM_FORMAT_RGB888, handles, pitches, offs,
                      &d.fb_id, 0))
        die("ADDFB2 (RGB888)");
    struct drm_mode_map_dumb m = {.handle = c.handle};
    if (drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &m)) die("MAP_DUMB");
    d.map = mmap(NULL, c.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                 m.offset);
    if (d.map == MAP_FAILED) die("mmap dumb");
    memset(d.map, 0, c.size);           /* the protocol's zero padding */
    return d;
}

/* A plane of `type` the CRTC can use -- a free one, where `free_only` --
 * or 0. */
static uint32_t find_plane(int fd, int crtc_index, uint64_t type,
                           int free_only) {
    drmModePlaneResPtr planes = drmModeGetPlaneResources(fd);
    if (!planes) die("GetPlaneResources");
    uint32_t found = 0;
    for (uint32_t i = 0; i < planes->count_planes && !found; i++) {
        drmModePlanePtr p = drmModeGetPlane(fd, planes->planes[i]);
        if (!p) continue;
        if ((p->possible_crtcs & (1u << crtc_index))
            && (!free_only || p->fb_id == 0)) {
            drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(
                fd, p->plane_id, DRM_MODE_OBJECT_PLANE);
            for (uint32_t j = 0; props && j < props->count_props; j++) {
                drmModePropertyPtr pr = drmModeGetProperty(fd, props->props[j]);
                if (pr && !strcmp(pr->name, "type")
                    && props->prop_values[j] == type)
                    found = p->plane_id;
                drmModeFreeProperty(pr);
            }
            drmModeFreeObjectProperties(props);
        }
        drmModeFreePlane(p);
    }
    drmModeFreePlaneResources(planes);
    return found;
}

/* The overlay the payload rides: overlay only -- a cursor plane would
 * take the SETPLANE and refuse the size twenty minutes later, in the
 * compositor. */
static uint32_t overlay_plane(int fd, int crtc_index) {
    uint32_t found = find_plane(fd, crtc_index, DRM_PLANE_TYPE_OVERLAY, 1);
    if (!found) { fprintf(stderr, "no overlay plane free\n"); exit(1); }
    return found;
}

/* A plane's property, by name: its id, or 0. */
static uint32_t plane_prop(int fd, uint32_t plane, const char *name) {
    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(
        fd, plane, DRM_MODE_OBJECT_PLANE);
    uint32_t id = 0;
    for (uint32_t j = 0; props && j < props->count_props && !id; j++) {
        drmModePropertyPtr pr = drmModeGetProperty(fd, props->props[j]);
        if (pr && !strcmp(pr->name, name)) id = pr->prop_id;
        drmModeFreeProperty(pr);
    }
    drmModeFreeObjectProperties(props);
    return id;
}

static void force_full_range(int fd, uint32_t connector_id) {
    /* A limited-range link clamps to 16..235 -- precisely where raw
     * data lives. Same policy, same reason as kms.py. */
    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(
        fd, connector_id, DRM_MODE_OBJECT_CONNECTOR);
    for (uint32_t i = 0; props && i < props->count_props; i++) {
        drmModePropertyPtr pr = drmModeGetProperty(fd, props->props[i]);
        if (pr && !strcmp(pr->name, "Broadcast RGB")) {
            for (int e = 0; e < pr->count_enums; e++)
                if (!strcmp(pr->enums[e].name, "Full"))
                    drmModeObjectSetProperty(fd, connector_id,
                        DRM_MODE_OBJECT_CONNECTOR, pr->prop_id,
                        pr->enums[e].value);
        }
        drmModeFreeProperty(pr);
    }
    drmModeFreeObjectProperties(props);
}

/* ---------------------------------------------------------------- */
/* V4L2: the kernel driver owns the sensor, we own eight ioctls      */
/* ---------------------------------------------------------------- */

static int set_ctrl(int fd, uint32_t id, int32_t value, const char *name) {
    struct v4l2_control c = {.id = id, .value = value};
    if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0) {
        fprintf(stderr, "S_CTRL %s=%d: %s\n", name, value, strerror(errno));
        return -1;
    }
    return 0;
}

/* Ask the sensor what it is producing, instead of declaring it.
 *
 * One ioctl answers three questions that were three separate baked
 * constants: the geometry, the mosaic order and the sample depth --
 * the last two both encoded in the mediabus code. The order in
 * particular was a guess that cost a bench day (a morning "proving"
 * BGGR before GBRG rendered true), and a guess is exactly what a
 * sensor never needs to be asked twice about.
 *
 * Returns 0 and leaves everything untouched if the subdev will not
 * answer -- an older kernel, an unusual driver -- so a bridge that
 * cannot ask still runs on the numbers it was built with. */
static int discover_sensor(int sfd, uint32_t *w, uint32_t *h, int *row) {
    struct v4l2_subdev_format f = {.which = V4L2_SUBDEV_FORMAT_ACTIVE,
                                   .pad = 0};
    if (ioctl(sfd, VIDIOC_SUBDEV_G_FMT, &f) < 0)
        return 0;
    for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
        if (FORMATS[i].mbus == f.format.code) {
            *w = f.format.width;
            *h = f.format.height;
            *row = (int)i;
            return 1;
        }
    fprintf(stderr, "bridge: sensor reports mediabus code 0x%04x, which "
            "this build does not know; keeping %ux%u\n",
            f.format.code, *w, *h);
    return 0;
}

static int64_t get_pixel_rate(int fd) {
    struct v4l2_ext_control c = {.id = V4L2_CID_PIXEL_RATE};
    struct v4l2_ext_controls cs = {
        .which = V4L2_CTRL_WHICH_CUR_VAL, .count = 1, .controls = &c};
    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) < 0) die("PIXEL_RATE");
    return c.value64;
}

/* ---------------------------------------------------------------- */

static double ts_sec(struct timeval tv) {
    return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
}

/* Bring a kernel timestamp into the MONOTONIC frame, whatever clock
 * stamped it. The event in question just happened, so whichever clock
 * puts it in the recent past is the clock that stamped it. Assuming
 * instead of identifying cost two forty-second runs: first the vblank
 * side, then the capture side, each time NTP slew wearing the costume
 * of crystal drift while the actuator shouted forty lines a frame into
 * a meter that could not see it. */
static double to_mono(double ts, int *bad) {
    struct timespec m, r;
    clock_gettime(CLOCK_MONOTONIC, &m);
    clock_gettime(CLOCK_REALTIME, &r);
    double nm = m.tv_sec + m.tv_nsec * 1e-9;
    double nr = r.tv_sec + r.tv_nsec * 1e-9;
    double age_m = nm - ts, age_r = nr - ts;
    if (age_m >= 0 && age_m < 0.5) return ts;
    if (age_r >= 0 && age_r < 0.5) return nm - age_r;
    if (bad) *bad = 1;
    return ts;
}


/* WHICH NODE IS THE SENSOR, asked by NAME rather than by number.
 *
 * /dev/videoN is enumeration order, not identity. This was pinned to
 * /dev/video2 and stayed correct until a power cut on 2026-08-25: the
 * Pi came back with unicam on video0 and a USB capture card on video1
 * and video2, so the bridge asked a UVC card for 10-bit Bayer and got
 * "S_FMT: Invalid argument" -- the right complaint about the wrong
 * device. Everything after that was consequence: the unit failed,
 * Restart=always made it flap, and the panel read the flapping as "no
 * bridge here" and opened the sensor itself.
 *
 * The kernel publishes a stable name for every node, so ask for that.
 * The name is generation-specific -- Pi 0-4 present the CSI-2 receiver
 * as `unicam`, Pi 5 replaced it with `rp1-cfe` -- so this is a table,
 * in preference order, not one string.
 */
static const char *const CAPTURE_NAMES[] = {
    "unicam-image",         /* bcm2835-unicam: Pi 0-4 */
    "rp1-cfe-csi2_ch0",     /* rp1-cfe: Pi 5 */
};

static int resolve_capture_node(char *out, size_t out_len) {
    for (size_t n = 0; n < sizeof CAPTURE_NAMES / sizeof *CAPTURE_NAMES; n++) {
        DIR *d = opendir("/sys/class/video4linux");
        if (!d) return -1;
        struct dirent *e;
        while ((e = readdir(d))) {
            if (strncmp(e->d_name, "video", 5)) continue;
            /* 23 of prefix + a 240-bounded node name + "/name" */
            char path[320], name[128];
            snprintf(path, sizeof path,
                     "/sys/class/video4linux/%.240s/name",
                     e->d_name);
            FILE *f = fopen(path, "r");
            if (!f) continue;
            if (!fgets(name, sizeof name, f)) { fclose(f); continue; }
            fclose(f);
            name[strcspn(name, "\n")] = 0;
            if (strcmp(name, CAPTURE_NAMES[n])) continue;
            /* precision bound: d_name is a 255-byte field and
             * the compiler cannot see that a node name is short */
            snprintf(out, out_len, "/dev/%.240s", e->d_name);
            closedir(d);
            return 0;
        }
        closedir(d);
    }
    return -1;
}

/* The display: its device, the connected connector and a CRTC that can
 * reach it, waited for. 0, or 1 where none came. */
static int open_display(int *dfd_out, uint32_t *conn_out, uint32_t *crtc_out,
                        int *index_out) {
    int dfd = open("/dev/dri/card1", O_RDWR);
    if (dfd < 0) dfd = open("/dev/dri/card0", O_RDWR);
    if (dfd < 0) die("open dri");
    drmSetClientCap(dfd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
    /* atomic, for the one change that must land whole: a loop run's
     * header and payload, on two planes, in one vertical blank */
    drmSetClientCap(dfd, DRM_CLIENT_CAP_ATOMIC, 1);

    /* The sink may not be there YET. At power-on the board downstream
     * boots and loads its bitstream a minute or two after this Pi, and
     * until it does there is no HDMI sink to drive. Refusing at once
     * left the unit to burn its restart budget in seconds and end
     * `failed` with the board seconds from ready. So a missing display
     * is waited for -- BOUNDED, because one that never comes is the
     * refusal it always was, and the unit must still end `failed`
     * rather than wait forever. While it waits the bridge is running
     * and owns the camera, which is the truth. */
    enum { DISPLAY_WAIT_S = 300, DISPLAY_REPORT_S = 30 };
    uint32_t connector_id = 0, crtc_id = 0;
    int crtc_index = 0;
    for (int waited = 0;; waited++) {
        drmModeResPtr res = drmModeGetResources(dfd);
        if (!res) die("GetResources");
        for (int i = 0; i < res->count_connectors && !connector_id; i++) {
            drmModeConnectorPtr c =
                drmModeGetConnector(dfd, res->connectors[i]);
            if (c && c->connection == DRM_MODE_CONNECTED) {
                connector_id = c->connector_id;
                /* The CRTC comes from the connector's ENCODERS' mask --
                 * never from a guess. An all-ones fallback once picked
                 * CRTC 0, which on vc4 is txp, the writeback engine: a
                 * perfectly real CRTC that can never drive HDMI, and an
                 * EINVAL that names no names. The encoder knows which
                 * CRTCs can reach the wire; ask it. */
                uint32_t possible = 0;
                for (int j = 0; j < c->count_encoders; j++) {
                    drmModeEncoderPtr e =
                        drmModeGetEncoder(dfd, c->encoders[j]);
                    if (!e) continue;
                    if (e->crtc_id && !crtc_id) crtc_id = e->crtc_id;
                    possible |= e->possible_crtcs;
                    drmModeFreeEncoder(e);
                }
                for (int k = 0; k < res->count_crtcs; k++)
                    if (crtc_id ? res->crtcs[k] == crtc_id
                                : (possible & (1u << k))) {
                        crtc_id = res->crtcs[k]; crtc_index = k; break;
                    }
            }
            drmModeFreeConnector(c);
        }
        drmModeFreeResources(res);
        if (connector_id && crtc_id) break;
        connector_id = crtc_id = 0;
        crtc_index = 0;
        if (waited >= DISPLAY_WAIT_S) {
            fprintf(stderr, "no connected connector/CRTC after %d s\n",
                    DISPLAY_WAIT_S);
            close(dfd);
            return 1;
        }
        if (waited % DISPLAY_REPORT_S == 0)
            printf("bridge: no display connected yet -- waiting for the "
                   "sink (%d s of %d)\n", waited, DISPLAY_WAIT_S);
        sleep(1);
    }
    force_full_range(dfd, connector_id);
    *dfd_out = dfd; *conn_out = connector_id; *crtc_out = crtc_id;
    *index_out = crtc_index;
    return 0;
}

/* The link's clock is the far end's: its receiver is built for one, and
 * it says which in its EDID -- the first detailed timing, the container
 * it prefers. A sink that states none gets the 1080p30 family's 74.25
 * MHz. --link-khz overrides. */
static uint32_t sink_link_khz(void) {
    uint8_t *e = NULL;
    link_read_state(&e);
    uint32_t clk = e ? (uint32_t)(e[54] | e[55] << 8) * 10u : 0;
    printf("bridge: link at %u kHz, %s\n", clk ? clk : 74250u,
           clk ? "as the sink's EDID prefers" : "the sink stating none");
    return clk ? clk : 74250u;
}

/* ---------------------------------------------------------------- */
/* THE DISPLAY, owned for the life of the process. A run asks for    */
/* its container and its mode and gets the least change that makes   */
/* them: nothing where both already hold; the mode set once, in a     */
/* vertical blank, where only the timing moves; new buffers where the */
/* geometry does, drawn before they go on the wire. Between two runs  */
/* nothing else ever drives the cable -- the console that used to,    */
/* for most of a second each time the process was restarted, put an   */
/* unknown signal on it and cost the far end two link drops.          */
/* ---------------------------------------------------------------- */
static struct {
    int dfd, crtc_index;
    uint32_t crtc, conn, plane;
    /* the primary plane, which scans the header row's buffer, and each
     * plane's FB_ID property, for a change of both in one commit */
    uint32_t primary, primary_fb, plane_fb;
    drmModeModeInfo mode;               /* on the wire; clock 0: none yet */
} D = {.dfd = -1};

/* A container: the header row's buffer, the payload's, and the spare
 * the sensor writes while the picture holds still. */
struct container {
    struct dumb base, pay, scratch;
    int scratch_fd;
    uint32_t cw, lines, height;
};
static struct container ON = {.scratch_fd = -1};    /* on the wire */
static struct container NEW = {.scratch_fd = -1};   /* being drawn */

static void free_fb(struct dumb *d) {
    if (!d->map) return;
    munmap(d->map, d->size);
    drmModeRmFB(D.dfd, d->fb_id);
    struct drm_mode_destroy_dumb x = {.handle = d->handle};
    drmIoctl(D.dfd, DRM_IOCTL_MODE_DESTROY_DUMB, &x);
    memset(d, 0, sizeof *d);
}

static void container_free(struct container *c) {
    if (c->scratch_fd >= 0) close(c->scratch_fd);
    free_fb(&c->base); free_fb(&c->pay); free_fb(&c->scratch);
    memset(c, 0, sizeof *c);
    c->scratch_fd = -1;
}

static int wait_vblank(void) {
    drmVBlank vb = {.request = {
        .type = DRM_VBLANK_RELATIVE
              | ((D.crtc_index << DRM_VBLANK_HIGH_CRTC_SHIFT)
                 & DRM_VBLANK_HIGH_CRTC_MASK),
        .sequence = 1}};
    return drmWaitVBlank(D.dfd, &vb);
}

/* The buffers a run draws into: the ones on the wire where they already
 * hold its container -- `cw` link pixels by `lines`, the header row's
 * included, over a `height`-line frame -- else new ones, zeroed, which
 * the run may draw its first frame into before display_show. */
static struct container *display_buffers(uint32_t cw, uint32_t lines,
                                         uint32_t height) {
    if (ON.base.map && ON.cw == cw && ON.lines == lines
        && ON.height == height)
        return &ON;
    container_free(&NEW);
    NEW.base = make_fb(D.dfd, cw, lines);
    NEW.pay = make_fb(D.dfd, cw, height);
    NEW.scratch = make_fb(D.dfd, cw, height);
    if (drmPrimeHandleToFD(D.dfd, NEW.scratch.handle, DRM_CLOEXEC | DRM_RDWR,
                           &NEW.scratch_fd)) {
        fprintf(stderr, "bridge: no scratch dma-buf; pause unavailable\n");
        NEW.scratch_fd = -1;
    }
    NEW.cw = cw; NEW.lines = lines; NEW.height = height;
    return &NEW;
}

/* Put the container display_buffers gave and `mode` on the wire: nothing
 * where both are there already; else the change made in a vertical blank
 * -- the frame on the wire finished whole, the mode set landing in its
 * blanking -- and the buffers it replaced freed after. 0, or -1 with
 * `why` said and the wire as it was. */
static int display_show(const drmModeModeInfo *mode, char *why, size_t cap) {
    int fresh = NEW.base.map != NULL;
    if (!fresh && !memcmp(&D.mode, mode, sizeof *mode)) return 0;
    struct container *c = fresh ? &NEW : &ON;
    /* The old mode's last frame is whole on the wire; the frames after it
     * carry no header, so the far end takes none of them. The display
     * drops the old mode well after the call that sets the new one --
     * ~20 ms on a vc4, wherever that lands in a frame -- so the cut must
     * fall in a frame that is no frame to the far end, never in a picture.
     * The header is cleared at one vertical blank and the mode set at the
     * next: the vblank's news arrives after the blank begins, sometimes
     * after the next frame's header line has already gone out, and that
     * frame, still a picture, was the one the cut landed in. The new
     * mode's first frame gets its header as the run draws it. */
    if (D.mode.clock) {
        wait_vblank();
        memset(ON.base.map, 0, HEADER_BYTES);
        wait_vblank();
    }
    if (drmModeSetCrtc(D.dfd, D.crtc, c->base.fb_id, 0, 0, &D.conn, 1,
                       (drmModeModeInfo *)mode) ||
        drmModeSetPlane(D.dfd, D.plane, D.crtc, c->pay.fb_id, 0,
                        0, 1, c->cw, c->height,
                        0, 0, c->cw << 16, c->height << 16)) {
        snprintf(why, cap, "the display refused the %ux%u mode: %s",
                 mode->htotal, mode->vtotal, strerror(errno));
        if (fresh) container_free(&NEW);
        return -1;
    }
    D.mode = *mode;
    if (fresh) {
        container_free(&ON);
        ON = NEW;
        memset(&NEW, 0, sizeof NEW);
        NEW.scratch_fd = -1;
    }
    /* the pause and the preview read the spare through these */
    scratch_fd = ON.scratch_fd;
    scratch_map = ON.scratch.map;
    scratch_pitch = ON.scratch.pitch;
    base_map_ro = ON.base.map;
    bstate.payload_map = ON.pay.map;
    bstate.payload_pitch = ON.pay.pitch;
    return 0;
}

/* ---------------------------------------------------------------- */
/* RUNS. A run is one configuration streaming -- the sensor in a mode,*/
/* or a clip -- and it ends one of three ways: the process is done,   */
/* a reconfiguration is waiting (the frame on the wire finished       */
/* whole), or this configuration cannot run, said in WHY. The first   */
/* configuration is the command line's; every later one arrives as    */
/* `bridge reconfigure`, and the run that follows answers it -- ok    */
/* once it streams, or the refusal, the previous configuration then   */
/* running again. Every run counts its frames in the header from 0,   */
/* the same configuration asked for again included: a                 */
/* reconfiguration is a change of content, and the count starting     */
/* over is how the far end can tell where one ends and the next       */
/* begins.                                                            */
/* ---------------------------------------------------------------- */
enum { RUN_DONE, RUN_RECONFIGURE, RUN_REFUSED };
static char WHY[256];
static uint32_t SINK_KHZ;               /* the sink's clock, asked once */
static int SECONDS;                     /* the process's --seconds */
static time_t T0;

static int refuse(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(WHY, sizeof WHY, fmt, ap);
    va_end(ap);
    fprintf(stderr, "bridge: %s\n", WHY);
    return RUN_REFUSED;
}

static int time_up(void) {
    return SECONDS > 0 && time(NULL) - T0 >= SECONDS;
}

/* The run streams: its mode told to the module, and the reconfiguration
 * that asked for it answered. */
static void run_started(void) {
    bstate.started = time(NULL);
    link_watch(1);
    reconfig_answer("ok %s %ux%u of %ux%u at %u kHz, %u Hz",
                    CLIP_RUN ? "clip" : LOOP_RUN ? "loop" : "sensor", WIDTH, HEIGHT,
                    LINK.h_total, LINK.v_total, LINK.pixel_khz, LINK.hz);
}

/* A CLIP RUN: no sensor. The clip is raw payload frames -- each `h`
 * packed lines of the format's bytes, tightly -- and the link is timed by
 * the clip's own line (in link clocks) and frame (in lines), which
 * whoever plans it chose; frames are paced by the display's own vblank,
 * one clip frame a link frame, so the far end sees the clip's rate. The
 * bridge writes the header, as it does for the sensor. Every hook runs as
 * usual, with no sensor to hand on. A new clip and a new rate are each a
 * reconfiguration: on a link
 * of the same timing the next frame is simply the new clip's; at a new
 * frame length the mode is set once, in a vertical blank. */
/* One frame of the clip into a container: its rows of `bytes`, and for a
 * clip of whole containers its own header row first, as it was recorded. */
static void draw_clip(struct container *c, size_t at, int whole, size_t bytes) {
    const uint8_t *src = CLIP + at * CLIP_FBYTES;
    if (whole) { memcpy(c->base.map, src, bytes); src += bytes; }
    for (uint32_t r = 0; r < HEIGHT; r++)
        memcpy(c->pay.map + (size_t)r * c->pay.pitch, src + (size_t)r * bytes,
               bytes);
}

static int run_clip(const struct bridge_args *a) {
    CLIP_RUN = 1; LOOP_RUN = 0; paused = 0;
    char why[256];
    /* A clip of whole containers is a recording of the wire, and it goes
     * back out as it was: each frame's own header row, the frame's
     * geometry, depth and mosaic order as its first header states them --
     * that header is the one owner of those facts, so nothing here
     * restates them. Payload rows get the header this bridge writes. */
    const int whole = a->containers != 0;
    int row;
    uint32_t cw;
    size_t bytes;                       /* a row's bytes in the file */
    if (whole) {
        struct bl_header h;
        int fd = open(a->clip, O_RDONLY);
        int got = fd < 0 ? -1 : (int)read(fd, &h, sizeof h);
        if (fd >= 0) close(fd);
        if (got != (int)sizeof h)
            return refuse("%s: no header to read", a->clip);
        row = -1;
        for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
            if (FORMATS[i].bayerlink == h.fourcc) row = (int)i;
        if (h.magic != BL_MAGIC || row < 0 || !h.width || !h.height)
            return refuse("%s: not bayerlink containers", a->clip);
        WIDTH = h.width; HEIGHT = h.height;
        cw = a->containers;
        bytes = (size_t)cw * 3;
        if (bytes < (size_t)WIDTH * FORMATS[row].bits / 8u || bytes < HEADER_BYTES)
            return refuse("%s: a %u-pixel container cannot hold a %u-sample "
                          "line", a->clip, cw, WIDTH);
        CLIP_FBYTES = bytes * (HEIGHT + 1);
    } else {
        row = format_row(a->order_given ? a->order : "GBRG", (int)a->clip_bits);
        WIDTH = a->clip_w; HEIGHT = a->clip_h;
        bytes = (size_t)WIDTH * FORMATS[row].bits / 8u;
        CLIP_FBYTES = bytes * HEIGHT;
        cw = link_width(NULL, WIDTH, HEIGHT, FORMATS[row].bits, 0);
    }
    fmt_row = row;
    if (clip_map_file(a->clip, why, sizeof why) < 0) return refuse("%s", why);
    uint32_t link_khz = a->link_khz ? a->link_khz : SINK_KHZ;
    uint32_t line = a->line_clocks ? a->line_clocks : WIDTH;
    uint32_t frame = a->frame_lines ? a->frame_lines
                                    : HEIGHT + 1 + 8 + LOCK_SPARE;
    if (line < WIDTH || frame < HEIGHT + 10 ||
        derive_raster(WIDTH, HEIGHT, cw, link_khz * 1000.0,
                      (int)line - (int)WIDTH,
                      (int)frame - (int)HEIGHT - LOCK_SPARE, link_khz) < 0)
        return refuse("a %ux%u clip does not fit a %u-clock line and "
                      "%u-line frame at %u kHz", WIDTH, HEIGHT, line, frame,
                      link_khz);
    set_raster(&LINK);
    printf("bridge: clip %s: %zu %ux%u %s %d-bit frame(s), link raster "
           "%ux%u of %ux%u at %u kHz, %u Hz\n",
           a->clip, CLIP_FRAMES, WIDTH, HEIGHT, FORMATS[row].order,
           FORMATS[row].bits, LINK.h_active, LINK.v_active, LINK.h_total,
           LINK.v_total, link_khz, LINK.hz);

    /* new buffers get the clip's first frame before they go on the wire,
     * so the first frame of a new mode is a whole one */
    struct container *c = display_buffers(LINK.h_active, VDISP, HEIGHT);
    if (c == &NEW) {
        if (!whole) pack_header(c->base.map, 0, SOURCE_ID);
        draw_clip(c, 0, whole, bytes);
    }
    if (display_show(&MODE, why, sizeof why) < 0) return refuse("%s", why);
    bstate.locked = 1;
    bstate.sensor_fd = -1;
    run_started();

    /* Each frame is drawn at the vblank before it is scanned: the copy
     * runs far faster than the beam, so it stays ahead of it, as the
     * sensor's writes do. */
    uint32_t seq = 0;
    size_t at = 0;
    while (running && !time_up()) {
        if (wait_vblank()) {
            fprintf(stderr, "bridge: WaitVBlank: %s\n", strerror(errno));
            break;
        }
        /* the frame on the wire is whole, and repeats until the next
         * configuration draws over it */
        if (reconfig_pending) return RUN_RECONFIGURE;
        if (!whole) pack_header(ON.base.map, seq, SOURCE_ID);
        if (CLIP_NEW) at = 0;
        if (CLIP_NEW || CLIP_FRAMES > 1) {
            CLIP_NEW = 0;
            draw_clip(&ON, at, whole, bytes);
            at = (at + 1) % CLIP_FRAMES;
        }
        seq++;
        bstate.seq = seq;
        struct bridge_frame bf = {.seq = seq, .locked = 1, .sensor_fd = -1};
        bridge_frame_hook(&bf);
        if ((seq & 31) == 0) link_watch(0);
        ctl_poll();
    }
    printf("bridge: %u frames of the clip\n", seq);
    return RUN_DONE;
}

/* A LOOP RUN: a sensor whose frames come in loops -- a measurement's,
 * such as a time-of-flight module's correlation of the returning light
 * with its reference phases, one phase a frame. Its frames reach the
 * wire as any sensor's do -- unicam writes each into a buffer the
 * display scans, the samples untouched, and the CPU writes the header.
 *
 * ITS FRAMES COME AHEAD OF TIME. Such a sensor captures a loop's frames
 * a few milliseconds apart, faster than the display scans a frame, and
 * then waits most of a loop. Each frame lands in a buffer of its own and
 * goes on the wire at the first vertical blank after it arrived, in the
 * order they came; a blank with no new frame clears the header, so that
 * frame is no frame to the far end -- which keeps every frame that comes
 * and reads the latest at its own pace. The link runs a little faster
 * than the sensor's frame rate, timed from its frames at start, so the
 * frames waiting drain between loops: none is dropped while the sensor
 * keeps its time, and were it ever to run ahead, the oldest waiting
 * would go back to unicam.
 *
 * WHERE IN ITS LOOP A FRAME IS comes from when it arrived. A loop starts
 * after the long gap between bursts; unicam's frame sequence counts
 * every frame it saw, the ones it had no buffer for included, so a
 * dropped frame never shifts the count that follows. The first frame
 * after a long gap anchors the count, and a frame's place is its
 * distance from that anchor, modulo the loop. "Long" is measured, not
 * stated: longer than the mean interval between frames -- unicam's
 * count over the time it took, which no one odd frame moves. The
 * sensor's rate is timed between loop starts, past its first loops,
 * which it takes to settle.
 *
 * WHERE THE FAR END IS TOLD rides frame_seq: each loop's first frame
 * takes the next multiple of the loop, so frame_seq modulo the loop IS
 * the place on the wire. Nothing is added to the header for it.
 *
 * A FRAME AND ITS COUNT GO OUT TOGETHER. The payload rides the overlay
 * plane and the header row the primary's buffer beneath it; each frame
 * has a header row of its own, and both planes change in one atomic
 * commit, so the display takes them in the same blank. Written into the
 * buffer already on the wire once the overlay's change returned instead,
 * the header missed that frame's top line and went out with the next:
 * every frame reached the far end with its predecessor's count -- in the
 * same order, none lost, and every phase one place off. */
/* The frame on the wire, those waiting for it, and those unicam holds:
 * it takes the buffer for the frame after next when a frame starts, and
 * without one there a burst's next phase is lost. */
#define LOOP_BUFS_MAX (4 * LOOP_MAX)
/* The default line, in link clocks, when no planner chose one. */
#define LOOP_LINE_CLOCKS 2200
/* The link's frame rate over the sensor's: the frames waiting drain
 * between bursts. */
#define LOOP_LEAD 1.05
/* Long enough to find the bursts and time them before going on the
 * wire. */
#define LOOP_WARMUP_BURSTS 32
#define LOOP_WARMUP_MS 3000
/* The loops the sensor takes to settle, untimed. */
#define LOOP_SETTLE 4

/* A control found by the name V4L2 tools print -- lower case, words
 * joined by underscores -- since a module's private controls have no
 * fixed id. Its id, or 0. */
static uint32_t ctrl_by_name(int fd, const char *want) {
    struct v4l2_queryctrl q = {.id = V4L2_CTRL_FLAG_NEXT_CTRL};
    while (ioctl(fd, VIDIOC_QUERYCTRL, &q) == 0) {
        char name[sizeof q.name];
        size_t i = 0;
        for (; i + 1 < sizeof name && q.name[i]; i++) {
            char ch = (char)q.name[i];
            name[i] = ch == ' ' ? '_'
                    : (ch >= 'A' && ch <= 'Z') ? (char)(ch - 'A' + 'a') : ch;
        }
        name[i] = 0;
        if (!strcmp(name, want)) return q.id;
        q.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    return 0;
}

/* The driver controls the run states, each one the driver must have:
 * 0, or the run refused with why. */
static int set_named_ctrls(int sfd, const struct bridge_args *a) {
    for (int i = 0; i < a->n_ctrls; i++) {
        uint32_t id = ctrl_by_name(sfd, a->ctrl_name[i]);
        if (!id)
            return refuse("the sensor's driver has no control %s",
                          a->ctrl_name[i]);
        if (set_ctrl(sfd, id, a->ctrl_value[i], a->ctrl_name[i]))
            return refuse("the sensor's driver refused %s=%d",
                          a->ctrl_name[i], (int)a->ctrl_value[i]);
    }
    return 0;
}

static struct {
    int vfd;
    /* the loop, and the buffers for it: the frame on the wire, those
     * waiting for it, and those unicam holds -- it takes the buffer for
     * the frame after next when a frame starts, and without one there a
     * burst's next frame is lost. At most `waiting` wait for the wire;
     * past it the oldest goes back. */
    uint32_t loop;
    int bufs, waiting;
    struct dumb buf[LOOP_BUFS_MAX];
    int fd[LOOP_BUFS_MAX];              /* each buffer's dmabuf */
    /* each frame's own header row: the container's base, the header
     * written into it, swapped in with the frame's payload */
    struct dumb hdr[LOOP_BUFS_MAX];
    int blank;                          /* the container's own base on */
    /* the frames waiting for the wire, oldest first: buffer, frame_seq */
    int wait_buf[LOOP_BUFS_MAX];
    uint32_t wait_seq[LOOP_BUFS_MAX];
    int wait_r, wait_n;
    int keeping;                        /* frames kept for the wire */
    /* the arrival timing: the first frame and the latest, by time and by
     * unicam's count, which counts the frames it dropped; the loop's
     * first frame, and the frame_seq it takes */
    int have_prev, anchored;
    uint32_t prev_seq, anchor, anchor_seq, seq_first;
    double prev_t, t_first;
    /* the sensor's rate: the first and latest loop start timed */
    int timed;
    uint32_t seq0_first, seq0_last;
    double t0_first, t0_last;
    /* told, not acted on */
    unsigned long bursts, dropped, idle, misplaced;
} T;

/* A buffer back to unicam, to be written again. */
static void loop_queue(int i) {
    struct v4l2_buffer b = {.index = (uint32_t)i,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_DMABUF,
        .length = (uint32_t)T.buf[i].size};
    b.m.fd = T.fd[i];
    if (ioctl(T.vfd, VIDIOC_QBUF, &b) < 0) die("QBUF");
}

/* One captured frame: placed in its loop, and kept for the wire
 * or handed straight back. */
static void loop_take(const struct v4l2_buffer *b) {
    double t = ts_sec(b->timestamp);
    uint32_t sq = b->sequence;
    int i = (int)b->index;
    if (!T.have_prev) { T.seq_first = sq; T.t_first = t; }
    if (T.have_prev && sq == T.prev_seq + 1 && sq - T.seq_first >= 2 * T.loop) {
        double mean = (t - T.t_first) / (double)(sq - T.seq_first);
        if (t - T.prev_t > mean) {          /* a loop starts */
            if (T.anchored) {
                if ((sq - T.anchor) % T.loop) T.misplaced++;
                /* its first frame takes the next multiple of the loop */
                uint32_t at = T.anchor_seq + (sq - T.anchor) + T.loop - 1u;
                T.anchor_seq = at - at % T.loop;
            }
            T.anchor = sq;
            T.anchored = 1;
            T.bursts++;
            if (T.bursts == LOOP_SETTLE) T.timed = 0;    /* timed from here */
            if (!T.timed) { T.timed = 1; T.seq0_first = sq; T.t0_first = t; }
            T.seq0_last = sq; T.t0_last = t;
        }
    }
    T.have_prev = 1; T.prev_seq = sq; T.prev_t = t;
    if (!T.anchored || !T.keeping || (b->flags & V4L2_BUF_FLAG_ERROR)) {
        loop_queue(i);
        return;
    }
    if (T.wait_n == T.waiting) {      /* the oldest goes back */
        loop_queue(T.wait_buf[T.wait_r]);
        T.wait_r = (T.wait_r + 1) % T.bufs;
        T.wait_n--;
        T.dropped++;
    }
    int at = (T.wait_r + T.wait_n) % T.bufs;
    T.wait_buf[at] = i;
    T.wait_seq[at] = T.anchor_seq + (sq - T.anchor);
    T.wait_n++;
}

/* Every frame unicam has finished, taken. */
static int loop_drain(void) {
    for (;;) {
        struct v4l2_buffer b = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                .memory = V4L2_MEMORY_DMABUF};
        if (ioctl(T.vfd, VIDIOC_DQBUF, &b) < 0)
            return errno == EAGAIN ? 0 : -1;
        loop_take(&b);
    }
}

/* The header row's buffer and the payload's on the wire from the next
 * vertical blank, both in one commit, so the display takes them in the
 * same blank: a frame's count never goes out with another frame's
 * samples. The call returns once the display has taken them: the frame
 * before is off the wire. */
static int show_frame(uint32_t base_fb, uint32_t pay_fb) {
    drmModeAtomicReqPtr req = drmModeAtomicAlloc();
    if (!req) return -1;
    int rc = (drmModeAtomicAddProperty(req, D.primary, D.primary_fb,
                                       base_fb) < 0 ||
              drmModeAtomicAddProperty(req, D.plane, D.plane_fb,
                                       pay_fb) < 0)
        ? -1 : drmModeAtomicCommit(D.dfd, req, 0, NULL);
    drmModeAtomicFree(req);
    return rc;
}

/* Frame `i` on the wire, its count `fs` in its own header row. */
static int loop_scan(int i, uint32_t fs) {
    pack_header(T.hdr[i].map, fs, SOURCE_ID);
    if (show_frame(T.hdr[i].fb_id, T.buf[i].fb_id)) return -1;
    bstate.payload_map = T.buf[i].map;
    base_map_ro = T.hdr[i].map;
    T.blank = 0;
    return 0;
}

static int run_loop(const struct bridge_args *a) {
    CLIP_RUN = 0; LOOP_RUN = a->loop; paused = 0;
    char video_found[288], why[256];
    const char *video = a->video[0] ? a->video : NULL;
    int sfd = -1, streaming = 0, on_plane = 0,
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int rc = RUN_DONE;
    memset(&T, 0, sizeof T);
    T.vfd = -1;
    T.loop = a->loop;
    T.bufs = 4 * (int)a->loop;
    T.waiting = T.bufs - 4;
    for (int i = 0; i < LOOP_BUFS_MAX; i++) T.fd[i] = -1;
    if (!D.primary || !D.primary_fb || !D.plane_fb)
        return refuse("the display has no primary plane to carry a frame's "
                      "header with its payload in one commit");
    if (!video) {
        if (resolve_capture_node(video_found, sizeof video_found))
            return refuse("no CSI-2 receiver node found by name; name one "
                          "with --video");
        video = video_found;
    }
    sfd = open(a->subdev, O_RDWR);
    if (sfd < 0) { rc = refuse("open %s: %s", a->subdev, strerror(errno)); goto out; }
    struct v4l2_subdev_format sf = {.which = V4L2_SUBDEV_FORMAT_ACTIVE};
    if (ioctl(sfd, VIDIOC_SUBDEV_G_FMT, &sf) < 0) {
        rc = refuse("the module will not say its format: %s", strerror(errno));
        goto out;
    }
    fmt_row = -1;
    for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
        if (FORMATS[i].mbus == sf.format.code) { fmt_row = (int)i; break; }
    if (fmt_row < 0) {
        rc = refuse("the sensor sends mediabus code 0x%x, which this build "
                    "does not know", sf.format.code);
        goto out;
    }
    WIDTH = sf.format.width; HEIGHT = sf.format.height;
    if ((rc = set_named_ctrls(sfd, a)) != 0) goto out;

    /* --- the link: the planner's, else at the module's own rate,
     * which is measured once its frames flow ----------------------- */
    uint32_t cw = link_width(video, WIDTH, HEIGHT, FORMATS[fmt_row].bits,
                             FORMATS[fmt_row].v4l2);
    if (!cw) {
        rc = refuse("unicam keeps no stride near a %u-sample line", WIDTH);
        goto out;
    }
    T.vfd = open(video, O_RDWR | O_NONBLOCK);
    if (T.vfd < 0) { rc = refuse("open %s: %s", video, strerror(errno)); goto out; }

    /* The buffers the module's frames land in are the display's: the
     * header row's container first, then one payload buffer a frame. */
    struct container *c = display_buffers(cw, HEIGHT + 1, HEIGHT);
    for (int i = 0; i < T.bufs; i++) {
        T.buf[i] = make_fb(D.dfd, cw, HEIGHT);
        T.hdr[i] = make_fb(D.dfd, cw, HEIGHT + 1);
        if (drmPrimeHandleToFD(D.dfd, T.buf[i].handle,
                               DRM_CLOEXEC | DRM_RDWR, &T.fd[i]))
            die("PrimeHandleToFD");
    }
    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    fmt.fmt.pix.width = WIDTH; fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = FORMATS[fmt_row].v4l2;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = T.buf[0].pitch;
    if (ioctl(T.vfd, VIDIOC_S_FMT, &fmt) < 0 ||
        fmt.fmt.pix.pixelformat != FORMATS[fmt_row].v4l2 ||
        fmt.fmt.pix.width != WIDTH || fmt.fmt.pix.height != HEIGHT) {
        rc = refuse("unicam will not deliver %ux%u %d-bit %s", WIDTH, HEIGHT,
                    FORMATS[fmt_row].bits, FORMATS[fmt_row].order);
        goto out;
    }
    if (fmt.fmt.pix.bytesperline != T.buf[0].pitch) {
        rc = refuse("unicam kept bytesperline %u, need %u: the stride IS "
                    "the bridge, refusing", fmt.fmt.pix.bytesperline,
                    T.buf[0].pitch);
        goto out;
    }
    struct v4l2_requestbuffers req = {.count = (uint32_t)T.bufs,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_DMABUF};
    if (ioctl(T.vfd, VIDIOC_REQBUFS, &req) < 0 || req.count != (uint32_t)T.bufs) {
        rc = refuse("unicam gave %u of %d capture buffers", req.count,
                    T.bufs);
        goto out;
    }
    for (int i = 0; i < T.bufs; i++) loop_queue(i);
    if (ioctl(T.vfd, VIDIOC_STREAMON, &type) < 0) {
        rc = refuse("STREAMON: %s", strerror(errno));
        goto out;
    }
    streaming = 1;

    /* --- the sensor found and timed before anything goes out ------- */
    struct timespec w0, wn;
    clock_gettime(CLOCK_MONOTONIC, &w0);
    for (;;) {
        struct pollfd pf = {.fd = T.vfd, .events = POLLIN};
        poll(&pf, 1, 100);
        if (loop_drain() < 0) die("DQBUF");
        clock_gettime(CLOCK_MONOTONIC, &wn);
        long ms = (wn.tv_sec - w0.tv_sec) * 1000
                + (wn.tv_nsec - w0.tv_nsec) / 1000000;
        if (T.bursts >= LOOP_WARMUP_BURSTS) break;
        if (!running || ms > LOOP_WARMUP_MS) {
            rc = refuse("%lu whole loop(s) in %ld ms (frames %s): no "
                        "bursts of %u to find the loop by", T.bursts, ms,
                        T.have_prev ? "came" : "never came", T.loop);
            goto out;
        }
    }
    double frame_hz = (double)(T.seq0_last - T.seq0_first)
                    / (T.t0_last - T.t0_first);
    double loop_s = T.loop / frame_hz;
    uint32_t link_khz = a->link_khz ? a->link_khz : SINK_KHZ;
    uint32_t line = a->line_clocks ? a->line_clocks : LOOP_LINE_CLOCKS;
    uint32_t frame = a->frame_lines ? a->frame_lines
        : (uint32_t)(link_khz * 1000.0 / (frame_hz * LOOP_LEAD * line));
    if (line < WIDTH || frame < HEIGHT + 10 ||
        derive_raster(WIDTH, HEIGHT, cw, link_khz * 1000.0,
                      (int)line - (int)WIDTH,
                      (int)frame - (int)HEIGHT - LOCK_SPARE, link_khz) < 0) {
        rc = refuse("a %ux%u frame does not fit a %u-clock line and "
                    "%u-line frame at %u kHz", WIDTH, HEIGHT, line, frame,
                    link_khz);
        goto out;
    }
    set_raster(&LINK);
    double link_hz = link_khz * 1000.0 / ((double)LINK.h_total * LINK.v_total);
    printf("bridge: loop of %u %ux%u, every %.3f ms (%.3f frames/s); "
           "link raster %ux%u of %ux%u at %u kHz, %.3f Hz, %+.1f%% on the "
           "sensor%s\n",
           T.loop, WIDTH, HEIGHT, loop_s * 1e3, frame_hz, LINK.h_active,
           LINK.v_active, LINK.h_total, LINK.v_total, link_khz, link_hz,
           (link_hz / frame_hz - 1.0) * 100.0,
           link_hz < frame_hz ? " -- SLOWER: frames will be dropped" : "");

    /* --- on the wire: each frame as it came, at the first blank after
     * it arrived; the header clear until there is one --------------- */
    if (c == &NEW) memset(c->base.map, 0, HEADER_BYTES);
    if (display_show(&MODE, why, sizeof why) < 0) {
        rc = refuse("%s", why);
        goto out;
    }
    memset(ON.base.map, 0, HEADER_BYTES);
    bstate.locked = 1;
    bstate.sensor_fd = sfd;
    run_started();
    T.keeping = 1;
    T.dropped = T.idle = 0;             /* the warm-up's were nobody's */

    /* Each pass is one frame of the link. A frame waiting goes on the
     * wire with its own header row, both taken at the next vertical
     * blank; the one before is then off the wire and goes back to
     * unicam. With none waiting, the container's own base goes back on,
     * its header clear: the frame the display repeats is no frame. */
    int shown = -1;
    uint32_t frames = 0;
    while (running && !time_up()) {
        if (reconfig_pending) { rc = RUN_RECONFIGURE; break; }
        if (loop_drain() < 0) {
            fprintf(stderr, "bridge: DQBUF: %s\n", strerror(errno));
            break;
        }
        if (T.wait_n && !paused) {
            int i = T.wait_buf[T.wait_r];
            uint32_t fs = T.wait_seq[T.wait_r];
            T.wait_r = (T.wait_r + 1) % T.bufs;
            T.wait_n--;
            if (loop_scan(i, fs)) {
                fprintf(stderr, "bridge: atomic commit: %s\n",
                        strerror(errno));
                loop_queue(i);
                break;
            }
            on_plane = 1;
            if (shown >= 0) loop_queue(shown);
            shown = i;
            bstate.seq = fs;
        } else {
            int bad = T.blank || shown < 0
                ? wait_vblank()
                : show_frame(ON.base.fb_id, T.buf[shown].fb_id);
            if (bad < 0) {
                fprintf(stderr, "bridge: idle blank: %s\n", strerror(errno));
                break;
            }
            T.blank = 1;
            base_map_ro = ON.base.map;
            T.idle++;
        }
        frames++;
        struct bridge_frame bf = {.seq = frames, .locked = 1, .sensor_fd = sfd};
        bridge_frame_hook(&bf);
        if ((frames & 31) == 0) link_watch(0);
        if (frames % 3600 == 0)
            printf("bridge: loop %lu loops, %lu frames dropped, %lu "
                   "blanks idle, %lu out of place\n", T.bursts, T.dropped,
                   T.idle, T.misplaced);
        ctl_poll();
    }
    printf("bridge: loop %u link frames; %lu loops, %lu frames "
           "dropped, %lu blanks idle, %lu out of place\n", frames, T.bursts,
           T.dropped, T.idle, T.misplaced);
out:
    /* The loop's buffers leave the display before they are freed: the
     * container's own base and payload back in one commit, its header
     * clear, so the frame that shows them is no frame to the far end. */
    if (on_plane) {
        show_frame(ON.base.fb_id, ON.pay.fb_id);
        bstate.payload_map = ON.pay.map;
        base_map_ro = ON.base.map;
    }
    if (streaming) ioctl(T.vfd, VIDIOC_STREAMOFF, &type);
    if (T.vfd >= 0) {
        struct v4l2_requestbuffers none = {.count = 0,
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_DMABUF};
        ioctl(T.vfd, VIDIOC_REQBUFS, &none);
        close(T.vfd);
    }
    for (int i = 0; i < T.bufs; i++) {
        if (T.fd[i] >= 0) close(T.fd[i]);
        free_fb(&T.buf[i]);
        free_fb(&T.hdr[i]);
    }
    if (sfd >= 0) close(sfd);
    bstate.sensor_fd = -1;
    return rc;
}

/* A SENSOR RUN: the sensor in the mode asked for (or the one it is in),
 * its line locked to the link's and its frame start held by the genlock.
 * It ends cleanly for a reconfiguration: the next capture is sent to the
 * spare, so the one already in flight finishes the displayed buffer --
 * the last frame on the wire is a whole sensor frame, held until the next
 * run draws over it -- and only then does the sensor stop. */
static int run_sensor(const struct bridge_args *a) {
    CLIP_RUN = 0; LOOP_RUN = 0; paused = 0;
    order_given = a->order_given;
    const char *want_order = a->order_given ? a->order : NULL;
    unsigned want_w = a->want_w, want_h = a->want_h;
    unsigned frame_lines = a->frame_lines, line_ns = a->line_ns;
    int margin_lines = a->margin_lines;
    uint32_t link_khz = a->link_khz ? a->link_khz : SINK_KHZ;
    char video_found[288];
    const char *video = a->video[0] ? a->video : NULL;
    int sfd = -1, vfd = -1, dmabuf = -1, streaming = 0, type =
        V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int rc = RUN_DONE;
    char why[256];
    /* Not given: find it by NAME. Refused rather than guessed when no
     * known receiver is present -- a guess streams from whatever
     * enumerated into that slot, and a UVC capture card answers the
     * open perfectly happily. */
    if (!video) {
        if (resolve_capture_node(video_found, sizeof video_found))
            return refuse("no CSI-2 receiver node found by name (looked for "
                          "unicam-image, rp1-cfe-csi2_ch0 in "
                          "/sys/class/video4linux). Name one with --video if "
                          "this Pi presents another.");
        video = video_found;
        printf("bridge: sensor node %s (found by name, not number)\n", video);
    }

    /* --- camera --------------------------------------------------- */
    sfd = open(a->subdev, O_RDWR);
    if (sfd < 0) {
        rc = refuse("open %s: %s", a->subdev, strerror(errno));
        goto out;
    }
    if ((rc = set_named_ctrls(sfd, a)) != 0) goto out;
    /* The mode asked for, or -- none asked -- the one the sensor is in:
     * this file has no size of its own to offer it. */
    struct v4l2_subdev_format sf = {
        .which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = 0,
        .format = {.width = want_w, .height = want_h,
                   .code = FORMATS[fmt_row].mbus,
                   .field = V4L2_FIELD_NONE}};
    if (want_w ? ioctl(sfd, VIDIOC_SUBDEV_S_FMT, &sf) < 0
               : ioctl(sfd, VIDIOC_SUBDEV_G_FMT, &sf) < 0) {
        rc = refuse("the sensor refused %ux%u: %s", want_w, want_h,
                    strerror(errno));
        goto out;
    }
    WIDTH = sf.format.width;
    HEIGHT = sf.format.height;

    /* ...and now ASK, rather than assume it took what was offered.
     * The answer settles the geometry, the mosaic order and the depth
     * in one go -- three constants this file used to declare, and each
     * of which has cost a bench day at least once when the declaration
     * and the sensor disagreed. An explicit --order still wins, since
     * an operator correcting a driver is the case that flag exists
     * for. */
    {
        uint32_t dw = WIDTH, dh = HEIGHT;
        int drow = fmt_row;
        if (discover_sensor(sfd, &dw, &dh, &drow)) {
            WIDTH = dw;
            HEIGHT = dh;
            /* The row settles all three names at once: what unicam is
             * asked to deliver, and what the header tells the receiver
             * to unpack. An explicit --order still wins, but only the
             * ORDER -- the depth is the sensor's to state. */
            if (order_given && want_order) {
                int r = format_row(want_order, FORMATS[drow].bits);
                if (r < 0) {
                    rc = refuse("no %s format at %d bits", want_order,
                                FORMATS[drow].bits);
                    goto out;
                }
                fmt_row = r;
            } else {
                fmt_row = drow;
            }
            printf("bridge: sensor says %ux%u %s %d-bit%s\n", WIDTH, HEIGHT,
                   FORMATS[fmt_row].order, FORMATS[fmt_row].bits,
                   order_given ? " (order overridden on the command line)"
                               : "");
        }
    }

    /* The link raster, from the sensor's own timing in this mode: its
     * pixel rate and the blanking its driver states as the mode's -- its
     * defaults, never the values in force, which are whatever the last
     * run left (reading those, each restart lengthened the line by the
     * rounding of the one before). */
    {
        int32_t hb = 0, vb = 0;
        double pr = (double)get_pixel_rate(sfd);
        if (ctrl_default(sfd, V4L2_CID_HBLANK, &hb) < 0 ||
            ctrl_default(sfd, V4L2_CID_VBLANK, &vb) < 0 || pr <= 0) {
            rc = refuse("the sensor does not state its pixel rate and "
                        "blanking, so its line cannot be carried");
            goto out;
        }
        if (line_ns) {
            /* the longest sensor line that does not exceed the time asked:
             * the link's line, rounded up from it, then lands on it */
            int asked = (int)(line_ns * 1e-9 * pr) - (int)WIDTH;
            if (asked < hb) {
                rc = refuse("%u ns is shorter than the sensor's shortest "
                            "%ux%u line", line_ns, WIDTH, HEIGHT);
                goto out;
            }
            hb = asked;
        }
        if (frame_lines) {
            int asked = (int)frame_lines - (int)HEIGHT - LOCK_SPARE;
            if (asked < vb) {
                rc = refuse("%u lines is shorter than the sensor's shortest "
                            "%ux%u frame (%u lines)", frame_lines, WIDTH,
                            HEIGHT, HEIGHT + (unsigned)vb + LOCK_SPARE);
                goto out;
            }
            vb = asked;
        }
        /* the receiver's rate: a line of WIDTH samples lasts at least
         * WIDTH link clocks, or its output falls behind the link */
        int rate_hb = (int)ceil(WIDTH * pr / (link_khz * 1000.0)) - (int)WIDTH;
        if (hb < rate_hb) {
            printf("bridge: the sensor's %ux%u line is stretched to %u "
                   "pixels, the %u link clocks its samples take\n", WIDTH,
                   HEIGHT, WIDTH + (unsigned)rate_hb, WIDTH);
            hb = rate_hb;
        }
        uint32_t cw = link_width(video, WIDTH, HEIGHT, FORMATS[fmt_row].bits,
                                 FORMATS[fmt_row].v4l2);
        if (!cw) {
            rc = refuse("unicam keeps no stride near a packed %u-sample line",
                        WIDTH);
            goto out;
        }
        if (derive_raster(WIDTH, HEIGHT, cw, pr, hb, vb, link_khz) < 0) {
            rc = refuse("a %ux%u frame with %d+%d blanking does not fit a "
                        "link at %u kHz", WIDTH, HEIGHT, hb, vb, link_khz);
            goto out;
        }
        set_raster(&LINK);
        printf("bridge: link raster %ux%u of %ux%u at %u kHz, %u Hz -- "
               "the sensor's\n", LINK.h_active, LINK.v_active, LINK.h_total,
               LINK.v_total, link_khz, LINK.hz);
    }

    /* the container: link lines as wide as a packed sensor line, on the
     * wire before the sensor is told where to write */
    display_buffers(LINK.h_active, VDISP, HEIGHT);
    if (display_show(&MODE, why, sizeof why) < 0) {
        rc = refuse("%s", why);
        goto out;
    }
    if (drmPrimeHandleToFD(D.dfd, ON.pay.handle, DRM_CLOEXEC | DRM_RDWR,
                           &dmabuf))
        die("PrimeHandleToFD");

    if (set_ctrl(sfd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "ae") ||
        set_ctrl(sfd, V4L2_CID_AUTOGAIN, 0, "agc") ||
        set_ctrl(sfd, V4L2_CID_AUTO_WHITE_BALANCE, 0, "awb") ||
        set_ctrl(sfd, V4L2_CID_TEST_PATTERN, a->test_pattern, "pattern")) {
        rc = refuse("the sensor refused a control (see above)");
        goto out;
    }

    /* LINE LOCK, not just frame lock. The sensor's HTS is writable
     * and OBEYS -- verified 2026-08-19, fps following hblank to
     * three digits in two modes. So make the sensor's line time
     * EQUAL the display's, and its VTS equal the display's vtotal:
     * the two rasters then run stride for stride and the write
     * beam's lead is CONSTANT down the frame (with unequal line
     * times it grows ~75 lines top to bottom, and the margin must
     * be budgeted at the tightest point). All derived from the
     * sensor's own pixel rate and the mode table, not tabulated. */
    double pixel_rate = (double)get_pixel_rate(sfd);
    double t_frame_d = (double)MODE.htotal * MODE.vtotal / (MODE.clock * 1e3);
    double t_row_d = t_frame_d / MODE.vtotal;
    int hblank_nom = (int)(t_row_d * pixel_rate + 0.5) - WIDTH;
    int vblank_fix = MODE.vtotal - HEIGHT;
    double t_row_s = (WIDTH + hblank_nom) / pixel_rate;
    /* The integration time is not ours to set, but the RASTER IT MUST
     * FIT is: an exposure beyond VTS silently stretches the frame and
     * the line lock goes with it (the 16fps surprise of 2026-08-18).
     * Only this file knows that bound, because it just computed the
     * blanking that defines it -- so it is PUBLISHED and complained
     * about, never overwritten. Whoever owns the value respects it. */
    int exp_bound = HEIGHT + vblank_fix - 8;
    EXP_BOUND = exp_bound;
    printf("bridge: integration must fit %d lines (VTS bound); the value "
           "is the sensor package's\n", exp_bound);
    {
        int32_t in_force = -1;
        read_ctrl(sfd, V4L2_CID_EXPOSURE, &in_force);
        if (in_force > exp_bound)
            fprintf(stderr, "bridge: sensor is at exposure %d, past the %d "
                            "line bound -- frames will stretch and the lock "
                            "will drift until the sensor package sets a "
                            "value that fits\n", in_force, exp_bound);
    }
    if (set_ctrl(sfd, V4L2_CID_VBLANK, vblank_fix, "vblank") ||
        set_ctrl(sfd, V4L2_CID_HBLANK, hblank_nom, "hblank")) {
        rc = refuse("the sensor refused the line lock's blanking");
        goto out;
    }

    vfd = open(video, O_RDWR | O_NONBLOCK);
    if (vfd < 0) die("open video");
    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    fmt.fmt.pix.width = WIDTH; fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = FORMATS[fmt_row].v4l2;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = ON.pay.pitch;
    if (ioctl(vfd, VIDIOC_S_FMT, &fmt) < 0) {
        rc = refuse("unicam refused %ux%u: %s", WIDTH, HEIGHT,
                    strerror(errno));
        goto out;
    }
    if (fmt.fmt.pix.bytesperline != ON.pay.pitch) {
        rc = refuse("unicam kept bytesperline %u, need %u: the stride IS "
                    "the bridge, refusing", fmt.fmt.pix.bytesperline,
                    ON.pay.pitch);
        goto out;
    }

    /* TWO buffer slots, ONE buffer: unicam pipelines -- it wants the
     * next slot queued before the current frame ends, and a single
     * slot loses every other frame (592 of 1200, measured). Both
     * slots import the same dmabuf, so every frame still lands in
     * the one beam-racing BO. */
    struct v4l2_requestbuffers req = {
        .count = CAPTURE_BUFS, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_DMABUF};
    if (ioctl(vfd, VIDIOC_REQBUFS, &req) < 0) die("REQBUFS");

    for (uint32_t i = 0; i < CAPTURE_BUFS; i++) {
        struct v4l2_buffer buf = {
            .index = i, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_DMABUF, .length = (uint32_t)ON.pay.size};
        buf.m.fd = dmabuf;
        if (ioctl(vfd, VIDIOC_QBUF, &buf) < 0) die("QBUF");
    }
    if (ioctl(vfd, VIDIOC_STREAMON, &type) < 0) {
        rc = refuse("STREAMON: %s", strerror(errno));
        goto out;
    }
    streaming = 1;

    printf("bridge: %dx%d pGAA, pitch %u, LINE-LOCKED hblank %d "
           "vblank %d, row %.3fus(s)/%.3fus(d), margin %d lines\n",
           WIDTH, HEIGHT, ON.pay.pitch, hblank_nom, vblank_fix,
           t_row_s * 1e6, t_row_d * 1e6, margin_lines);

    bstate.sensor_fd = sfd;
    /* the module hears this run's mode before the first frame */
    run_started();

    /* --- the loop: DQBUF, header, genlock, QBUF -------------------- */
    /* WHERE the margin lives: the writer's line time is shorter than
     * the reader's, so its lead GROWS down the frame -- the minimum is
     * at the TOP. A margin at the frame END would let the reader start
     * ahead and be overtaken mid-frame: a locked loop then pins a
     * stationary tear seam around row 750. So the setpoint is on the
     * frame START: writer begins `margin` display lines before the
     * reader's vblank, phrased through the frame-END time because
     * that is the event we can stamp. Top-of-frame latency is then
     * margin+2 display lines -- under a millisecond. */
    /* The margin is ADAPTIVE -- derived continuously from the
     * loop's own telemetry instead of measured once and tabulated.
     * An asymmetric ratchet on three timescales: EXPAND within a
     * frame when the filtered beam comes within 4 lines of eating
     * the margin (a close call is acted on before it can repeat);
     * SHRINK a line a minute, only while the margin carries 4+
     * lines over the worst excursion of the last half hour; and
     * every setpoint move slews at 1 line/frame -- inside the
     * winsorizer's +/-3 band, so the filter state needs no surgery
     * and the vblank knob carries it as ordinary trims. Floor 4
     * (structural 2 + 2), ceiling 24. On a quiet Pi 5 this settles
     * near the floor; on a loaded Zero it rides higher; nobody
     * re-runs a bench procedure. */
    double margin = margin_lines, margin_goal = margin;
    double target = HEIGHT * t_row_s - margin * t_row_d;
    double bucket_max[6] = {0}; int bucket_i = 0, m_cool = 0;
    double integ = 0, lines_f = 0; int vblank_cmd = vblank_fix, locked = 0;
    /* The margin question, answered by measurement instead of by
     * choosing: record the RAW phase excursion once locked -- the
     * unfiltered tail is what eats margin, the EMA is cosmetics --
     * and the counts over 1/2/5 lines give the tail its shape. */
    double raw_max = -1e9, raw_min = 1e9;
    long n_gt1 = 0, n_gt2 = 0, n_gt5 = 0, n_stat = 0;
    /* Outlier handling: the self-stamp's noise is FAT-TAILED -- a
     * fifth of wakeups land more than a line late on this Pi, and
     * rare ones land half a frame away. Hard rejection hurts both
     * ways it can be built: judged against stillness it goes deaf
     * to the loop's own slews (measured: 89% rejected, limit
     * cycle); judged against prediction with a streak-escape, the
     * escapes let half-frame liars yank the filter and the loop
     * then steers the REAL beam into 18-line excursions. So:
     * WINSORIZE. Every sample is clamped to the dead-reckoned
     * prediction +/-3 lines -- the loop is never deaf (a true step
     * walks it over at 3 lines a frame) and no liar can pull harder
     * than 3 lines, once. The verdict reads the FILTERED
     * trajectory: the beam integrates, it cannot teleport. */
    long n_win = 0;
    double pred = 0; int have_pred = 0;
    double flt_max = -1e9, flt_min = 1e9;
    uint32_t seq = 0; time_t t0 = time(NULL), last_note = 0;
    time_t last_verdict = time(NULL);
    time_t bucket_t0 = t0, last_shrink = t0;
    double vb_eq = vblank_fix;
    /* a reconfiguration asked: 1 once the next capture went to the spare,
     * so the one in flight is the displayed buffer's last */
    int stopping = 0;
    while (running && !time_up()) {
        struct pollfd pf = {.fd = vfd, .events = POLLIN};
        if (poll(&pf, 1, 2000) <= 0) {
            fprintf(stderr, "bridge: no frame for 2s\n");
            // The control socket must survive a stalled camera. It used
            // to be serviced only at the END of a frame, so a sensor
            // that stopped delivering made the bridge deaf as well as
            // blind: connections queued until the backlog filled and
            // callers got EAGAIN from connect(), which reads as "the
            // bridge is gone" exactly when someone is trying to ask it
            // what is wrong.
            ctl_poll();
            /* no frame is in flight to finish */
            if (reconfig_pending) { rc = RUN_RECONFIGURE; goto out; }
            continue;
        }
        struct v4l2_buffer dq = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_DMABUF};
        dq.m.fd = dmabuf;
        if (ioctl(vfd, VIDIOC_DQBUF, &dq) < 0) { perror("DQBUF"); break; }
        /* the last frame into the displayed buffer is in: it stays on
         * the wire, whole, until the next run draws over it */
        if (stopping) {
            wait_vblank();
            rc = RUN_RECONFIGURE;
            goto out;
        }
        /* Self-stamp at the DQBUF wakeup. This unicam SAYS its stamps
         * are monotonic (flags 0x2000) and then delivers zero -- read
         * raw off the bench, 2026-08-19. poll() wakes at frame-done,
         * so under SCHED_FIFO our own clock IS the frame-end time to
         * within scheduling jitter; the loop filter absorbs that. */
        struct timespec dqts;
        clock_gettime(CLOCK_MONOTONIC, &dqts);
        double t_now = dqts.tv_sec + dqts.tv_nsec * 1e-9;
        seq++;
        pack_header(ON.base.map, seq, SOURCE_ID);
        /* Where the NEXT frame lands: the displayed buffer, or the
         * scratch one while paused -- or once a reconfiguration is
         * waiting, so the capture already in flight is the displayed
         * buffer's last. Both slots may carry either fd -- V4L2 takes
         * the dma-buf at queue time, so a pause is one assignment and
         * never a STREAMOFF. */
        if (reconfig_pending && !paused) {
            if (scratch_fd < 0) { rc = RUN_RECONFIGURE; goto out; }
            stopping = 1;
        }
        const int sensor_shown = !paused && !stopping;
        dq.m.fd = (!sensor_shown && scratch_fd >= 0) ? scratch_fd : dmabuf;
        if (ioctl(vfd, VIDIOC_QBUF, &dq) < 0) { perror("QBUF"); break; }
        /* paused, the displayed buffer is not the sensor's: nothing is in
         * flight to it, so the frame on the wire is whole already */
        if (reconfig_pending && paused) {
            wait_vblank();
            rc = RUN_RECONFIGURE;
            goto out;
        }

        /* PHASE: sensor frame-end against the last display vblank,
         * both on the kernel's monotonic clock. */
        /* The CRTC is not necessarily index 0, and a vblank request
         * names its CRTC in the HIGH_CRTC bits -- silently absent
         * reports, not an error code, is how that one shows itself. */
        drmVBlank vb = {.request = {
            .type = DRM_VBLANK_RELATIVE
                  | ((D.crtc_index << DRM_VBLANK_HIGH_CRTC_SHIFT)
                     & DRM_VBLANK_HIGH_CRTC_MASK),
            .sequence = 0}};
        int vbrc = drmWaitVBlank(D.dfd, &vb);
        if (vbrc && seq == 1)
            fprintf(stderr, "WaitVBlank: %s (genlock disabled)\n",
                    strerror(errno));
        if (vbrc == 0) {
            int bad = 0;
            int stamped = dq.timestamp.tv_sec || dq.timestamp.tv_usec;
            double t_dq = stamped ? to_mono(ts_sec(dq.timestamp), &bad)
                                  : t_now;
            double t_vb = to_mono((double)vb.reply.tval_sec
                        + (double)vb.reply.tval_usec * 1e-6, &bad);
            if (seq == 1)
                printf("bridge: frame time from %s, vblank age %.4fs%s\n",
                       stamped ? "driver stamp" : "DQBUF wakeup (self)",
                       t_now - t_vb,
                       bad ? " -- UNIDENTIFIED CLOCK, genlock blind" : "");
            if (bad) continue;
            double err = fmod(t_dq - t_vb - target, t_frame_d);
            if (err > t_frame_d / 2) err -= t_frame_d;
            if (err < -t_frame_d / 2) err += t_frame_d;
            double lines = err / t_row_s;
            double wrap = t_frame_d / t_row_s;
            double v_cmd = (vblank_cmd - vblank_fix) * t_row_s / t_row_d;
            if (have_pred) {
                pred += v_cmd;
                if (pred > wrap / 2) pred -= wrap;
                if (pred < -wrap / 2) pred += wrap;
            }
            double used = lines;
            if (have_pred) {
                if (used > pred + 3.0) { used = pred + 3.0; n_win++; }
                if (used < pred - 3.0) { used = pred - 3.0; n_win++; }
            }
            pred = used; have_pred = 1;
            lines_f = (seq < 5) ? used : lines_f * 0.9 + used * 0.1;
            if (locked) {
                if (lines_f > flt_max) flt_max = lines_f;
                if (lines_f < flt_min) flt_min = lines_f;
                /* the ratchet's memory: worst positive excursion,
                 * in six 5-minute buckets = a 30-minute window */
                if (time(NULL) - bucket_t0 >= 300) {
                    bucket_t0 = time(NULL);
                    bucket_i = (bucket_i + 1) % 6;
                    bucket_max[bucket_i] = 0;
                }
                if (lines_f > bucket_max[bucket_i])
                    bucket_max[bucket_i] = lines_f;
                double worst = 0;
                for (int k = 0; k < 6; k++)
                    if (bucket_max[k] > worst) worst = bucket_max[k];
                /* Expansion judges the beam only from a SETTLED
                 * loop: while the setpoint slews, every reading
                 * carries the slew itself, and judging it expands
                 * again -- the first launch rode that feedback
                 * straight to the ceiling. So: no expansion during
                 * a slew, and a 3-second cooldown after every step
                 * for the PI to re-zero. */
                if (m_cool > 0) m_cool--;
                if (m_cool == 0 && margin == margin_goal
                    && lines_f > margin - 4)
                    margin_goal = lines_f + 4;      /* expand, now   */
                if (time(NULL) - last_shrink >= 60) {
                    last_shrink = time(NULL);
                    if (margin_goal > worst + 4 && margin_goal > 4)
                        margin_goal -= 1;           /* shrink, gently */
                }
                if (margin_goal < 4) margin_goal = 4;
                if (margin_goal > 24) margin_goal = 24;
                if (margin < margin_goal) { margin += 1; m_cool = 90; }
                else if (margin > margin_goal) { margin -= 1; m_cool = 90; }
                target = HEIGHT * t_row_s - margin * t_row_d;
            }
            /* A slow PI on the VBLANK knob, and only that knob:
             * an HTS write is an I2C transaction landing WHEREVER
             * the bus finishes -- mid-readout, on a sensor with no
             * group hold, it glitches the row it lands on, one
             * corrupted line per dither step at a random height
             * (seen on the TV within the hour when hblank was the
             * knob). A VBLANK write only means anything inside
             * blanking, so it is glitch-free BY CONSTRUCTION. Hence
             * the split: hblank sets the RATE once (line lock,
             * never touched while streaming), vblank trims the
             * PHASE. A LATE write beam needs a FASTER sensor: the
             * trim subtracts. */
            integ = integ * 0.995 + used;
            int adj = (int)(0.30 * used + 0.02 * integ);
            if (adj > 20) adj = 20;
            if (adj < -20) adj = -20;
            int cmd = vblank_fix - adj;
            if (cmd < 24) cmd = 24;
            if (cmd != vblank_cmd) {
                if (set_ctrl(sfd, V4L2_CID_VBLANK, cmd, "vblank")) break;
                vblank_cmd = cmd;
            }
            /* where the trim SETTLES is telemetry: a persistent
             * equilibrium away from nominal means the boot-time
             * hblank rate estimate has aged (thermals) -- logged,
             * and re-derived at the next stream start, never
             * touched live. */
            vb_eq = vb_eq * 0.999 + cmd * 0.001;
            if (locked) {
                n_stat++;
                if (lines > raw_max) raw_max = lines;
                if (lines < raw_min) raw_min = lines;
                double a = fabs(lines);
                if (a > 1) n_gt1++;
                if (a > 2) n_gt2++;
                if (a > 5) n_gt5++;
            }
            locked = locked || (lines_f > -3 && lines_f < 3);
            if (time(NULL) - last_note >= 5) {
                last_note = time(NULL);
                printf("bridge: seq %u (%.1f fps), phase %+.2f lines, "
                       "vblank %d, margin %.0f, %s raw[%+.2f..%+.2f] "
                       ">1:%ld >2:%ld >5:%ld of %ld\n", seq,
                       seq / (double)(time(NULL) - t0 ? : 1),
                       lines_f, vblank_cmd, margin,
                       locked ? "LOCKED" : "acquiring",
                       raw_min, raw_max, n_gt1, n_gt2, n_gt5, n_stat);
                if (vb_eq - vblank_fix > 3 || vblank_fix - vb_eq > 3)
                    printf("bridge: vblank equilibrium %.1f -- hblank "
                           "rate estimate aging\n", vb_eq);
                if (n_win)
                    printf("bridge: beam flt[%+.2f..%+.2f], %ld stamps "
                           "winsorized\n", flt_min, flt_max, n_win);
                /* Under a supervisor there is no exit to print the
                 * soak verdict at, so it testifies hourly instead:
                 * an unbounded run must still report, or the bound
                 * was the only thing making it honest. */
                if (n_stat && time(NULL) - last_verdict >= 3600) {
                    last_verdict = time(NULL);
                    printf("bridge: HOURLY over %ld locked frames: "
                           "BEAM (filtered) [%+.2f..%+.2f] lines; "
                           "margin %.0f (goal %.0f); raw "
                           "[%+.2f..%+.2f], >1/2/5: %ld/%ld/%ld, "
                           "%ld winsorized.\n",
                           n_stat, flt_min, flt_max, margin,
                           margin_goal, raw_min, raw_max,
                           n_gt1, n_gt2, n_gt5, n_win);
                }
            }
        }
        bstate.seq = seq;
        bstate.phase = lines_f;
        bstate.locked = locked;
        bstate.margin = margin;
        bstate.vblank = vblank_cmd;
        struct bridge_frame bf = {
            .seq = seq, .phase_lines = lines_f,
            .locked = locked, .sensor_fd = sfd};
        bridge_frame_hook(&bf);
        /* once a second: cheap, and a replug never waits long */
        if ((seq & 31) == 0) link_watch(0);
        /* every frame: an accept() that mostly returns EAGAIN */
        ctl_poll();
    }
    if (n_stat)
        printf("bridge: SOAK VERDICT over %ld locked frames: BEAM "
               "(filtered) [%+.2f..%+.2f] lines; adaptive margin "
               "settled at %.0f (goal %.0f). Stamps (raw) "
               "[%+.2f..%+.2f], beyond 1/2/5 lines: %ld/%ld/%ld, "
               "%ld winsorized.\n",
               n_stat, flt_min, flt_max, margin, margin_goal,
               raw_min, raw_max, n_gt1, n_gt2, n_gt5, n_win);
    printf("bridge: %u frames\n", seq);
out:
    if (streaming) ioctl(vfd, VIDIOC_STREAMOFF, &type);
    if (vfd >= 0) close(vfd);
    if (dmabuf >= 0) close(dmabuf);
    if (sfd >= 0) close(sfd);
    bstate.sensor_fd = -1;
    return rc;
}

int main(int argc, char **argv) {
    /* NO exposure, NO gain. They belong to the sensor package, where
     * the AE loop lives; this file knows GEOMETRY -- the raster, the
     * line lock, the blanking that makes them equal. Both were written
     * here at startup anyway, and gain=200 is 12.5x on an ov5647: the
     * raw arrived 55% saturated and every stage downstream was blamed
     * in turn for highlights lost at the sensor. This file's own
     * control socket had been refusing callers on exactly that ground
     * ("the AE loop's to write, not a caller's") while main() did it. */
    struct bridge_args a;
    char why[256];
    int parsed = parse_args(argc - 1, argv + 1, &a, why, sizeof why);
    if (parsed < 0) {
        fprintf(stderr, "bridge: %s\n", why);
        return parsed == -2 ? 2 : 1;
    }
    crc_init();
    setvbuf(stdout, NULL, _IOLBF, 0);   /* notes must land, not pool */
    if (a.selftest) {
        /* `make check` compares these bytes with the Python reference:
         * the header layout has ONE owner and this copy is verified
         * against it, never trusted. */
        uint8_t h[48];
        uint32_t seqs[3] = {7, 0, 123456};
        /* the reference vectors' frame, not a default of this file's */
        WIDTH = 1920; HEIGHT = 1080;
        for (int i = 0; i < 3; i++) {
            pack_header(h, seqs[i], SOURCE_ID);
            for (int b = 0; b < 48; b++) printf("%02x", h[b]);
            printf("\n");
        }
        /* Every row's bayerlink fourcc, and every crop shift, checked
         * against the codec that defines them -- the table is proven
         * rather than believed. */
        for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
            printf("fmt %s %d %c%c%c%c\n", FORMATS[i].order, FORMATS[i].bits,
                   (char)(FORMATS[i].bayerlink & 0xff),
                   (char)((FORMATS[i].bayerlink >> 8) & 0xff),
                   (char)((FORMATS[i].bayerlink >> 16) & 0xff),
                   (char)((FORMATS[i].bayerlink >> 24) & 0xff));
        for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
            for (unsigned dy = 0; dy < 2 && strcmp(FORMATS[i].order, "MONO");
                 dy++)
                for (unsigned dx = 0; dx < 2; dx++)
                    printf("crop %s %u %u %s\n", FORMATS[i].order, dx, dy,
                           FORMATS[format_after_crop((int)i, dx, dy)].order);
        return 0;
    }
    /* After the selftest, deliberately: that block's stdout is compared
     * byte-for-byte against the Python reference, so anything printed
     * ahead of it is a failure of `make check`. The selftest needs no
     * device -- it proves the header layout, not the hardware. */
    SECONDS = a.seconds;
    T0 = time(NULL);
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    /* Bound the tails: this is why the shipping twin is C. Refusal to
     * grant either is a warning, not an exit -- a dev box still works. */
    struct sched_param sp = {.sched_priority = 10};
    if (sched_setscheduler(0, SCHED_FIFO, &sp))
        fprintf(stderr, "note: no SCHED_FIFO (%s)\n", strerror(errno));
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
        fprintf(stderr, "note: no mlockall (%s)\n", strerror(errno));

    /* --- display: opened once, held to the end -------------------- */
    if (open_display(&D.dfd, &D.conn, &D.crtc, &D.crtc_index)) return 1;
    D.plane = overlay_plane(D.dfd, D.crtc_index);
    D.primary = find_plane(D.dfd, D.crtc_index, DRM_PLANE_TYPE_PRIMARY, 0);
    D.primary_fb = D.primary ? plane_prop(D.dfd, D.primary, "FB_ID") : 0;
    D.plane_fb = plane_prop(D.dfd, D.plane, "FB_ID");
    SINK_KHZ = sink_link_khz();
    ctl_open();

    /* --- runs: the command line's, then each reconfiguration -------- */
    struct bridge_args prev;
    int have_prev = 0;
    for (;;) {
        /* What the sensor's own module states of it, where the run
         * names none: how its frames come, what its driver is set to. */
        struct bridge_args run = a;
        if (!run.clip[0]) {
            struct bridge_source src = {0};
            bridge_source_hook(&src);
            if (!run.loop) run.loop = src.loop;
            for (int i = 0; !a.n_ctrls && i < src.n_controls &&
                            i < CTRLS_MAX; i++) {
                snprintf(run.ctrl_name[i], sizeof run.ctrl_name[0], "%s",
                         src.controls[i].name);
                run.ctrl_value[i] = src.controls[i].value;
                run.n_ctrls = i + 1;
            }
            if (run.loop && (run.loop < 2 || run.loop > LOOP_MAX)) {
                fprintf(stderr, "bridge: the sensor's module states a "
                        "loop of %u; 2 to %d are carried\n", run.loop,
                        LOOP_MAX);
                return 1;
            }
        }
        int r = run.clip[0] ? run_clip(&run) : run.loop ? run_loop(&run)
                                                 : run_sensor(&run);
        if (r == RUN_RECONFIGURE) {
            prev = a; have_prev = 1;
            a = PENDING;
            reconfig_pending = 0;
            continue;
        }
        if (r == RUN_REFUSED && have_prev) {
            reconfig_answer("err %s -- the previous configuration runs again",
                            WHY);
            a = prev; have_prev = 0;
            continue;
        }
        if (r == RUN_REFUSED) return 1;
        break;
    }
    reconfig_answer("err the bridge is stopping");
    container_free(&NEW);
    container_free(&ON);
    close(D.dfd);
    return 0;
}
