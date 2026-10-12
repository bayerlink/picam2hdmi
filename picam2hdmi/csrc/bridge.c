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
 * This is the shipping twin of picam2hdmi/bridge.py, which remains the
 * reference implementation -- same architecture, proven first (colour
 * bars through the whole chain, 2026-08-19). C because the kernel ABI
 * structs come from the kernel's own headers instead of hand-built
 * copies, because a single static-ish binary is the appliance shape,
 * and because SCHED_FIFO + mlockall bound the tails Python only makes
 * unlikely. `make check` holds the two implementations byte-identical
 * where they must agree: the header.
 *
 * WHAT THIS ADDS over the Python twin: the genlock loop. The sensor
 * and the display run on crystals that owe each other nothing;
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
#include "rasters.h"
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
static uint32_t WIDTH  = 1920;        /* the sensor's, until asked   */
static uint32_t HEIGHT = 1080;
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
    int phase = order_phase(FORMATS[row].order)
                ^ (int)((y0 & 1u) << 1) ^ (int)(x0 & 1u);
    for (unsigned i = 0; i < sizeof FORMATS / sizeof *FORMATS; i++)
        if (FORMATS[i].bits == FORMATS[row].bits &&
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
#define SOURCE_ID_OV5647 0

/* The modeline scanned out: the chosen raster VERBATIM, claiming one
 * more active line than it has -- the header row -- inside its own
 * vertical blanking (kms.py's container: both ends are ours, and the
 * receiver counts data-enable rather than reading mode names). */
static drmModeModeInfo MODE;
static struct bridge_mode BMODE;

/* The raster a WxH frame at HZ rides: one whose active width holds the
 * frame and whose lines hold it and its header row before the vertical
 * sync, at that rate, the slowest such -- the least the link has to
 * carry. NULL: none does. */
static const struct bridge_raster *pick_raster(uint32_t w, uint32_t h,
                                               uint32_t hz) {
    const struct bridge_raster *best = NULL;
    for (size_t i = 0; i < BRIDGE_N_RASTERS; i++) {
        const struct bridge_raster *r = &BRIDGE_RASTERS[i];
        if (r->hz != hz || w > r->h_active || h + 1 >= r->vs_beg) continue;
        if (!best || r->pixel_khz < best->pixel_khz) best = r;
    }
    return best;
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
/* ONE engine: the display is driven from the sensor or from a clip in
 * memory, never from a second process. A clip is payload rows only,
 * tightly packed -- the bridge keeps writing the bayerlink header
 * itself, so a recording cannot carry a stale one. */
static uint8_t *clip_map;
static size_t clip_bytes, clip_frame_bytes, clip_frames, clip_at;
static int clip_drawn;
static int scratch_fd = -1;
/* How many captures V4L2 holds at once. Named because two places need
 * it to agree: REQBUFS, and how many frames a still must be redrawn for
 * after a switch (see clip_drawn). */
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
                 "width=%u height=%u bits=%d order=%s raster=%s "
                 "phase=%.2f margin=%.0f "
                 "vblank=%d uptime=%ld exposure=%d gain=%d",
                 bstate.seq, bstate.locked, paused,
                 clip_map ? "clip" : "sensor", WIDTH, HEIGHT,
                 FORMATS[fmt_row].bits, FORMATS[fmt_row].order,
                 RASTER ? RASTER->name : "-", bstate.phase,
                 bstate.margin,
                 bstate.vblank,
                 bstate.started ? (long)(time(NULL) - bstate.started) : 0L,
                 exposure, gain);
        return;
    }
    if (!strncmp(cmd, "source ", 7)) {
        const char *what = cmd + 7;
        if (!strcmp(what, "sensor")) {
            if (clip_map) { munmap(clip_map, clip_bytes); clip_map = NULL; }
            clip_drawn = 0;
            snprintf(reply, cap, "ok the sensor drives the display");
            return;
        }
        char path[160];
        if (sscanf(what, "clip %159s", path) == 1) {
            int fd = open(path, O_RDONLY);
            struct stat st;
            if (fd < 0 || fstat(fd, &st) < 0) {
                snprintf(reply, cap, "err %s: %s", path, strerror(errno));
                if (fd >= 0) close(fd);
                return;
            }
            /* A clip frame is a whole bayerlink container -- the
             * header row included. That row carries the CFA phase,
             * the geometry and the depth of the frame BEHIND it, so
             * replacing it with this bridge's own header would hand
             * the receiver the live sensor's facts about somebody
             * else's picture. */
            size_t frame = (size_t)WIDTH * VDISP * 3;
            if ((size_t)st.st_size < frame) {
                snprintf(reply, cap, "err %s: %lld bytes is less than one "
                         "%ux%u container", path, (long long)st.st_size,
                         WIDTH, VDISP);
                close(fd);
                return;
            }
            uint8_t *m = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
            close(fd);
            if (m == MAP_FAILED) {
                snprintf(reply, cap, "err %s: %s", path, strerror(errno));
                return;
            }
            if (clip_map) munmap(clip_map, clip_bytes);
            clip_map = m;
            clip_bytes = st.st_size;
            clip_frame_bytes = frame;
            clip_frames = st.st_size / frame;
            clip_at = 0;
            clip_drawn = 0;
            snprintf(reply, cap, "ok %zu frame(s) from %s", clip_frames, path);
            return;
        }
        snprintf(reply, cap, "err source takes: sensor, clip <path>");
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
        int ok = fwrite(base_map_ro, 1, (size_t)WIDTH * 3, f) ==
                 (size_t)WIDTH * 3;
        for (uint32_t r = 0; ok && r < HEIGHT; r++)
            ok = fwrite(bstate.payload_map + (size_t)r * bstate.payload_pitch,
                        1, (size_t)WIDTH * 3, f) == (size_t)WIDTH * 3;
        fclose(f);
        if (!ok) {
            snprintf(reply, cap, "err %s: short write", cpath);
            return;
        }
        snprintf(reply, cap, "ok %s %u %u %s", cpath, WIDTH, VDISP,
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
        if (!strncmp(cmd, "bridge ", 7))
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
                                  .height = HEIGHT, .lines = VDISP };
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

static uint32_t overlay_plane(int fd, int crtc_index) {
    drmModePlaneResPtr planes = drmModeGetPlaneResources(fd);
    if (!planes) die("GetPlaneResources");
    uint32_t found = 0;
    for (uint32_t i = 0; i < planes->count_planes && !found; i++) {
        drmModePlanePtr p = drmModeGetPlane(fd, planes->planes[i]);
        if (!p) continue;
        if ((p->possible_crtcs & (1u << crtc_index)) && p->fb_id == 0) {
            /* overlay only: a cursor plane would take the SETPLANE and
             * refuse the size twenty minutes later, in the compositor */
            drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(
                fd, p->plane_id, DRM_MODE_OBJECT_PLANE);
            for (uint32_t j = 0; props && j < props->count_props; j++) {
                drmModePropertyPtr pr = drmModeGetProperty(fd, props->props[j]);
                if (pr && !strcmp(pr->name, "type")
                    && props->prop_values[j] == DRM_PLANE_TYPE_OVERLAY)
                    found = p->plane_id;
                drmModeFreeProperty(pr);
            }
            drmModeFreeObjectProperties(props);
        }
        drmModeFreePlane(p);
    }
    drmModeFreePlaneResources(planes);
    if (!found) { fprintf(stderr, "no overlay plane free\n"); exit(1); }
    return found;
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

static void set_ctrl(int fd, uint32_t id, int32_t value, const char *name) {
    struct v4l2_control c = {.id = id, .value = value};
    if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0) {
        fprintf(stderr, "S_CTRL %s=%d: %s\n", name, value, strerror(errno));
        exit(1);
    }
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

int main(int argc, char **argv) {
    /* NO exposure, NO gain. They belong to the sensor package, where
     * the AE loop lives; this file knows GEOMETRY -- the raster, the
     * line lock, the blanking that makes them equal. Both were written
     * here at startup anyway, and gain=200 is 12.5x on an ov5647: the
     * raw arrived 55% saturated and every stage downstream was blamed
     * in turn for highlights lost at the sensor. This file's own
     * control socket had been refusing callers on exactly that ground
     * ("the AE loop's to write, not a caller's") while main() did it. */
    int seconds = 0, margin_lines = 10;
    /* A mode change is a RESTART, deliberately: the whole setup path
     * recomputes the line lock from the sensor's pixel rate and the new
     * geometry, and doing that live would mean redoing by hand what
     * start-up already does correctly. The frame need only fit the
     * raster -- it sits inside it, and the header tells the receiver
     * the truth. */
    unsigned want_w = 0, want_h = 0, want_hz = 30;
    const char *want_order = NULL;
    int test_pattern = 0, selftest = 0;
    const char *video = NULL, *subdev = "/dev/v4l-subdev0";
    char video_found[288];
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--selftest")) selftest = 1;
        else if (!strcmp(argv[i], "--seconds")) seconds = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--margin")) margin_lines = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--test-pattern")) test_pattern = 1;
        else if (!strcmp(argv[i], "--order")) {
            order_given = 1;
            want_order = argv[++i];
        }
        else if (!strcmp(argv[i], "--mode")) {
            int n = sscanf(argv[++i], "%ux%u@%u", &want_w, &want_h, &want_hz);
            if (n != 2 && n != 3) {
                fprintf(stderr, "bridge: --mode wants WxH[@HZ], e.g. "
                        "1296x972 or 1920x1080@60\n");
                return 1;
            }
        }
        else if (!strcmp(argv[i], "--video")) video = argv[++i];
        else if (!strcmp(argv[i], "--subdev")) subdev = argv[++i];
        else { fprintf(stderr, "unknown arg %s\n", argv[i]); return 2; }
    }
    crc_init();
    setvbuf(stdout, NULL, _IOLBF, 0);   /* notes must land, not pool */
    {
        uint32_t w = want_w ? want_w : WIDTH, h = want_h ? want_h : HEIGHT;
        const struct bridge_raster *r = pick_raster(w, h, want_hz);
        if (!r) {
            fprintf(stderr, "bridge: no raster carries %ux%u at %u Hz; "
                    "the rasters are:", w, h, want_hz);
            for (size_t i = 0; i < BRIDGE_N_RASTERS; i++)
                fprintf(stderr, " %s", BRIDGE_RASTERS[i].name);
            fprintf(stderr, "\n");
            return 1;
        }
        set_raster(r);
        if (!selftest)
            printf("bridge: raster %s for a %ux%u frame\n", r->name, w, h);
    }

    if (selftest) {
        /* `make check` compares these bytes with the Python reference:
         * the header layout has ONE owner and this copy is verified
         * against it, never trusted. */
        uint8_t h[48];
        uint32_t seqs[3] = {7, 0, 123456};
        for (int i = 0; i < 3; i++) {
            pack_header(h, seqs[i], SOURCE_ID_OV5647);
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
            for (unsigned dy = 0; dy < 2; dy++)
                for (unsigned dx = 0; dx < 2; dx++)
                    printf("crop %s %u %u %s\n", FORMATS[i].order, dx, dy,
                           FORMATS[format_after_crop((int)i, dx, dy)].order);
        return 0;
    }

    /* Not given: find it by NAME. Refused rather than guessed when no
     * known receiver is present -- a guess streams from whatever
     * enumerated into that slot, and a UVC capture card answers the
     * open perfectly happily.
     *
     * After the selftest, deliberately: that block's stdout is compared
     * byte-for-byte against the Python reference, so anything printed
     * ahead of it is a failure of `make check`. The selftest needs no
     * device -- it proves the header layout, not the hardware. */
    if (!video) {
        if (resolve_capture_node(video_found, sizeof video_found)) {
            fprintf(stderr, "bridge: no CSI-2 receiver node found by name "
                            "(looked for unicam-image, rp1-cfe-csi2_ch0 in "
                            "/sys/class/video4linux). Name one with "
                            "--video if this Pi presents another.\n");
            return 1;
        }
        video = video_found;
        printf("bridge: sensor node %s (found by name, not number)\n", video);
    }

    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    /* Bound the tails: this is why the shipping twin is C. Refusal to
     * grant either is a warning, not an exit -- a dev box still works. */
    struct sched_param sp = {.sched_priority = 10};
    if (sched_setscheduler(0, SCHED_FIFO, &sp))
        fprintf(stderr, "note: no SCHED_FIFO (%s)\n", strerror(errno));
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
        fprintf(stderr, "note: no mlockall (%s)\n", strerror(errno));

    /* --- display ------------------------------------------------- */
    int dfd = open("/dev/dri/card1", O_RDWR);
    if (dfd < 0) dfd = open("/dev/dri/card0", O_RDWR);
    if (dfd < 0) die("open dri");
    drmSetClientCap(dfd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);

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
            return 1;
        }
        if (waited % DISPLAY_REPORT_S == 0)
            printf("bridge: no display connected yet -- waiting for the "
                   "sink (%d s of %d)\n", waited, DISPLAY_WAIT_S);
        sleep(1);
    }
    force_full_range(dfd, connector_id);

    struct dumb base = make_fb(dfd, WIDTH, VDISP);
    struct dumb pay = make_fb(dfd, WIDTH, HEIGHT);
    if (drmModeSetCrtc(dfd, crtc_id, base.fb_id, 0, 0, &connector_id, 1,
                       &MODE))
        die("SetCrtc");
    uint32_t plane = overlay_plane(dfd, crtc_index);
    if (drmModeSetPlane(dfd, plane, crtc_id, pay.fb_id, 0,
                        0, 1, WIDTH, HEIGHT,
                        0, 0, WIDTH << 16, HEIGHT << 16))
        die("SetPlane");
    /* Somewhere for the sensor to write while the display holds still.
     * Same geometry, never scanned out -- so the only cost of pause is
     * one buffer, and the picture never learns about it. */
    struct dumb scratch = make_fb(dfd, WIDTH, HEIGHT);
    scratch_map = scratch.map;
    scratch_pitch = scratch.pitch;
    if (drmPrimeHandleToFD(dfd, scratch.handle, DRM_CLOEXEC | DRM_RDWR,
                           &scratch_fd))
        fprintf(stderr, "bridge: no scratch dma-buf; pause unavailable\n");

    int dmabuf = -1;
    if (drmPrimeHandleToFD(dfd, pay.handle, DRM_CLOEXEC | DRM_RDWR, &dmabuf))
        die("PrimeHandleToFD");

    /* --- camera --------------------------------------------------- */
    int sfd = open(subdev, O_RDWR);
    if (sfd < 0) die("open subdev");
    struct v4l2_subdev_format sf = {
        .which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = 0,
        .format = {.width = want_w ? want_w : WIDTH,
                   .height = want_h ? want_h : HEIGHT,
                   .code = FORMATS[fmt_row].mbus,
                   .field = V4L2_FIELD_NONE}};
    if (ioctl(sfd, VIDIOC_SUBDEV_S_FMT, &sf) < 0) die("SUBDEV_S_FMT");

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
            if (dw > RASTER->h_active || dh + 1 >= RASTER->vs_beg) {
                fprintf(stderr, "bridge: the sensor is sending %ux%u, which "
                        "does not fit the %s raster chosen for the mode "
                        "asked. Choose a smaller sensor mode.\n",
                        dw, dh, RASTER->name);
                return 1;
            }
            WIDTH = dw;
            HEIGHT = dh;
            /* The row settles all three names at once: what unicam is
             * asked to deliver, and what the header tells the receiver
             * to unpack. An explicit --order still wins, but only the
             * ORDER -- the depth is the sensor's to state. */
            if (order_given && want_order) {
                int r = format_row(want_order, FORMATS[drow].bits);
                if (r < 0) {
                    fprintf(stderr, "bridge: no %s format at %d bits\n",
                            want_order, FORMATS[drow].bits);
                    return 1;
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

    set_ctrl(sfd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "ae");
    set_ctrl(sfd, V4L2_CID_AUTOGAIN, 0, "agc");
    set_ctrl(sfd, V4L2_CID_AUTO_WHITE_BALANCE, 0, "awb");
    set_ctrl(sfd, V4L2_CID_TEST_PATTERN, test_pattern, "pattern");

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
    set_ctrl(sfd, V4L2_CID_VBLANK, vblank_fix, "vblank");
    set_ctrl(sfd, V4L2_CID_HBLANK, hblank_nom, "hblank");

    int vfd = open(video, O_RDWR | O_NONBLOCK);
    if (vfd < 0) die("open video");
    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};
    fmt.fmt.pix.width = WIDTH; fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = FORMATS[fmt_row].v4l2;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = pay.pitch;
    if (ioctl(vfd, VIDIOC_S_FMT, &fmt) < 0) die("S_FMT");
    if (fmt.fmt.pix.bytesperline != pay.pitch) {
        fprintf(stderr, "unicam kept bytesperline %u, need %u: the stride "
                "IS the bridge, refusing\n",
                fmt.fmt.pix.bytesperline, pay.pitch);
        return 1;
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
            .memory = V4L2_MEMORY_DMABUF, .length = (uint32_t)pay.size};
        buf.m.fd = dmabuf;
        if (ioctl(vfd, VIDIOC_QBUF, &buf) < 0) die("QBUF");
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(vfd, VIDIOC_STREAMON, &type) < 0) die("STREAMON");

    printf("bridge: %dx%d pGAA, pitch %u, LINE-LOCKED hblank %d "
           "vblank %d, row %.3fus(s)/%.3fus(d), margin %d lines\n",
           WIDTH, HEIGHT, pay.pitch, hblank_nom, vblank_fix,
           t_row_s * 1e6, t_row_d * 1e6, margin_lines);

    /* the module hears the initial link state before the first frame */
    link_watch(1);
    ctl_open();

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
    while (running && (seconds <= 0 || time(NULL) - t0 < seconds)) {
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
            continue;
        }
        struct v4l2_buffer dq = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_DMABUF};
        dq.m.fd = dmabuf;
        if (ioctl(vfd, VIDIOC_DQBUF, &dq) < 0) { perror("DQBUF"); break; }
        /* Self-stamp at the DQBUF wakeup. This unicam SAYS its stamps
         * are monotonic (flags 0x2000) and then delivers zero -- read
         * raw off the bench, 2026-08-19. poll() wakes at frame-done,
         * so under SCHED_FIFO our own clock IS the frame-end time to
         * within scheduling jitter; the loop filter absorbs that. */
        struct timespec dqts;
        clock_gettime(CLOCK_MONOTONIC, &dqts);
        double t_now = dqts.tv_sec + dqts.tv_nsec * 1e-9;
        seq++;
        /* Record the wire, replay the wire: a clip carries its own
         * header and it goes out untouched, so a replayed capture is
         * the bytes that were captured. */
        if (!clip_map)
            pack_header(base.map, seq, SOURCE_ID_OV5647);
        /* Where the NEXT frame lands: the displayed buffer, or the
         * scratch one while paused. Both slots may carry either fd --
         * V4L2 takes the dma-buf at queue time, so a pause is one
         * assignment and never a STREAMOFF. */
        const int sensor_shown = !paused && !clip_map;
        dq.m.fd = (!sensor_shown && scratch_fd >= 0) ? scratch_fd : dmabuf;
        /* A clip is drawn into the same buffer the sensor would have
         * written, so everything downstream -- the header, the raster,
         * the genlock -- is unchanged. A still is copied once: there is
         * no reason to move six megabytes a frame to show one picture. */
        /* A still is redrawn for CAPTURE_BUFS frames, not once. At the
         * moment of the switch every queued capture still targets the
         * DISPLAYED buffer -- only the one queued below is redirected to
         * scratch. So the still is copied in, and then the sensor
         * completes the capture that was already in flight straight over
         * the top of it. Drawn once, nothing ever put it back: the
         * television sat on the last sensor frame until the source was
         * clicked a SECOND time, which reset this counter and redrew
         * after the queue had drained. */
        if (clip_map && (clip_frames > 1 || clip_drawn < CAPTURE_BUFS)) {
            const uint8_t *src = clip_map + clip_at * clip_frame_bytes;
            memcpy(base.map, src, WIDTH * 3);          /* the recorded header */
            for (uint32_t r = 0; r < HEIGHT; r++)
                memcpy(pay.map + (size_t)r * pay.pitch,
                       src + (size_t)(r + 1) * WIDTH * 3, WIDTH * 3);
            clip_at = (clip_at + 1) % clip_frames;
            /* Clamped: a moving clip redraws every frame anyway, and an
             * unbounded counter would be the only thing here that could
             * overflow given long enough. */
            if (clip_drawn < CAPTURE_BUFS) clip_drawn++;
        }
        if (ioctl(vfd, VIDIOC_QBUF, &dq) < 0) { perror("QBUF"); break; }

        /* PHASE: sensor frame-end against the last display vblank,
         * both on the kernel's monotonic clock. */
        /* The CRTC is not necessarily index 0, and a vblank request
         * names its CRTC in the HIGH_CRTC bits -- silently absent
         * reports, not an error code, is how that one shows itself. */
        drmVBlank vb = {.request = {
            .type = DRM_VBLANK_RELATIVE
                  | ((crtc_index << DRM_VBLANK_HIGH_CRTC_SHIFT)
                     & DRM_VBLANK_HIGH_CRTC_MASK),
            .sequence = 0}};
        int vbrc = drmWaitVBlank(dfd, &vb);
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
                set_ctrl(sfd, V4L2_CID_VBLANK, cmd, "vblank");
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
        bstate.sensor_fd = sfd;
        bstate.started = t0;
        base_map_ro = base.map;
        bstate.payload_map = pay.map;
        bstate.payload_pitch = pay.pitch;
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
    ioctl(vfd, VIDIOC_STREAMOFF, &type);
    close(vfd); close(sfd); close(dmabuf); close(dfd);
    printf("bridge: %u frames\n", seq);
    return 0;
}
