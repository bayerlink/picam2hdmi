"""Command line: `picam2hdmi pattern` works anywhere; `stream` needs a Pi."""
from __future__ import annotations

import argparse
import sys

import numpy as np

from bayerlink import pattern, protocol


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="picam2hdmi",
        description="Raw Bayer over HDMI, speaking bayerlink v1 "
                    "(github.com/bayerlink/bayerlink).")
    sub = parser.add_subparsers(dest="command", required=True)

    pat = sub.add_parser(
        "pattern",
        help="encode a test pattern into a bayerlink container (runs anywhere)")
    pat.add_argument("--mode", default="counting",
                     choices=sorted(pattern.PATTERNS))
    pat.add_argument("--width", type=int, default=2028,
                     help="samples per line (default: HQ camera 2028)")
    pat.add_argument("--height", type=int, default=1078,
                     help="camera lines; a 1080-line display carries at most "
                          "1079 (one line is the header), 1078 keeps whole "
                          "CFA rows")
    pat.add_argument("--display", default="1920x1080",
                     help="container geometry, WxH (default 1920x1080)")
    pat.add_argument("--bayer", default="RGGB",
                     choices=sorted(protocol.BAYER_PHASE))
    pat.add_argument("--frame-seq", type=int, default=0)
    pat.add_argument("--out", required=True,
                     help=".npy for the (H,W,3) container, .bin for raw bytes")

    stream = sub.add_parser(
        "stream", help="scan out over HDMI (runs on the Pi)")
    stream.add_argument("--source", default="pattern",
                        choices=["pattern", "camera"])
    stream.add_argument("--pattern", default="counting",
                        choices=sorted(pattern.PATTERNS))
    stream.add_argument("--width", type=int, default=2028)
    stream.add_argument("--height", type=int, default=1078)
    stream.add_argument("--bayer", default="RGGB",
                        choices=sorted(protocol.BAYER_PHASE))
    stream.add_argument("--mode", default="1920x1080@30",
                        help="required display mode WxH@Hz, or 'preferred'")
    stream.add_argument("--connector", type=int, default=None)
    stream.add_argument("--card", default=None, help="/dev/dri/cardN override")
    stream.add_argument("--luma-tunnel", action="store_true",
                        help="wrap the container in bayerlink's luma tunnel, "
                             "for Y-only capture paths (cheap YUY2 dongles); "
                             "pick a small --width/--height, capacity is 1/6")

    args = parser.parse_args(argv)

    if args.command == "pattern":
        width, _, height = args.display.partition("x")
        display = (int(width), int(height))
        raw = pattern.generate(args.mode, args.width, args.height)
        frame = protocol.encode_frame(
            raw, args.bayer, frame_seq=args.frame_seq, display=display,
            flags=protocol.FLAG_TEST_PATTERN)
        if args.out.endswith(".npy"):
            np.save(args.out, frame)
        else:
            frame.tofile(args.out)
        # Prove the file by decoding it back before claiming success.
        header, decoded = protocol.decode_frame(frame)
        assert np.array_equal(decoded, raw)
        print(f"wrote {args.out}: {args.mode} {args.width}x{args.height} "
              f"({header.fourcc}, seq {header.frame_seq}) in "
              f"{display[0]}x{display[1]}, round-trip verified")
        return 0

    if args.command == "stream":
        from . import output

        if args.mode == "preferred":
            want = None
            display = None      # resolved after the mode is known
        else:
            size, _, hz = args.mode.partition("@")
            width, _, height = size.partition("x")
            want = (int(width), int(height), int(hz or 60))
            display = (want[0], want[1])
        if args.source == "camera":
            from . import capture
            capture.frames()    # states its own status precisely
            return 1
        if display is None:
            parser.error("--mode preferred needs --source camera; a pattern "
                         "must be encoded for a known geometry, so require "
                         "the mode explicitly (e.g. --mode 1920x1080@30)")
        frames = output.pattern_frames(args.pattern, args.width, args.height,
                                       args.bayer, display,
                                       luma_tunnel=args.luma_tunnel)
        try:
            output.stream(frames, mode=want, connector=args.connector,
                          card_path=args.card,
                          on_frame=lambda i: print(f"frame {i}", end="\r")
                          if i % 300 == 0 else None)
        except KeyboardInterrupt:
            print("\nstopped")
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
