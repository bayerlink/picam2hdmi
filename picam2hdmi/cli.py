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
        "stream", help="scan out over HDMI (Raspberry Pi only; next milestone)")
    stream.add_argument("--source", default="pattern",
                        choices=["pattern", "camera"])

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
        from . import capture, output  # noqa: F401  (their docstrings are the design)
        output.stream(frames=None)

    return 0


if __name__ == "__main__":
    sys.exit(main())
