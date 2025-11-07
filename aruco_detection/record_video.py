"""
Simple OpenCV video recorder.

Features:
- Record from webcam (default) or an input stream/file
- Save to a chosen output file with codec, fps, and resolution
- Optional live preview window
- Optional horizontal flip for selfie view
- Stop with 'q' or after --max-duration seconds

Examples:
  - Record from webcam 0 to MP4 (H.264-compatible 'mp4v'):
      python3 aruco_detection/record_video.py --output recordings/out.mp4 --codec mp4v --fps 30 --width 1280 --height 720

  - Record from a video file to AVI (XVID):
      python3 aruco_detection/record_video.py --video input.mp4 --output out.avi --codec XVID

Note: For MP4 files, use --codec mp4v and an .mp4 extension. For AVI, use XVID or MJPG and .avi extension.
"""

import argparse
import os
import time
from datetime import datetime

import cv2


def parse_args():
    p = argparse.ArgumentParser(description="OpenCV video recorder")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--video", type=str, default=None, help="Path/URL to input video instead of webcam")
    src.add_argument("--camera", type=int, default=0, help="Camera index to open (default: 0)")

    p.add_argument("--output", type=str, default=None, help="Output video file path (auto if omitted)")
    p.add_argument("--codec", type=str, default="mp4v", help="FOURCC codec (e.g., mp4v, XVID, MJPG, H264)")
    p.add_argument("--fps", type=float, default=0.0, help="Target FPS (0 to auto-detect from source or default 30)")
    p.add_argument("--width", type=int, default=0, help="Capture width (0 to keep source default)")
    p.add_argument("--height", type=int, default=0, help="Capture height (0 to keep source default)")
    p.add_argument("--no-display", action="store_true", help="Disable preview window")
    p.add_argument("--flip-h", action="store_true", help="Flip frames horizontally before saving/displaying")
    p.add_argument("--max-duration", type=float, default=0.0, help="Max duration in seconds (0 for unlimited)")
    return p.parse_args()


def ensure_parent_dir(path: str):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)


def build_default_output(codec: str) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Choose extension based on codec
    codec_l = codec.lower()
    if codec_l in ("mp4v", "h264", "avc1", "x264"):
        ext = ".mp4"
    else:
        ext = ".avi"
    return os.path.join("recordings", f"recording_{ts}{ext}")


def to_fourcc(codec: str) -> int:
    c = codec.strip()
    if len(c) != 4:
        # Try known aliases
        aliases = {
            "mp4": "mp4v",
            "h264": "H264",
            "mjpg": "MJPG",
        }
        c = aliases.get(c.lower(), c)
        if len(c) != 4:
            raise ValueError(f"Codec must be a 4-character FOURCC (got '{codec}')")
    # Build FOURCC integer manually for compatibility
    return (ord(c[0]) & 0xFF) \
         | ((ord(c[1]) & 0xFF) << 8) \
         | ((ord(c[2]) & 0xFF) << 16) \
         | ((ord(c[3]) & 0xFF) << 24)


def main():
    args = parse_args()

    # Determine source
    source = args.video if args.video else args.camera
    print(f"Opening source: {source}")
    cap = cv2.VideoCapture(source)

    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print(f"Error: could not open source {source}")
        return 1

    # Read first frame to determine size
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Error: could not read first frame from source")
        cap.release()
        return 1

    if args.flip_h:
        frame = cv2.flip(frame, 1)

    h, w = frame.shape[:2]

    # Determine FPS
    src_fps = cap.get(cv2.CAP_PROP_FPS)
    fps = args.fps if args.fps > 0 else (src_fps if src_fps and src_fps > 0 else 30.0)

    # Determine output
    output_path = args.output if args.output else build_default_output(args.codec)
    ensure_parent_dir(output_path)

    # Create writer
    try:
        fourcc = to_fourcc(args.codec)
    except ValueError as e:
        print(f"Error: {e}")
        cap.release()
        return 1

    writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))
    if not writer.isOpened():
        print(f"Error: could not open VideoWriter for '{output_path}'")
        cap.release()
        return 1

    print(f"Recording started -> {output_path}")
    print(f"Resolution: {w}x{h}, FPS: {fps:.2f}, Codec: {args.codec}")
    print("Press 'q' to stop recording")

    start_time = time.time()
    frame_count = 0
    window_name = "Recording (press 'q' to stop)"

    # Write the first frame we already grabbed
    writer.write(frame)
    frame_count += 1

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of stream or read error; stopping.")
            break

        if args.flip_h:
            frame = cv2.flip(frame, 1)

        # Overlay simple HUD
        if not args.no_display:
            elapsed = time.time() - start_time
            rec_text = f"REC  {frame_count}  {elapsed:6.1f}s"
            cv2.circle(frame, (20, 20), 8, (0, 0, 255), -1)
            cv2.putText(frame, rec_text, (40, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        writer.write(frame)
        frame_count += 1

        if not args.no_display:
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("'q' pressed; stopping.")
                break

        if args.max_duration > 0:
            if (time.time() - start_time) >= args.max_duration:
                print("Max duration reached; stopping.")
                break

    writer.release()
    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()
    print(f"Saved {frame_count} frames to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
