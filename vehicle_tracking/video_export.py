from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
import subprocess

from PySide6.QtGui import QImage


def export_qimages_to_mp4(
    output_path: Path,
    frames: Iterable[QImage],
    width: int,
    height: int,
    fps: int = 20,
) -> int:
    try:
        import imageio_ffmpeg
    except ImportError as exc:
        raise RuntimeError(
            "MP4 export requires imageio-ffmpeg. Install the project requirements first."
        ) from exc

    executable = imageio_ffmpeg.get_ffmpeg_exe()
    command = [
        executable,
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    count = 0
    assert process.stdin is not None
    try:
        for image in frames:
            frame = image.convertToFormat(QImage.Format.Format_RGB888)
            if frame.width() != width or frame.height() != height:
                raise ValueError("Every MP4 frame must have the configured dimensions.")
            process.stdin.write(frame.bits().tobytes())
            count += 1
        process.stdin.close()
        error = process.stderr.read().decode("utf-8", errors="replace") if process.stderr else ""
        if process.stderr is not None:
            process.stderr.close()
        return_code = process.wait()
    except Exception:
        process.kill()
        raise
    if return_code != 0:
        raise RuntimeError(f"FFmpeg could not create the MP4: {error[-1000:]}")
    if count == 0:
        output_path.unlink(missing_ok=True)
        raise ValueError("No animation frames were supplied.")
    return count
