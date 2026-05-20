"""
Extract global track normalization bounds across all available maps.

This script scans the maps/ directory for all track folders and computes
global normalization bounds for observation normalization.

Usage:
    python train/extract_global_track_norm_bounds.py

After running, copy the printed values into gymkhana/envs/utils.py:
    GLOBAL_MAX_CURVATURE
    GLOBAL_MIN_WIDTH
    GLOBAL_MAX_WIDTH
"""

from pathlib import Path

from train.train_utils import compute_global_track_bounds

MAPS_DIR = Path(__file__).resolve().parent.parent / "maps"


def get_all_track_names() -> list[str]:
    """Extract all track names from subdirectories in maps/ folder."""
    track_names = []
    for subdir in MAPS_DIR.iterdir():
        track_name = subdir.name
        if subdir.is_dir() and not track_name.startswith("."):
            track_names.append(track_name)
    return sorted(track_names)


if __name__ == "__main__":
    print(f"Scanning {MAPS_DIR} for available tracks...")
    track_names = get_all_track_names()
    print(f"Found {len(track_names)} tracks: {', '.join(track_names)}\n")

    compute_global_track_bounds(track_names)

    print("Done! Copy the three GLOBAL_* lines above into gymkhana/envs/utils.py")
