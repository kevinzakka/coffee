from pathlib import Path

# Path to the root of the project.
_PROJECT_ROOT = Path(__file__).parent.parent

# Path to the root of the src files, i.e. `coffee/`.
_SRC_ROOT = _PROJECT_ROOT / "coffee"

# Path to the directory containing URDF assets.
_URDF_PATH = _SRC_ROOT / "models" / "vendor"
