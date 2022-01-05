from pathlib import Path

# Path to the root of the project.
_PROJECT_ROOT = Path(__file__).parent.parent

# Path to the root of the src files, i.e. `coffee/`.
_SRC_ROOT = _PROJECT_ROOT / "coffee"

# Path to the directory containing meshes and URDF files.
ASSETS_PATH = str(_PROJECT_ROOT / "vendor")
