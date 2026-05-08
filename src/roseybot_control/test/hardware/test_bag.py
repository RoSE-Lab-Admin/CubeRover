import subprocess
import os
from pathlib import Path
import pytest

@pytest.mark.hardware
def test_bag():
    # set paths
    bag_dir = Path("/mnt/d/ros_bags")
    script_path = Path(__file__).resolve().parent/"bag_topics.py"

    # set bash script variables
    env = os.environ.copy()
    env["BAG_DIR"] = str(bag_dir)
    env["DURATION"] = "10"

    # run bash script
    result = subprocess.run(
        ["python3", str(script_path)],
        check=True,
        text=True,
        capture_output=True,
        env=env
    )

    bag_path: Path | None = None

    # grab output of bash script
    for line in result.stdout.splitlines():
        if line.startswith("BAG_PATH="):
            bag_path = Path(line.split("=", 1)[1].strip())
            break
        
    # check if the bag path exists
    assert bag_path and bag_path.exists() and bag_path.is_dir(), f"Bag Folder Missing: {bag_path}"

    # checks if the metadata file exists
    meta_file = bag_path / "metadata.yaml"
    assert meta_file.exists(), f"metadata.yaml missing"

