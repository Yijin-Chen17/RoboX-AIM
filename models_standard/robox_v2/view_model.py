"""Open the packaged model in the standard MuJoCo viewer."""
from pathlib import Path

import mujoco
import mujoco.viewer


if __name__ == '__main__':
    model_path = Path(__file__).resolve().with_name('robox_v2.xml')
    model = mujoco.MjModel.from_xml_path(str(model_path))
    mujoco.viewer.launch(model=model)
