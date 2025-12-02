from .mounted_panda import MountedPanda
from .on_the_ground_panda import OnTheGroundPanda
from .mounted_UR5e import MountedUR5e

from robosuite.robots.single_arm import SingleArm
from robosuite.robots import ROBOT_CLASS_MAPPING

ROBOT_CLASS_MAPPING.update(
    {
        "MountedPanda": SingleArm,
        "OnTheGroundPanda": SingleArm,
        "MountedUR5e": SingleArm,
    }
)
