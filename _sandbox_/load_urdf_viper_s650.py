
from morse.builder.urdf import URDF

_resource_filename = "/home/dieesrod/Workspaces/ROS/anchoring/src/robots_in_morse/robots/klwr_robot/klwr_description/urdf/kuka_lwr.urdf"
_resource_filename = "/home/dieesrod/Workspaces/MORSE/morse_simulator_urdf/testing/urdf/res/r2d2.urdf"
_resource_filename = "/home/dieesrod/Workspaces/ROS/anchoring/src/adept_viper_robot/adept_s650_support/urdf/s650.urdf"


def load_simulation():

    urdf = URDF(_resource_filename)
    return urdf.build()


if __name__ == "__main__":
    load_simulation()

