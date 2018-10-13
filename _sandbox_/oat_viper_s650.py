
import logging

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

logging.info('Watch out!!!')

from morse.builder import FakeRobot, Robot, Environment
from morse.builder.actuators import Armature, KukaLWR
from morse.builder.sensors import Pose, ArmaturePose

_resource_filename = "/home/dieesrod/Workspaces/ROS/anchoring/src/robots_in_morse/robots/klwr_robot/klwr_description/urdf/kuka_lwr.urdf"
_resource_filename = "/home/dieesrod/Workspaces/MORSE/morse_simulator_urdf/testing/urdf/res/r2d2.urdf"
_resource_filename = "/home/dieesrod/Workspaces/ROS/anchoring/src/adept_viper_robot/adept_s650_support/urdf/s650.urdf"


def load_simulation_1():
    viper_robot = Robot(_resource_filename, name="kuka_lwr")

    env = Environment("empty", fastmode = False)
    env.set_camera_location([2.0, -2.0, 4.0])


def load_simulation_2():

    armature = Armature(model_name=_resource_filename)
    env = Environment("empty", fastmode=False)

    env = Environment("empty", fastmode = False)
    env.set_camera_location([2.0, -2.0, 4.0])


class AdeptViperS650(Armature):
    _name = "OAT Viper s650"
    _short_desc = "Omron Adept - 6DoF Robotic Arm"

    def __init__(self, name=None):
        Armature.__init__(self, name, model_name="oat_viper_s650")
        self.create_ik_targets(["joint_6"])

def load_simulation_3_1():

    robot = FakeRobot('robot')
    arm = AdeptViperS650()

    pose_sensor = ArmaturePose()
    arm.append(pose_sensor)

    robot.append(arm)

    # robot_namespace = "/" + robot.name + "/"

    # robot.add_default_interface('ros')
    #from morse_helpers.adapters import ROSRegister
    #ROSRegister.add_controller(arm, robot_namespace + 'arm', 'ArmCtrlByActions')

    env = Environment("apartment")
    env.set_camera_location([2.0, -2.0, 4.0])

def load_simulation_3_2():

    robot = FakeRobot('robot')
    arm = KukaLWR()

    pose_sensor = ArmaturePose()
    arm.append(pose_sensor)

    robot.append(arm)

    env = Environment("apartment")
    env.set_camera_location([2.0, -2.0, 4.0])

class ArmatureFromURDF(Armature):
    _name = "armature_from_urdf"
    _short_desc = "Omron Adept - 6DoF Robotic Arm"

    def __init__(self, resource_filename=""):
        Armature.__init__(self, self._name, model_name=resource_filename)

def load_simulation_4():

    robot = FakeRobot()
    arm = ArmatureFromURDF(_resource_filename)
    arm.create_ik_targets(["joint_6"])

    robot.append(arm)

    env = Environment("empty", fastmode=False)
    env.set_camera_location([2.0, -2.0, 4.0])

def load_simulation_5():

    robot = FakeRobot()

    from morse.builder.urdf import URDF
    arm = URDF(_resource_filename).build()

    # pose_sensor = ArmaturePose()
    # arm.append(pose_sensor)

    robot.append(arm)

    env = Environment("empty", fastmode=False)
    env.set_camera_location([2.0, -2.0, 4.0])

if __name__ == "__main__":
    load_simulation_3_1()

