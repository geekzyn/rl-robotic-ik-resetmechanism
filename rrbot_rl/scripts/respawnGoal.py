import rospy
import time
import os
import numpy as np

from gazebo_msgs.srv import DeleteModel, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

from utils import robot_model


class Respawn():
    def __init__(self):
        # Initialization of node for controlling joint1 and joint2 positions.
        # rospy.init_node('respawner_node', anonymous=True)

        rate = rospy.Rate(80)  # Rate 80 Hz

        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('gazebo/set_model_state')
        rospy.loginfo("Gazebo services loaded")

        self.model_name = "cardboard_box"

        # Define publishers for joint1 and joint2 position controller commands.
        self.joint1publisher = rospy.Publisher(
            '/rrbot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2publisher = rospy.Publisher(
            '/rrbot/joint2_position_controller/command', Float64, queue_size=10)

        self.positioning = True
        self.initial_EE_pose = [1.0, 2.4]
        self.joint_step = [0.01, 0.035]
        self.step_upward = [0.02, 0.04]

        self.current_EE_pose = self.initial_EE_pose

        self.init_pose = self.getModelState()
        self.robot_set_start()

        # Set the robot kinematics model

        self.robot_model = robot_model.robot()
        px, py, pz = self.robot_model.getForwardKinematics()
        self.X = [px, py, pz]  # The state function of the robot model

        # Theta function of the tobot model
        self.theta1, self.theta2 = self.robot_model.getInverseKinematics()

    def joint_publisher(self, joint1_position, joint2_position, joint=0):
        """Publishes joint data to the robot

        Args:
            joint1_position (std_msgs/Float64): Joint value for q1
            joint2_position (std_msgs/Float64): Joint value for q2
            joint (int, optional): Select the joint that need to be published (1/2). Defaults to 0 (otherwise).
        """
        if joint == 1:
            self.joint1publisher(joint1_position)
        elif joint == 2:
            self.joint2publisher(joint2_position)
        else:
            self.joint1publisher.publish(joint1_position)
            self.joint2publisher.publish(joint2_position)

    def robot_set_start(self):
        """Sets robot to home position
        """
        self.joint_publisher(self.initial_EE_pose[0], self.initial_EE_pose[1])

    def robot_rollback(self, joint1_position, joint2_position):
        """Sets the rollback function for the rrbot

        Args:
            joint1_position (float): Joint position of q1
            joint2_position (float): Joint position of q2

        Returns:
            joint1_position, joint2_position: Updated joint position value through steps
        """
        for x in range(1, 50):
            joint2_position = joint2_position + self.step_upward[1] * x
            joint1_position = joint1_position - self.step_upward[0] * x
            self.joint_publisher(joint1_position, joint2_position)

        return joint1_position, joint2_position

    def setModelState(self, pose):
        """Sets the model pose (here cardboard box)

        Args:
            pose (geomtery_msgs/Pose): Pose of the model to be set
        """
        model_state = ModelState()

        model_state.model_name = self.model_name
        model_state.pose = pose

        try:
            set_state = rospy.ServiceProxy(
                '/gazebo/set_model_state', SetModelState)
            response = set_state(model_state)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)

    def getModelState(self):
        """Get the current state of the mode (here cardboard box)

        Returns:
            pose (std_msgs/Pose): The current pose of the box model
        """
        pose = Pose()

        try:
            get_state = rospy.ServiceProxy(
                '/gazebo/get_model_state', GetModelState)
            response = get_state(self.model_name, "")
        except rospy.ServiceException as e:
            rospy.WARN("Service call failed: %s" % e)

        return response.pose

    def softRespawnModel(self):
        """Performs instant simulation reset of the model and the robot (doesn't perform inverse kinematics)
        """
        try:

            # Reset Cardboard Box
            self.setModelState(self.init_pose)

            # Reset Robot Model
            self.robot_set_start()

        except rospy.ServiceException as e:
            rospy.WARN("Soft Reset Failed")

    def hardRespawnModel(self):
        """The hard reset is made in this function involving inverse kinematics and
        joint space based trajectory planning for this application
        """
        joint1_position, joint2_position = 0, 0

        if not self.init_pose == self.getModelState():
            try:
                # Reset Cardboard Box
                # Due to varying dynamic properties, changing the plan for trajectory planning (Joint space)
                joint1initial_position, joint2initial_position = self.setRobotState(
                    1, 2)
                for x in range(0, 50):
                    joint1_position = joint1initial_position + \
                        self.joint_step[0] * x
                    joint2_position = joint2initial_position - \
                        self.joint_step[1] * x
                    self.joint_publisher(joint1_position, joint2_position)
                    rospy.sleep(0.2)

                # Reset Robot Model
                # The planning is still based on joint space configurations
                joint1initial_position, joint2initial_position = joint1_position, joint2_position
                for x in range(1, 30):
                    joint1_position = joint1initial_position + \
                        self.joint_step[0] * x
                    joint2_position = joint2initial_position + \
                        self.joint_step[1] * x
                    self.joint_publisher(joint1_position, joint2_position)
                    rospy.sleep(0.2)
                rospy.sleep(1)
                self.setRobotState(1, 2)

            except rospy.ServiceException as e:
                rospy.WARN("Hard Reset Failed")

        self.init_pose = self.getModelState()

    def deleteModel(self):
        """Deletes the cardboard box model
        """
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy(
                    'gazebo/delete_model', DeleteModel)
                del_model_prox(self.model_name)
                break
            else:
                pass

    def getPosition(self, delete=False):
        """Gets the current position of the model

        Args:
            delete (bool, optional): Delete the box model. Defaults to False.

        Returns:
            x, y, z (float): The position of the box model
        """
        if delete:
            self.deleteModel()

        ret = self.getModelState()

        return ret.position.x, ret.position.y, ret.position.z

    def setPosition(self, x, y, z):
        """Sets the position of the cardboard box model

        Args:
            x (float): x-coordinate
            y (float): y-coordinate
            z (float): z-coordinate
        """
        pose = Pose()

        pose.position.x, pose.position.y, pose.position.z = x, y, z

        self.setModelState(pose)

    def getRobotState(self, theta_1, theta_2):
        """Gets the current position of the robot end effector

        Args:
            theta_1 (float): Joint angle of joint 1 in radians
            theta_2 (float): Joint angle of joint 2 in radians

        Returns:
            X (tuple): Tuple of the state [x, y, z] of the robot model
        """

        x = self.X[0](theta_1, theta_2)
        y = self.X[1](theta_1, theta_2)
        z = self.X[2](theta_1, theta_2)

        return tuple([x, y, z])

    def setRobotState(self, px, pz):
        """Sets the robot end effector to the desired pose given x and z (Note: y remains constant for this model)

        Args:
            px (float): X coordinate of the robot end-effector
            pz (float): Z coordinate of the robot end-effector
        """

        theta2 = self.theta2(px, pz)
        theta11 = self.theta1(px, pz, theta2[0])
        theta12 = self.theta1(px, pz, theta2[1])

        if np.nan in theta2 or np.nan in theta11 or np.nan in theta12:
            rospy.WARN(
                "Unable to perform hard reset. Performing simulation reset")
            self.robot_set_start()
        else:
            # An offset of pi/2 is added to support the frame changes in x-axis
            self.joint_publisher(theta11[1], theta2[0])

        return theta11[1], theta2[0]


if __name__ == "__main__":
    spawner = Respawn()

    # x, y, z = spawner.getPosition()
    # print(x, y, z)
    # time.sleep(5)

    # spawner.setPosition(x + 0.5, y, z)

    # time.sleep(5)
    spawner.hardRespawnModel()
