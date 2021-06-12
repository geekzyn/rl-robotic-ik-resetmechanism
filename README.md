# Deep Reinforcement Learning - An Inverse Kinematic Approach to the Reset Mechanism
This repository contains the implementation for an IK definition of the reset mechanism in a RL-framework. The 2R robotic arm manipulator learns to move a cuboid to a goal position (pulling towards the end of the table). Instead of an instant reset of the environment at the end of an episode, the robotic arm will use IK to put the cuboid back to its initial position. The implementation is tested in ROS Noetic\Melodic and the algorithm used for training the robot is Soft Actor Critic (SAC).

## Installation

 - First, start by cloning the repository into a ROS workspace. For example `robot_ws/src`

```
git clone <repo id>
```

 - Then build the package when you are in workspace `robot_ws/`

```
catkin_make
```

 - To install the dependencies, run

```
rosdep install --from-paths src --ignore-src -r -y
```

 - Create a venv based on the ROS system.

 - To install the required packages for SAC agent, move to the repository folder and run,

```
pip install -r requirements.txt
```

## Usage

 - To start using the package, check whether the workspace is sourced,

```
source <workspace>/devel/setup.bash
```

 - To start the environment setup for the reinforcement learning,

```
roslaunch rrbot_rl env_setup.launch
```

 - To start the training process run,

```
python src/rrbot_rl/rrbot_rl/scripts/sac
```

## Test

To test the ros controller, when the workspace is sourced, run in a new terminal

```
rosrun rrbot_rl move_box
```
