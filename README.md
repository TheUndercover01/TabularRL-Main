# Reinforcement Learning for Robotic Arm Manipulation

This repository serves as the primary repository housing three submodules: Monte Carlo, Temporal Difference, and n-step Bootstrapping (Links Above).

## Results

https://github.com/TheUndercover01/TabularRL-Main/assets/141422918/05a18cc4-e1c0-46b3-8900-96df2a6a3e33

## Project Overview

This project transcends pre-programmed robots, introducing a bot I built from scratch. Its mission: finding the most efficient path to a specific state. To achieve this, I've implemented a diverse arsenal of 7 algorithms, ranging from classic Monte Carlo to sophisticated n-step bootstrapping. The bot boasts 3 unique joints, unlocking a staggering 24 possible configurations (explained below). Crafted in Onshape and refined through immersive PyBullet simulations, this project has provided me with extensive insights into the intricate world of robotic manipulation and reinforcement learning.

## Onshape Bot design

Check out the bot on Onshape [here](https://cad.onshape.com/documents/04a8f06c4e82eef0aab52342/w/e26ea93d189b4fb4644d2868/e/ce0ae9d693e713171509edc4?renderMode=0&leftPanel=false&uiState=65b6963083efbe35d664705e).

<img src="https://github.com/TheUndercover01/TabularRL-Robotics/blob/main/image_bot.png?raw=true" alt="Robotic Arm" width="575" height="600">

### Joint Analysis

Each component of the bot, from its joints to links, has been made from scratch. Here's a breakdown of the key joints and their corresponding links:

- **Joint 0:**
  - Type: Revolute joint
  - Link: 'stand'
  - Description: Connects the base to the stand.
  - No limits (can move 360&deg;)

- **Joint 1:**
  - Type: Slider joint
  - Link: 'clipper_comp'
  - Description: Connects the stand to the extended arm, facilitating vertical movement.
  - Limit is +210 mm (down) , -210 mm (up)

- **Joint 2:**
  - Type: Revolute joint
  - Link: 'gripper_front'
  - Description: Connects the front gripper to the extended arm.
  - Limit is -30&deg; (Open Gripper) , +15&deg; (Close Gripper)

- **Joint 3:**
  - Type: Revolute joint
  - Link: 'gripper_2'
  - Description: Connects the back gripper to the extended arm.
  - Limit is -30&deg; (Open Gripper) , +15&deg; (Close Gripper)


## Understanding State Representation in the Pickup Bot Environment

The state representation encapsulates crucial information about the bot's configuration and dynamics. Let's delve into the meaning of each component within the state tuple:

### 1. Stand Rotation (in radians):
- This element indicates the rotation of the bot's stand around its vertical axis.
- It represents the angular orientation of the bot's trunk or stand relative to its initial position.
- States can vary from - $\pi$ to + $\pi$ radians, covering a full circle of possible orientations.
- Possible Rotation: 0, $\pi$ /2, - $\pi$ /2, $\pi$

### 2. Slider Position along the Z-axis:
- The slider position refers to the vertical displacement of the bot's sliding mechanism.
- It signifies the elevation of the bot's gripper assembly, affecting its reach and interaction with the environment.
- States typically range from the lowest position (closest to the ground) to the highest position.
- Example values: 0.11 (lower position), 0.32 (mid position), 0.53 (upper position)

### 3. Gripper Orientation (Open or Closed):
- This component indicates the state of the bot's gripper mechanism, determining whether it is open or closed.
- Gripper orientation plays a critical role in the bot's ability to grasp, manipulate, or release objects within its vicinity.
- States are binary, representing either an open or closed gripper configuration.
- Example values: -0.84 (open), 0.04 (closed), etc.

By combining these three elements into a tuple, the Pickup Bot environment can precisely describe the bot's current configuration and readiness for performing tasks. 

Example : (0, 0.53, -0.84, -0.84) - this state represents the rotation of 0&deg; w.r.t the stand joint, 0.53mm represents that the arm is 0.21 (0.53 - 0.32) above the slider joints initial position, and -0.84 radians represents that the gripper is open.

## Exploring PyBullet Dynamics

- **Initialization**: Upon initialization, the environment is set up, including the connection to the PyBullet physics engine and the loading of the robot's URDF file. Parameters like gravity and initial positions are configured to establish a realistic simulation environment. `You can choose to start from a completely difference position/state, just add the new state as the 2nd parameter while Initialization of the environment`
  ```python
  if GUI:
      physicsClient = p.connect(p.GUI)
  else:
      physicsClient = p.connect(p.DIRECT)
  
  planeId = p.loadURDF("plane.urdf")
  p.setGravity(0,0,-10)
  startPos1 = [-0.19670987601116390886, -0.99812458682335825078, 0.63779767016]
  startOrientation1 = p.getQuaternionFromEuler([0,0,0])
  self.robot = p.loadURDF(robot,startPos1, startOrientation1,useFixedBase=1)
  
- **Joint Control**: The primary mechanism for controlling the bot's movements involves adjusting the positions of its joints. PyBullet's setJointMotorControlArray function is employed to set the desired positions for specific joints, enabling precise control over the bot's orientation and motion. **One thing to note here is that the `desired_posion` is always considered w.r.t to the starting point of the joint when it was first initialised. So, when working with multiple iterations through different states, we must consider `desired_position` w.r.t current position, which means we must account for this discrepancy.**
  ```python
  origin_shift =  self.stand_init - self.stand_rotation
  p.setJointMotorControlArray(self.robot, [0], p.POSITION_CONTROL, targetPositions=[origin_shift - desired_position])

- **Simulation Update**: Following the execution of each action, the PyBullet simulation is stepped forward in time to update the bot's position and orientation. This iterative process ensures that the bot's movements are accurately reflected within the simulated environment.
  ```python
  for i in range(100):
    p.stepSimulation()



## More on Repository and Submodules

This repository contains the environment for both the Monte Carlo and Temporal Difference algorithms. It links to three repositories where this bot is utilized in various algorithms. The bot file includes the URDF file for the bot.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## P.S. - Project Ideation 

When I embarked on this project, my goal was to transcend the conventional boundaries of reinforcement learning as outlined in textbooks such as "RL: An Introduction." Instead of confining myself to grid-based environments, I aimed to tackle more complex challenges. With a deep interest in robotics, I viewed this endeavor as an opportunity to explore the intersection of robotics and reinforcement learning, thereby enhancing my comprehension of both fields.

The journey proved arduous. I encountered setbacks with two failed robotics designs, underwent numerous trials using PyBullet, and iterated through various environment designs. Despite these obstacles, I remained steadfast in my determination and commitment. Today, I am thrilled to unveil the culmination of my efforts: a project that seamlessly integrates the intricacies of robotics with robust reinforcement learning algorithms.

This concludes the project information. Below is the experiments I ran to understand pybullet.
________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
## Project Notes

### Introduction
The purpose of these notes is to document observations and experiments conducted to understand the behavior of the bullet environment's joints. This understanding is crucial for defining states and rewards in the final environment for reinforcement learning (RL) applications.

### Test 1: Stand Movement Analysis
#### Experiment
- Rotate the stand by 180 degrees.
- Command:
  ```python
  p.setJointMotorControlArray(env, [0], p.POSITION_CONTROL, targetPositions= [-3.14]) 

#### Observations
- **Position Observation:** `p.getLinkState(env,0)[0]` yielded no change in value.
- **Orientation Observation:** `p.getLinkState(env,0)[1]` revealed a change in quaternions, particularly in the x and y components. Converted to Euler angles, this resulted in (3.14159265358978, 9.057052533490636e-15, 0.23406194496518992), indicating a rotation along the z-axis.

### Experiment Next: Stand Rotation Analysis Continued
- After rotating the stand by 180 degrees, further observations were made.
- The Euler angles changed significantly, emphasizing the importance of the z-coordinate in the orientation.

### Test 2: Slider Joint Movement Analysis
#### Experiment
- Manipulate the slider joint.
- Command:
  ```python
  p.setJointMotorControlArray(env, [1], p.POSITION_CONTROL, targetPositions= [+0.23])

#### Observations
- The z-coordinate of the Cartesian position of the center of mass changed by 0.23 units, indicating the significance of this coordinate in the slider joint movement.

### Test 3: Gripper Movement Analysis
#### Experiment
- Investigate gripper movement.
- Command:
  ```python
  p.setJointMotorControlArray(env, [2], p.POSITION_CONTROL, targetPositions= [-1.5261])

#### Observations
- Notable changes in the y-coordinate of the gripper's orientation were observed. This suggests the gripper's movement is relative to its initial position.

### Test 4: Analysis of Gripper 2
- Similar observations were made for Gripper 2, indicating consistent behavior across different gripper components.

### Insights and Discoveries
- Movements in the environment are relative to the bot's initial position.
- Setting the target position to specific radians can result in different rotational angles, highlighting the relative nature of the movements.

### Workaround for Joint Positioning
- It was discovered that adjusting the joint's origin helps achieve more accurate positioning.
- Shifting the origin of the joint enables more precise control over movements, ensuring actions are relative to the inertial frame.

- The key insight is that all motions within the bot's environment are relative to its initial position. This means that simply adding or subtracting values from the current position may not yield the expected results, especially when the bot's position has changed from its initial state.

#### Example Scenario (Slider Joint):
1. Suppose the current position of the slider joint is -0.32.
2. If we intend to move down by 0.21 units, intuitively we might expect adding 0.21 to the current position (-0.32 + 0.21 = -0.11) would achieve this.
3. However, if the current position is actually -0.11, adding 0.21 to it would result in 0.10, not the expected position of 0.32.

### Implementation Example (Slider Joint):
1. If the initial position of the joint is 0.32, and the current position is -0.11.
2. The difference between the initial and current position is 0.32 - (-0.11) = 0.43.
3. Now, to move up by 0.21 units from the current position, we subtract 0.21 from the calculated value: -0.11 - 0.21 = -0.32.
4. This adjustment ensures that the command is relative to the starting position, resulting in the desired movement.

### Consideration of Internal Friction:
- The explanation also touches upon the issue of internal friction causing slight deviations in the positions of links from their intended positions.
- Despite these deviations, the error introduced by internal friction is negligible, indicating that the proposed workaround effectively addresses the relative nature of motions without needing to account for minor positional discrepancies due to internal friction.

