# Learn from Intents: Learn to Grasp via Intention Discovery and its Application to Challenging Clutter


## 1. Overview
In this work, we propose a learning-based method for picking chanllenging objects and its application in dense clutter, which aims at singulating and simultaneously picking the objects one by one from a random clutter. This repository provides the implementation for training and testing.

**Picking in cluttered conveyor**
<p align = "center">
<img src="1.gif" width="640" height="360"> 
</p>

**Picking in cluttered table**
<p align = "center">
<img src="0.gif" width="640" height="360"> 
</p>

## 2. Prerequisites
### 2.1 Hardware
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/)
- [**Robotiq 140mm Adaptive parallel-jaw gripper**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
- [**RealSense Camera L515**](https://www.intelrealsense.com/lidar-camera-l515/)

### 2.2 Software
The code is built with Python 3.6. Dependencies are listed in requirements.yaml and can be installed via [Anaconda](https://www.anaconda.com/) by running:

    conda env create -n new_env -f environment.yml
    
## 3. Intention estimator learning

### 4.1 Grasp collecttion

You can collected simulated grasps and assign intents by running following code:
```
python collect_data.py
```

### 4.2 Train
```
python intent_train.py 
```
## 4. Grasping policy learning

You can start learning grasping policy by running following code:
```
python policy_train_rl.py
```

## 5.1 Test on Real Robot (UR10)
Here we provide the steps to test our method on a real robot.

**Robot control**

Robot is controlled via [this python software](https://github.com/SintefManufacturing/python-urx).

**Camera setup**

To deploy RealSense L515 camera,
Download and install the [librealsense SDK 2.0](https://github.com/IntelRealSense/librealsense)

**Start testing**

Then run the following code to start testing:
```
python test_in_real.py
```
