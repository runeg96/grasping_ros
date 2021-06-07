# grasping_ros

This repository is part of a semester project from group 1061 from the Robotics MSc program, and is based on the master thesis: ***Human-to-Robot Handovers Based on Visual Data for Optimisation of Industrial Tasks***. 

The project is a sub-system of the Little Helper 7+ dual-arm robot from Aalborg university, which is developed using ROS Melodic (Ubuntu 18.04).

## Authors

* **[Jan Kjær Jørgensen](mailto:jkja16@student.aau.dk)**           - *Group member* - [Jan Kjær Jørgensen](https://bitbucket.org/%7B342f1a45-adf0-43d0-856c-6a37d68d26d8%7D/)
* **[Rune Grønhøj](mailto:rgranh16@student.aau.dk)**               - *Group member* - [Rune Grønhøj](https://bitbucket.org/%7Be861c97c-c210-4770-bc27-9db291d95387%7D/)

## Acknowledgements

* Aalborg University
* Supervisors: Dimitris Chrysostomou

## Overview

This repository consist of many different sub-modules which all relate to antipodal robotic grasping based on deep learning. A description of each folder follows:

  - [**ggcnn**](https://github.com/dougsm/ggcnn): Generative Grasping CNN (GG-CNN) [Untouched].
  - [**gpd**](https://github.com/Janx1913/gpd/tree/b7dc050d50b48a82ac1ee77ee7eaccae47b9aa1f): Grasp Pose Detection (GPD) [Forked].
  - [**gpd_ros**](https://github.com/Janx1913/gpd_ros/tree/205e186f8433e6a05b2d6283b7a02d3be191260f): ROS Wrapper for GPD [Forked].
  - **grasp_lib**: Custom LH7 grasping library, with visualizer and wrapper for multiple methods and interfacing with LH7 robot.
  - [**grasp_multiObject_multiGrasp**](https://github.com/ivalab/grasp_multiObject_multiGrasp/tree/806ad3d71c2f413a74294fe75fe26ba4f32c8813): Grasp Multi-Object Multi-Grasp [Untouched].
  - [**graspnetAPI**](https://github.com/graspnet/graspnetAPI/tree/f312d7e2a73ca97f9e791cf3f1ab3aeacebac3e9): API for working with the Graspnet data-set and Graspnet Baseline [Untouched].
  - [**graspnet-baseline**](https://github.com/graspnet/graspnet-baseline/tree/a1e6f169575369fbdeb78aa1e2f0ef9f05ae6ee9): GraspNet Baseline [Untouched].
  - [**pointnet2_pytorch**](https://github.com/erikwijmans/Pointnet2_PyTorch/tree/acda965224f35854bc331cd5fe140393216b0a71): Pointnet2/Pointnet++ PyTorch [Untouched].
  - [**pointnet_gpd**](https://github.com/lianghongzhuo/PointNetGPD/tree/23b0afdf86eecaf281952acd2fdb98fd2625f54c): PointNetGPD: Detecting Grasp Configurations from Point Sets.
  - [**pytorch_6dof-graspnet**](https://github.com/jsll/pytorch_6dof-graspnet/tree/a620569a815dac47993dda6dbbcb34796b2f55e4): 6-DoF GraspNet: Variational Grasp Generation for Object Manipulation [Untouched].
  - [**ros_deep_grasp**](https://github.com/Janx1913/ros_deep_grasp/tree/24aec954922766d6b456428cf2bd12c6c1b77f52): This is the ROS implementation 'Real-world Multi-object, Multi-grasp Detection' [Forked].
  - [**vgn**](https://github.com/runeg96/vgn/tree/4f4644f52da663066bd8504f369d1c859a0ba0c1): Volumetric Grasping Network [Forked].
