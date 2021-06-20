# grasping_ros

This repository is part of a semester project from group 1061 from the Robotics MSc program, and is developed in relation to the master thesis: ***Human-to-Robot Handovers Based on Visual Data for Optimisation of Industrial Tasks***. 

The project is a sub-system of the Little Helper 7+ dual-arm robot from Aalborg university, which is developed using ROS Melodic (Ubuntu 18.04), see the main repo [**LH7+**](https://bitbucket.org/masterrob/lh7-handover/src/main/).

## Authors

* **[Jan Kjær Jørgensen](mailto:jkja16@student.aau.dk)**           - *Group member* - [Jan Kjær Jørgensen](https://bitbucket.org/%7B342f1a45-adf0-43d0-856c-6a37d68d26d8%7D/)
* **[Rune Grønhøj](mailto:rgranh16@student.aau.dk)**               - *Group member* - [Rune Grønhøj](https://bitbucket.org/%7Be861c97c-c210-4770-bc27-9db291d95387%7D/)

## Acknowledgements

* Aalborg University
* Supervisors: Dimitris Chrysostomou

## Overview

This repository consist of different sub-modules and folders inside the src folder, which all relate to antipodal robotic grasping based on deep learning. A description of each folder follows:

  - [**ggcnn**](https://github.com/dougsm/ggcnn): Generative Grasping CNN (GG-CNN) [*Untouched*].
  - [**gpd**](https://github.com/Janx1913/gpd/tree/b7dc050d50b48a82ac1ee77ee7eaccae47b9aa1f): Grasp Pose Detection (GPD) [*Forked*].
  - [**gpd_ros**](https://github.com/Janx1913/gpd_ros/tree/205e186f8433e6a05b2d6283b7a02d3be191260f): ROS Wrapper for GPD [*Forked*].
  - [**gr_grasp**](https://github.com/runeg96/robotic-grasping/tree/038a5723b54300cf3dfa1663aa8a9b49f3606218): Antipodal Robotic Grasping [*Forked*]    
  - **grasp_lib**: Custom LH7 grasping library, with visualizer and wrapper for multiple methods and interfacing with LH7 robot.
  - [**grasp_multiObject_multiGrasp**](https://github.com/ivalab/grasp_multiObject_multiGrasp/tree/806ad3d71c2f413a74294fe75fe26ba4f32c8813): Real-world Multi-object, Multi-grasp Detection [*Untouched*].
  - [**graspnetAPI**](https://github.com/graspnet/graspnetAPI/tree/f312d7e2a73ca97f9e791cf3f1ab3aeacebac3e9): API for working with the Graspnet data-set and Graspnet Baseline [*Untouched*].
  - [**graspnet-baseline**](https://github.com/graspnet/graspnet-baseline/tree/a1e6f169575369fbdeb78aa1e2f0ef9f05ae6ee9): GraspNet Baseline [*Untouched*].
  - [**pointnet2_pytorch**](https://github.com/erikwijmans/Pointnet2_PyTorch/tree/acda965224f35854bc331cd5fe140393216b0a71): Pointnet2/Pointnet++ PyTorch [*Untouched*].
  - [**pointnet_gpd**](https://github.com/lianghongzhuo/PointNetGPD/tree/23b0afdf86eecaf281952acd2fdb98fd2625f54c): PointNetGPD: Detecting Grasp Configurations from Point Sets. [*Untouched*]
  - [**pytorch_6dof-graspnet**](https://github.com/jsll/pytorch_6dof-graspnet/tree/a620569a815dac47993dda6dbbcb34796b2f55e4): 6-DoF GraspNet: Variational Grasp Generation for Object Manipulation [*Untouched*].
  - [**ros_deep_grasp**](https://github.com/Janx1913/ros_deep_grasp/tree/24aec954922766d6b456428cf2bd12c6c1b77f52): This is the ROS implementation of 'Real-world Multi-object, Multi-grasp Detection' [*Forked*].
  - [**vgn**](https://github.com/runeg96/vgn/tree/4f4644f52da663066bd8504f369d1c859a0ba0c1): Volumetric Grasping Network [*Forked*].

## Getting Started

To start generating grasp, the LH7 system has to be run or as a minimum the RealSense camera. The instructions are found on the main repo [**LH7+**](https://bitbucket.org/masterrob/lh7-handover/src/main/).

### Grasp Visualization

Running grasp visualizer using Rviz:
```shell
roslaunch grasp_lib visualizer.launch
```

This loads a special rviz configuration that is already subscribed to the relevant topic. 

### Grasp Generation

Inside the launch folder in *grasp_lib*, launch files for launching different grasping networks (inference) can be found. Some default models are available; however, some might need to be trained from scratch or downloaded from the original repos. The specific model can be specified in each launch file. Make sure to follow the installation of the choosen grasping repo before starting the grasp generation. 

[**install GG-CNN**](https://github.com/dougsm/ggcnn) and Launch GG-CNN:
```shell
roslaunch grasp_lib ggcnn_ros.launch # uses default model. Best model Graspnet
or
roslaunch grasp_lib ggcnn_ros.launch network:=path/to/model
```

[**install GG-CNN**](https://github.com/runeg96/robotic-grasping/tree/038a5723b54300cf3dfa1663aa8a9b49f3606218) and Launch GR-Covnet:
```shell
roslaunch grasp_lib gr_grasp_ros.launch # uses default model.
or
roslaunch grasp_lib gr_grasp_ros.launch network:=path/to/model 
```

## Training and Evaluation

The networks *ggcnn* and *gr_grasp*, have both been modified for training and evaluation for this project. In addition to training on the Cornell and the Jacquard datasets, the Graspnet dataset have been implemented as an option.


### Datasets

To get the datasets they can be downloaded from different places.

#### Cornell

1. As [the official Cornell Dataset website](http://pr.cs.cornell.edu/grasping/rect_data/data.php) has been down for a while, it is recommended to download the dataset from [this Kaggle page](https://www.kaggle.com/oneoneliu/cornell-grasp).
2. Extract the dataset and convert the PCD files to depth images by running `python -m utils.dataset_processing.generate_cornell_depth <Path To Dataset>`, from the *gr_grasp* submodule.

#### Jacquard

1. Download and extract the [Jacquard Dataset](https://jacquard.liris.cnrs.fr/).

#### Graspnet

1. Download and extract *Train Images*, *Test Images* and *Rectangle Grasp Labels* from the [Graspnet Dataset](https://graspnet.net/datasets.html).
2. Structure the data as explained in the *Format* section on the dataset website. A simplified version is seen below, with the minimum required files.
3. Convert the dataset using `python convert_graspnet.py --dataset-path <Path To Dataset> --fric-coef 0.4` from *grasp_lib/src/dataset_processing*

```shell
|-- scenes
    |-- scene_0000
    |   |-- realsense                       # data of realsense camera
    |   |   |-- rgb                         
    |   |   |   |-- 0000.png to 0255.png    # 256 rgb images
    |   |   `-- depth
    |   |   |   |-- 0000.png to 0255.png    # 256 depth images
    |   |   `-- rect
    |   |   |   |-- 0000.npy to 0255.npy    # 256 2D planar grasp labels
    |   |   |   
    |   `-- kinect
    |       |-- same structure as realsense
    |
    `-- scene_0001
    |
    `-- ... ...
    |
    `-- scene_0189
```
#### Multi-view

1. Download and extract the [Multi-view Dataset](https://www.kaggle.com/runegrnhj/cornell-inspired-multiview-grasping-dataset).

### Training

To train *ggcnn* or *gr_grasp* on a set of data, the following has to be run respectively:

```shell
# cd into the GG-CNN workspace
cd path_to_workspace/src/lh7-handover/grasping_ros/src/grasp_lib/src/ggcnn_wrapper
python3 train_ggcnn.py --dataset name_of_dataset --dataset-path path_to_dataset --network ggcnn2
```

```shell
cd path_to_workspace/src/lh7-handover/grasping_ros/src/gr_grasp
python3 train_network.py --dataset name_of_dataset --dataset-path path_to_dataset 
```

The name of dataset spesifies which dataset to use. It can be one of the following names:
- cornell
- jacquard
- graspnet
- custom

The custom dataset name is to use our own [**Multi-view dataset**](https://www.kaggle.com/runegrnhj/cornell-inspired-multiview-grasping-dataset).

#### Example
Traning GG-CNN2 with cornell
```shell
python3 train_ggcnn.py --dataset cornell --dataset-path /Downloads/cornell --network ggcnn2
```


### Evaluation

To evaluate *ggcnn* or *gr_grasp* on a dataset, the following has to be run respectively:

```shell
cd path_to_workspace/src/lh7-handover/grasping_ros/src/grasp_lib/src/ggcnn_wrapper
python3 ggcnn_eval.py --dataset dataset_name --dataset-path path_to_dataset --network path_to_model
```

```shell
cd path_to_workspace/src/lh7-handover/grasping_ros/src/gr_grasp
python3 evaluate.py --dataset dataset_name --dataset-path path_to_dataset --network path_to_model
```