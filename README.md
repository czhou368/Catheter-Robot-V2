# Monocular Reconstruction Project

This project is to realize 3D reconstruction using a micro camera or Milad's camera based on the **deep-image-matching** Structure from Motion (SfM) library.

## Preparation
* Create a conda environment
```
conda create -n catheter_robot python=3.10
conda activate catheter_robot
pip install --upgrade pip
```
* Install deep-image-matching library
```
cd ..
git clone https://github.com/3DOM-FBK/deep-image-matching.git
cd deep-image-matching
pip install -e .
cd ..
```
* Intall SAM2
```
git clone https://github.com/facebookresearch/sam2.git && cd sam2
pip install -e .
cd ..
```
* Install pycolmapgs

* Install pycolmap
```
pip install pycolmap==3.10.0
```
* Install Open3d
```
pip install open3d==0.18.0
```

* Go back to this project
```
cd ../Cathetor_Robot_v1.0
```

<!-- ## Run SfM
* Change ``dir = ""`` to your data directory (now defaultly set to the skull dataset)
* Run ``python sfm.py``. This will take a while cuz the dataset contains 259 images. Please be patient^_^. Normally I will use only about 100 images and it will work fine. So basically this step uses the **deep-image-matching** library to detect and match the features, and give a reconstruction with **3D points** and **camera poses**.
* If it works well, the results will be in your_dir/results_[configurations]. At this time you only need to look at models/0/ and reconstruction/.
    * cameras.bin / cameras.txt contains the intrinsic parameters of all reconstructed cameras in the dataset using one line per camera. We only use one camera so there should only be one line of camera information.
    * images.bin / images.txt contains the pose and keypoints of all reconstructed images in the dataset using two lines per image. The first two lines define the information of the first image, and so on. The reconstructed pose of an image is specified as the projection from world to the camera coordinate system of an image using a quaternion $(Q_W, Q_X, Q_Y, Q_Z)$ and a translation vector $(T_X, T_Y, T_Z)$. The quaternion is defined using the Hamilton convention. The coordinates of the projection/camera center are given by $-R^T * T$, where $R^T$ is the inverse/transpose of the $3\times 3$ rotation matrix composed from the quaternion and $T$ is the translation vector. The local camera coordinate system of an image is defined in a way that the $X$ axis points to the right, the $Y$ axis to the bottom, and the $Z$ axis to the front as seen from the image.
    * points3D.bin / points3D.txt contains the information of all reconstructed 3D points in the dataset using one line per point.

## Get object masks
I use labelme to manually get the segmentation annotations of the object. Labelme outputs .json files so I need to transform them into binary image masks.
* Run ``python json2mask.py``. Note the ``image_size`` and ``dir = ''`` should be correct.
* I only use 2 annotations here. The stratgy of using how many masks should be discussed later.

## Visualize your reconstruction
* Simply run ``python visualize_model.py --input_model [your model dir] --input_format [.bin/.txt]`` for visualization. For example, if you have run everything unchanged, you should run ``python visualize_model.py --input_model Data/skull/results_superpoint+lightglue_sequential_quality_high/models/0/ --input_format .bin``.

## To do
* Estimate the object pose (Point cloud center / RANSAC (if known 3D model))
* Segmentation network -->

## Move Robot
Connect the xbox series joystick and run ``python move_robot.py``

## Reconstruction
Run ``python reconstruction.py default``