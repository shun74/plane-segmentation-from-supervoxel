# Plane segmentation from supervoxel
Plane segmentation from  *Supervoxels Clustering* in Point Cloud Library.
A simple "Region Merge" algorithm using normal and centroid of regions detects planes from roughly labeled pcd created by *Supervoxels Clustering*. This is a very simple approach with robust performance and can be applied to areas such as ROS.

# Environment

* Ubuntu 20.04 LTS
* Opencv 4.5.2
* PCL 1.10.0
* Libfreenect2 (Optional)

# Build & How to use

## Clone & CMake

```
git clone https://github.com/shun74/plane-segmentation-from-supervoxel.git
cd plane-segmentation-from-supervoxel
mkdir build
cd build
cmake ../src
make
```

## Run
With RGB+D images. ( .png & 16bit .raw)
```
./main RGB_image_path Depth_image_path Configuration_file_path
```

With 3D data. ( .pcd)
```
./main PCD_data_path Configuration_file_path
```

### **Note**

Running this program requires configuration file. (Some samples in /configs.) Check the samples to see what parameters are available. An explanation is written for each parameter. Also, all the parameters listed in the samples are required. To change data type, change the PCD_DATA flag in the .conf file like as follows.

- When running with a set of 2d images. (RGB+D)
```
PCD_DATA = false
```
- When running with a PCD data.
```
PCD_DATA = true
```

# Working Sample

One sample in "/samples" so you can test with it.

**Sample image**
![RGB sample image](./samples/sample_color.png "RGB sample image")

## Running

```
./main ../samples/sample_color.png ../samples/sample_depth.raw ../configs/2d_image_sample.conf
```
or
```
./main ../samples/sample_original.pcd ../configs/3d_sample.conf
```

## Results
**USE_REDUCED_PCD = true** (To cut down computational costs.)
![Sparse pcd sample](./images/working_sample1.png "sparse sample")
**USE_REDUCED_PCD = false**
![Dense pcd sample](./images/working_sample2.png "dense sample")

# Supplementary Material
## What is Kinect ?

*Kinect V2* is an RGB+D sensor released by Microsoft in 2013. The original *Kinect V1* had a RGB resolution of 640 * 480, while the V2 has a full HD resolution of 1920 * 1080. The Depth sensor also has a resolution of 512 * 424 and can measure a range of 0.5m~4.5m. Recently, a smaller version with higher performance called *Azure Kinect* has been released. (Let's not mention that Kinect was a sensor for Xbox games.)

**Kinect V2**
![Kinect V2](./images/kinect_v2.jpg "kinect v2")

## What is Supervoxels Clustering ?

[PCL Official Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/supervoxel_clustering.html)

## Get data from Kinect V2