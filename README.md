# Plane segmentation from supervoxel
Plane segmentation from  "Supervoxel Clusterling" in Point Cloud Library.

# Environment

* Ubuntu 20.04
* Opencv 4.5.2
* PCL 1.10.0
* Libfreenect2 (Optional)

# Build & How to use

## Clone & CMake

```console
git clone https://github.com/shun74/plane-segmentation-from-supervoxel.git
cd plane-segmentation-from-supervoxel
mkdir build
cd build
cmake ../src
make
```

## Run
With RGB+D images. ( .png & 16bit .raw)
```console
./main RGB_image_path Depth_image_path Configuration_file_path
```

With 3D data. ( .pcd)
```console
./main PCD_data_path Configuration_file_path
```

**Note**

Running this program requires configuration file. Some samples in /configs. Check the samples to see what parameters are available. An explanation is written for each parameter. Also, all the parameters listed in the samples are required. To change data type. In the .conf file change the PCD_DATA flag like as follows.

- When running with a set of 2d images. (RGB+D)
```
PCD_DATA = false
```
- When running with a PCD data.
```
PCD_DATA = true
```

# Working Sample

There is one sample in /samples, you can test with it.

**Sample image**
![RGB sample image](./samples/sample_color.png "RGB sample image")

## Running

```console
./main ../samples/sample_color.png ../samples/sample_depth.raw ../configs/2d_image_sample.conf
```
or
```console
./main ../samples/sample_original.pcd ../configs/3d_sample.conf
```

## Results
**USE_REDUCED_PCD = true**
![Sparse pcd sample](./images/working_sample1.png "sparse sample")
**USE_REDUCED_PCD = false**
![Dense pcd sample](./images/working_sample2.png "dense sample")

# Supplementary Material
## What is Kinect ?

**Kinect V2**
![Kinect V2](./images/kinect_v2.jpg "kinect v2")
## What is Supervoxel Clusterling ?

## Get data from Kinect V2