![](../images/Picto+STEREOLABS_Black.jpg)

# Launch Files

Launch files provide a convenient way to start up multiple nodes and a master, as well as setting parameters.

Use **roslaunch** to open ZED launch files.
```
roslaunch zed_wrapper zed.launch
```

### Start the node with one ZED
Use the **zed.launch** file to launch a single camera nodelet. It includes one instance of **zed\_camera.launch**.
```
roslaunch zed_wrapper zed.launch
```

### Start the node with multiple ZED
To use multiple ZED, launch the **zed\_multi\_cam.launch** file which describes the different cameras. This example includes two instances of **zed\_camera.launch**.
```
roslaunch zed_wrapper zed_multi_cam.launch
```

### Start the node with multiple ZED and GPUs
You can configure the wrapper to assign a GPU to a ZED. In that case, it it is not possible to use several instances of **zed\_camera.launch** because different parameters need to be set for each ZED.
A sample **zed\_multi\_gpu.launch** file is available to show how to work with different ZED and GPUs.
```
roslaunch zed_wrapper zed_multi_gpu.launch
```
