from freenect2 import Device, FrameType
import numpy as np
import cv2

device = Device()
frames = {}
with device.running():
    for type_, frame in device:
        frames[type_] = frame
        if FrameType.Color in frames and FrameType.Depth in frames:
            break

rgb, depth = frames[FrameType.Color], frames[FrameType.Depth]
undistorted, registered, big_depth = device.registration.apply(
    rgb, depth, with_big_depth=True)

depth_img = big_depth.to_array()
depth_img = cv2.flip(depth_img, 1)
depth_raw = depth_img.flatten().astype(np.uint16)
depth_raw.tofile("depth.raw")

rgb_img = rgb.to_array()
rgb_img = cv2.flip(rgb_img, 1)
cv2.imwrite("color.png", rgb_img)

with open("camera_parameter.txt", "w") as f:
    f.writelines("Camera Parameters for reconstruction.\n")
    f.writelines(f"width: {rgb_img.shape[1]}\n")
    f.writelines(f"height: {rgb_img.shape[0]}\n")
    f.writelines(f"fx: {device.color_camera_params.fx}\n")
    f.writelines(f"fy: {device.color_camera_params.fy}\n")
    f.writelines(f"cx: {device.color_camera_params.cx}\n")
    f.writelines(f"cy: {device.color_camera_params.cy}")

with open('original.pcd', 'wb') as fobj:
   device.registration.write_big_pcd(fobj, big_depth, rgb)

import matplotlib.pyplot as plt
plt.figure(figsize=(12,7))
plt.subplot(1,2,1)
plt.axis("off")
plt.title("color.png")
plt.imshow(cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB))
plt.subplot(1,2,2)
plt.axis("off")
plt.title("depth.png")
plt.imshow(depth_img)
plt.show()