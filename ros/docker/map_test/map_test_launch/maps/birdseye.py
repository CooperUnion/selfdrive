import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from open3d import * # Add 'pip install' open3d to Dockerfile 
import torch
import pcl
# from show import *
# from tool import *

HRES = 0.35         # horizontal resolution (assuming 20Hz setting)
VRES = 0.4          # vertical res
VFOV = (-24.9, 2.0) # Field of view (-ve, +ve) along vertical axis
Y_FUDGE = 5         # y fudge factor of velodyne HDL 64E

def points_to_image_torch(xs, ys, ps, sensor_size=(180, 240)):
    xt, yt, pt = torch.from_numpy(xs), torch.from_numpy(ys), torch.from_numpy(ps)
    img = torch.zeros(sensor_size,dtype=float)
    img.index_put_((yt, xt), pt, accumulate=True)
    return img
    
def scale_to_255(a, min, max, dtype=np.uint8):
    """ Scales an array of values from specified min, max range to 0-255
        Optionally specify the data type of the output (default is uint8)
    """
    return (((a - min) / float(max - min)) * 255).astype(dtype)

def birds_eye_point_cloud(points, side_range=(-10, 10),fwd_range=(-10,10),res=0.1, min_height = -2.73, max_height = 1.27,saveto=None): # fix params
    x_lidar = points[:, 0]
    y_lidar = points[:, 1]
    z_lidar = points[:, 2]
    # r_lidar = points[:, 3]  # Reflectance

    # INDICES FILTER - of values within the desired rectangle
    # Note left side is positive y axis in LIDAR coordinates
    ff = np.logical_and((x_lidar > fwd_range[0]), (x_lidar < fwd_range[1]))
    ss = np.logical_and((y_lidar > -side_range[1]), (y_lidar < -side_range[0]))
    indices = np.argwhere(np.logical_and(ff,ss)).flatten()

    # CONVERT TO PIXEL POSITION VALUES - Based on resolution
    x_img = (-y_lidar[indices]/res).astype(np.int32) # x axis is -y in LIDAR
    y_img = (x_lidar[indices]/res).astype(np.int32)  # y axis is -x in LIDAR
                                                     # will be inverted later

    # SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
    # floor used to prevent issues with -ve vals rounding upwards
    x_img -= int(np.floor(side_range[0]/res))
    y_img -= int(np.floor(fwd_range[0]/res))

    # CLIP HEIGHT VALUES - to between min and max heights
    pixel_values = np.clip(a = z_lidar[indices],
                           a_min=min_height,
                           a_max=max_height)

    # RESCALE THE HEIGHT VALUES - to be between the range 0-255
    pixel_values  = scale_to_255(pixel_values, min=min_height, max=max_height)

    # FILL PIXEL VALUES IN IMAGE ARRAY
    x_max = int((side_range[1] - side_range[0])/res)
    y_max = int((fwd_range[1] - fwd_range[0])/res)
    im = np.zeros([y_max, x_max], dtype=np.uint8)
    im[-y_img, x_img] = pixel_values # -y because images start from top left
    return im
    # Convert from numpy array to a PIL image
    im = Image.fromarray(im)

    # SAVE THE IMAGE
    if saveto is not None:
        im.save(saveto)
    else:
        im.show()

def main():
    cloud = io.read_point_cloud("corner-cloud-decimated.ply") # Read the point cloud
    # visualization.draw_geometries([cloud]) # Visualize the point cloud     
    points = np.asarray(cloud.points)

    # print(matplotlib.scale.get_scale_names())

    birds_eye_point_cloud(points, side_range=(-10, 10), fwd_range=(-10, 10), res=0.1, saveto="wompbe.png")

    # colors = np.asarray(cloud.colors)
    # gray = np.mean(colors,axis=1)
    # print(f"min: {np.min(gray)}, max: {np.max(gray)}")
    # gray = [1 if g > 0.8 else 0 for g in gray]
    # # norm = matplotlib.colors.LogNorm(vmin=0.1, vmax=0.7)
    # plt.hist2d(x,y,bins=[600,600],weights=gray)

    plt.show()
    plt.savefig('wompbe.png')

    # points_to_image_torch(x,y,gray)

if __name__ == "__main__":
    main()
