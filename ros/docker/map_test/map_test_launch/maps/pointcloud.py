import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from open3d import * # Add 'pip install' open3d to Dockerfile 
import torch

def points_to_image_torch(xs, ys, ps, sensor_size=(180, 240)):
    xt, yt, pt = torch.from_numpy(xs), torch.from_numpy(ys), torch.from_numpy(ps)
    img = torch.zeros(sensor_size,dtype=float)
    img.index_put_((yt, xt), pt, accumulate=True)
    return img

def main():
    cloud = io.read_point_cloud("corner-cloud-decimated.ply") # Read the point cloud
    # visualization.draw_geometries([cloud]) # Visualize the point cloud     
    points = np.asarray(cloud.points)
    x = points[:,0]
    y = points[:,1]
    print(matplotlib.scale.get_scale_names())

    colors = np.asarray(cloud.colors)
    gray = np.mean(colors,axis=1)
    print(f"min: {np.min(gray)}, max: {np.max(gray)}")
    gray = [1 if g > 0.8 else 0 for g in gray]
    # norm = matplotlib.colors.LogNorm(vmin=0.1, vmax=0.7)
    plt.hist2d(x,y,bins=[600,600],weights=gray)

    plt.show()
    plt.savefig('womp.png')

    # points_to_image_torch(x,y,gray)

if __name__ == "__main__":
    main()
