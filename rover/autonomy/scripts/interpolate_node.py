#!/usr/bin/python3.8

# 1. Subscribe to PCL
# 2. Write to a PCD file (pcd original)
# 3. Interpolate original PCD and write new PCD file (combined pcd)
# 4. Read combined pcd and run grid map conversion
# 5. Run traversibility filter on converted grid map, publish traversability occuapncy grid
# Loop 1-5
# 1-2: Can be done in 1 node (subcribe: lidar pcl)
# 3: Python Script
# 4 & 5 Already in launch file 
# 

import numpy as np
import open3d as o3d
from scipy.interpolate import NearestNDInterpolator
import rospy
import os
from time import time

def main():
    rospy.init_node('interpolate_node')

    home = os.path.expanduser('~/')
    
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        tic = time()

        pcd = o3d.io.read_point_cloud(home + "rover_temp/raw.pcd")
        
        points = np.asarray(pcd.points)
        
        print(points.shape)

        # Filter out all points further than 25m from origin
        dist_filter = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        points = points[dist_filter <= 15]

        print(points.shape)        

        xy = points[:, :2]
        z = points[:,2]
        interpolator = NearestNDInterpolator(xy, z)

        radius = 15 # meters
        footprint = 0.2 # do not gerneate points within this dist to origin
        n_points = 500000 # num new points to generate
        lidar_height = 1.5208 # generate points +- this value in z

        int_xy = np.zeros((n_points, 2))

        i = 0
        while i < n_points:
            seed = np.random.uniform([-radius, -radius, -lidar_height], [radius, radius, lidar_height])
            
            if footprint <= np.linalg.norm(seed[:2]) <= radius: #if new point is within radius to origin
                int_xy[i] = seed[:2]
                i += 1

        int_z = interpolator(int_xy)
        int_points = np.hstack([int_xy, int_z[:, None]])
        
        filtered_points = np.abs(int_points[:, 2] + lidar_height) <= 1
        int_points = int_points[filtered_points]

        combined_points =  np.concatenate([points, int_points], axis=0)

        # Write new, combined
        pcd_writer = o3d.geometry.PointCloud()

        pcd_writer.points = o3d.utility.Vector3dVector(combined_points)
        o3d.io.write_point_cloud(home + "rover_temp/interpolated.pcd", pcd_writer, write_ascii=True)

        toc = time()
        
        print(str(toc - tic) + " sec", str(int_points.shape[0]) + " points generated")

        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
