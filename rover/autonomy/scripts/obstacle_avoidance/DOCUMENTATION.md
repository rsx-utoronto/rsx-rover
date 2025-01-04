# DWA Planner
This is like notes of most of my decision making while coding this file, it's not concise

- Facing issues with taking in parameters from launch file
    - Solution 1: Use `ros::param::get("~v_min", this->vr.v_min)` instead of `nh.param<double>("v_min", this->vr.v_min, 0.0)`
        - This would work but it would not let me set default values through the function
    - **Solution 2:** Make a private node handle using `ros::NodeHandle pnh("~")` and give the define function the private handle
    
    - Solution 3: If I load the parameters (`<rosparam file="$(find rover)/rover/autonomy/config/dwa_planner.yaml" command="load" />`) outside the `<node>` tag, it will be in global space and it will be taken by the planner file

- I tried using TF lookup for pose just for the kicks (I think it's supposed to be a more accurate approach)
    - I couldn't get it to work so currently just using odom for pose
    - Important stuff for ROS TF (courtesy of ChatGPT):

        In ROS TF2, the function signature is:

        ```cpp
        lookupTransform(const std::string& target_frame,
                        const std::string& source_frame,
                        const ros::Time& time);
        ```

        This returns the transform **that will convert data from** `source_frame` **into** `target_frame`. So:

        ```cpp
        lookupTransform("map", "base_link", ros::Time(0));
        ```

        - **Target** = `"map"`
        - **Source** = `"base_link"`

        If you have a point (or pose) expressed in the **`base_link`** frame, you multiply it by this transform to express it in the **`map`** frame.


- NOTE: The `InBBX` check doesn't work with octree_ for some reason. Don't use it in future.

- I noticed that out of the queried cells, only less than 1/3rd have nodes in the octomap. Other nodes don't even exist. Is this because of out of bounds errors (which seems unlikely after noticing the values) or is it because of interpolation?
    - ChatGPT's answer (which I think makes sense): 
        - In an **octomap::OcTree**, it’s actually **quite normal** for a large fraction of the coordinates you query (`search(x, y, z)`) to return `nullptr` (i.e., no node). It does *not* necessarily mean they are out of bounds; often it just means that **no specific leaf node** at that exact coordinate exists at the current resolution, or the space was never observed (“unknown” in octomap terms).
        - **OctoMap stores data in an octree structure** where space is subdivided into voxels at discrete resolutions (e.g. 0.05 m or 0.1 m).  
        - Only **occupied** voxels and certain **subdivided free** voxels are stored explicitly; large expanses of free (or unknown) space are often represented by **coarser nodes** at higher levels in the octree.
        - Hence, when you do:

            ```cpp
            octomap::OcTreeNode* node = octree_->search(x, y, z);
            ```
            …**if** \((x, y, z)\) **is not exactly the center of a leaf voxel** (or if it’s in an area the sensor never observed), you will likely get a `nullptr`. This does *not* mean it’s strictly out of bounds; it can simply be:
            - **At a coarser level** in the tree (so the specific leaf doesn’t exist), or  
            - **Completely unknown** (never observed).  

        - If you really need a more nuanced check, you can do something like:
            ```cpp
            OcTreeNode* node = octree_->search(x, y, z, depth);
            ```
            where you specify a deeper or shallower depth to allow the tree to return a node at some coarser level. You can then check if that node is free or occupied at that coarser resolution.
    - I am making a decision to consider the nodes that don't exist as unoccupied. If that creates a lot of problems, I will reconsider, maybe check a group of coordinates or check the distance to nearest obstacle or smth.

    - A few ways to emulate a nearest neighbour search in case the node is not found:
        - OctoMap **does not** provide a built-in “nearest-neighbor” search for leaf nodes, so there’s no single function call like `searchNearest(x, y, z)`. If your `search(...)` call returns `nullptr`, that typically means:

            - The queried point lies in unknown space, or  
            - It falls into a higher-level (coarser) node that has not been subdivided down to leaf resolution.
        - **Query at coarser depth**:
            By default, `search(x, y, z)` searches at the **full/leaf depth**. However, you can specify a smaller `depth`, which makes OctoMap return coarser (higher-level) nodes if a leaf-level node does not exist. For example:

            ```cpp
            unsigned int depth = octree->getTreeDepth() - 1; 
            // or any value < getTreeDepth()
            octomap::OcTreeNode* node = octree->search(x, y, z, depth);
            ```

            If a coarser node exists at that coordinate, `node` will be non-null. This isn’t a true “nearest neighbor” search, but it effectively says, “If I can’t find a leaf node, at least give me the coarser node covering (x, y, z).” 
        
        - **Expand or subdivide free space**

            OctoMap often stores large free regions at higher levels. You can force the octree to subdivide more finely, so that future queries at leaf resolution will succeed. For instance:
            ```cpp
            octree->expand();
            ```
            This tells the tree to expand all nodes (down to the max depth), so free space gets subdivided into child nodes. Then a subsequent `search(x, y, z)` might return a leaf node for those free coordinates rather than `nullptr`. (Be aware this can **dramatically** increase memory usage.)
## Obstacle Cost System

- **Current System:** I have implemented a height cost for obstacles. Every time a point in the robot footprint intersects with an occupied voxel, the max height at that (x, y) is calculated and added to the cost.
- **Problems:**
    - The cost becomes too high too fast especially if the code thinks that the robot footprint is in an obstacle.
    - If the robot footprint is calculated with higher resolution, height cost would get larger in case of collisions
    - Both of these problems cause height cost to be weighed more inevitably because of it's big number
- **Solution:**
    - Only calculate the height cost once per robot footprint, the max height the robot has to go through instead of the max height of every colliding point in the footprint
        - This could reduce the weight of height cost though which might cause the robot to choose trajectories that go through wall just to get a lower distance cost


## Future Goals

- Currently only supporting cuboid footprint for robot. Should be generalized to any convex polygon in future. Should also be generalized for 3D shapes.
- No feasible trajectory found
    - Most probable reason: Rover footprint is in an obstacle
        - Either have a recovery behaviour or override this case somehow (maybe introducing cost for obstacles instead of yes/no)
- Prevent a trajectory from going towards a wall and then realizing late
    - Have the obstacle cost - distance from nearest obstacle
- Try giving dwa it's own velocity (from cmd_vel topic) instead of odom data (just in case odom data is really bad or in case rover doesn't actually move at the velocity that it's given)

- The rover doesn't move at velocities lower than 0.1 so fix that in code so dwa doesn't tell rover to move at that velocity

- A problem I don't know how to solve: When distance is huge, it is automatically weighted more. For instance distance cost can be 300 but the other costs are always small (heading cost and height cost can't be as big). So any change in heading cost/height cost doesn't affect as much as the distance cost. However the distance cost can also be as small as 2 so I can't just weight it less because in that case, it is still important. So I am not sure how do I normalize these costs so they are in the same space and I can apply weights without thinking about some cost value blowing up or being too low in the same navigation.