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