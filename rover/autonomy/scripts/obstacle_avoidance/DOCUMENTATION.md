# DWA Planner
This is like notes of most of my decision making while coding this file, it's not concise

- Facing issues with taking in parameters from launch file
    - Solution 1: Use `ros::param::get("~v_min", this->vr.v_min)` instead of `nh.param<double>("v_min", this->vr.v_min, 0.0)`
        - This would work but it would not let me set default values through the function
    - **Solution 2:** Make a private node handle using `ros::NodeHandle pnh("~")` and give the define function the private handle
    
    - Solution 3: If I load the parameters (`<rosparam file="$(find rover)/rover/autonomy/config/dwa_planner.yaml" command="load" />`) outside the `<node>` tag, it will be in global space and it will be taken by the planner file