
# Video Overlay Tool

![](./example_screenshot.png =250x)

### Overview
Inputs:
- ROS bag file with several vehicles' trajectories (on `/<veh_name>/pose` topics)
- video from a stationary camera
- homography matrix

Outputs:
- video with trajectories overlayed on the stationary camera's stream

### Instructions
Clone the repo, `catkin_make`

```
rosrun video_overlay video_overlay_node
```

Contact: mfe@mit.edu