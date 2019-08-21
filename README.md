
# Video Overlay Tool

![](./example_screenshot.png)

### Overview
Inputs:
- ROS bag file with several vehicles' trajectories (on `/<veh_name>/pose` topics)
- video from a stationary camera
- homography matrix btwn camera plane and **planar** vehicle pose frame

Outputs:
- video with trajectories overlayed on the stationary camera's stream

### Instructions
Clone the repo, `catkin_make`, then:

```
rosrun video_overlay video_overlay_node
```

Contact: mfe@mit.edu
