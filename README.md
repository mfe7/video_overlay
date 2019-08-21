
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

If you don't have a homography matrix already, you should:
- find a recognizable moment in your video & take a screenshot
- find the corresponding vicon coordinates during that moment (from the bag)
- Click on the corresponding vehicle positions in the screenshot, using:
```
rosrun video_overlay video_overlay_node <screenshot_filename>
```
- Add the vicon and img coords to the list of pts used to compute H

Contact: mfe@mit.edu
