# Install

To install this stack for velmobil robot you need:
1. Install ros-kinetic-desktop package: ```sudo apt install ros-kinetic-desktop```
2. Source ROS kinetic setup bash script:
``` source /opt/ros/kinetic/setup.bash```
3. Get setup script from this repository: https://github.com/dudekw/velmwheel/blob/global-localization/scripts/setup.bash
4. Get rosinstall file from this repository: https://github.com/dudekw/velmwheel/blob/global-localization/scripts/velmwheel_sim_kinetic.rosinstall
5. Make directory for the workspace: eg. ```mkdir ~/ws_velmobil```
6. Launch the setup script from this repository with two arguments: 
```bash setup.bash <full path to workspace directory> <full path to rosinstall file>```

   e.g. ```bash setup.bash /home/my_user_name/ws_velmobil /home/my_user_name/velmwheel_sim_kinetic.rosinstall```
