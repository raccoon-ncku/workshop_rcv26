#!/bin/bash
set -e

# 1. Setup Environment
cd /home/ubuntu/workshop_ws
source /opt/ros/jazzy/setup.bash

# 2. Automate X11 Permissions
xhost +local:root > /dev/null 2>&1 || echo "export DISPLAY=:1" >> ~/.bashrc

# 3. Import & Dependency Sync
echo "--- Importing Repositories ---"
vcs import src < workshop.repos

echo "--- Installing System Dependencies ---"
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

# 4. Build
echo "--- Building Workspace ---"
colcon build --symlink-install

# 5. Clean Bashrc
sed -i '/workshop_ws\/install\/setup.bash/d' ~/.bashrc
echo "source /home/ubuntu/workshop_ws/install/setup.bash" >> ~/.bashrc

echo "--- Setup Complete! ---"