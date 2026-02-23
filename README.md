# Workshop Robotic Computer Vision 2026
This repository contains the materials for the Robotic Computer Vision Workshop 2026. The workshop will cover the following topics:

- Introduction to robotics
- KUKA robot Operation
- Robot programming in Rhino/Grasshopper
- Introduction to ROS2
- Setting up the development environment using Docker
- Basic ROS2 concepts and tools
- Basic robotic milling

## Schedule

| Day | Time | Topic |
| --- | --- | --- |
| 1 | 13:00  - 16:00  | Introduction to Robotics / Grouping / ROS: MoveIt and Motion Planning |
| 2 | 13:00  - 16:00  | Scanning and 3D Modeling |
| 3 | 09:30  - 16:00  | Design and Fabrication |
| 4 | 13:00  - 16:00  | Wrap-up |

## Course Materials

- Rhino / Grasshopper
- [KUKA prc and tutorials](https://drive.google.com/drive/folders/120M1iAXMrxJXd4COS3VNXvtPHMMm_vb0?usp=share_link) and [slides](https://app.rccn.dev/slidev/KUKA)
- [Visual Studio Code](https://code.visualstudio.com/), with extensions [devcontainer](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) for Windows/Mac or [Colima](https://colima.io/) for Mac users famaliar with brew.

## ROS
1. open this repository in Visual Studio Code
2. modify the `docker-compose.yml` file to set the `ROS_DOMAIN_ID` environment variable to a unique value (e.g., `ROS_DOMAIN_ID=352095`) to avoid conflicts with other ROS 2 instances on your network
3. click the `><` button in the bottom left corner and select "Reopen in Container"
4. wait for the container to build and open a new terminal inside the container
5. open browser and go to `http://localhost:6080/` to access the VNC desktop environment