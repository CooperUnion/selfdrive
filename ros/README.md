# ROS Environment
## Prerequisite
Refer to [documentation](https://confluence.cooperigvc.org/display/TECH/Initial+Environment+Setup) on how to install each component
- Docker Engine
- Docker-Compose
- NVIDIA-Docker

---
## Usage
### Starting services
1. Allow docker to xhost
```bash
xhost +si:localuser:root
```
2. Spin up the containers ***[YOU MUST ALWAYS SPIN UP MASTER]***
```bash
docker-compose up -d master <services>
```
Where <services> are the containers you want to spin up from the docker-compose.yml file. 

For example, if you want to spin up the zed container, run the following command: 
```bash
docker-compose up -d master zed
```
### Stopping services
```
docker-compose down
```

---
## Available Services
- master
- gazebo
- rplidar
- teleop
- zed
