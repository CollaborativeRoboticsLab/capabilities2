## Docker based startup information

### Setup the Capabilities2 container

Pull and start the docker container

```bash
cd docker
docker compose pull 
docker compose up -d
```

### Starting the Capabilities2 server

On the host computer, same terminal or a new one

```bash
docker exec -it capabilities2:jazzy bash
ros2 launch capabilities2_server server.launch.py
```

### Starting the Capabilities2 Fabric

On the host computer, iIn a new terminal

```bash
source install/setup.bash
ros2 launch capabilities2_fabric fabric.launch.py
```
