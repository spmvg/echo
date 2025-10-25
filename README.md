# Echo
Interactive voice chat system.

## Development setup
Build the container:
```shell
docker build -t echo -f docker/.Dockerfile .
```

Run the container:
```shell
docker run -it --rm --device /dev/snd -v $(pwd)/ros2_workspace:/root/ros2_workspace echo
```

Go to the workspace directory and build the package:
```shell
cd /root/ros2_workspace
colcon build
source install/local_setup.bash
```

Run the package:
```shell
ros2 run echo stt_onboard
```