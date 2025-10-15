# Echo
Interactive voice chat system.

## Development setup
Build the container:
```shell
docker build -t echo -f docker/.Dockerfile .
```

Run the container:
```shell
docker run -it -v $(pwd -W)/ros2_workspace:/root/ros2_workspace echo
```
