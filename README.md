# Echo
Interactive voice chat system. Currently still very much a work in progress.

## Development setup
Build the container:
```shell
docker build -t echo -f docker/.Dockerfile .
```

Configure environment variables:
```
export OPENAI_API_KEY=...
```

Run the container:
```shell
docker run -it --rm --env OPENAI_API_KEY=$OPENAI_API_KEY --device /dev/snd -v $(pwd)/ros2_workspace:/root/ros2_workspace echo
```
The parameter giving the speakers and microphone to the container `--device /dev/snd` will not work on Windows.
