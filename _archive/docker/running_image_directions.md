# Using rsx_docker_run.sh

This is a shell script that starts a container from the rsx_dev_rsx image (that you should've created from the Dockerfile using the docker_build.sh script).

The script takes **two positional arguments:**
1. your ipv4 address (used for connecting to X server for display)
2. path to the cloned rsx-rover repo on your local computer (used to establish a bind mount)

You can find your ipv4 address by typing `ipconfig` into a terminal (e.g., bash, powershell, windows cmdline).

For example, if my ipv4 address was 10.0.0.1, and the rsx-rover repo on my computer was at "C:\bryan\rsx-rover", then the command I would run is:
> `./rsx_docker_run.sh 10.0.0.1 "C:\bryan\rsx-rover"`

And in general, the command is:

> `./rsx_docker_run.sh <ipv4-address> <path-to-repo>`
- remember the quotes around the file path
- remember the `./` preceding the command

After running the command you'll get a container that you can now work with. You can also then connect to it using VScode.

**The Bind Mount:** the repo on your local computer will be mounted to the `/home/rsx/rover_ws/src/rsx-rover` directory in the container. This means any changes you make to that directory in the container will also change the directory on your local computer and vice versa; they are effectively the same directory.
