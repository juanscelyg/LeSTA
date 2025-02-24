# LeSTA - urban-traversability-dataset Demo

In this container is provided a minimal reproduction for examples for required packages for LeSTA. It is based on the docker configuration made by [Matias Mattamala](matias@robots.ox.ac.uk) y [Jonas Frey](jonfrey@ethz.ch) in [Wild Visual Navigation](https://github.com/leggedrobotics/wild_visual_navigation.git)

This example should be self-contained and it should run on a Docker container. This was tested on Ubuntu machines, we do not expect the GUI to run on Windows or Mac computers (due to X11 support).


## Preliminaries
The instructions assume you have Docker installed. If you haven't used Docker before, please take a look at the excellent resources prepared by Tobit Flatscher in [docker-for-robotics](https://github.com/2b-t/docker-for-robotics), which this demo builds upon.

# Get the required repositories
First clone the LeSTA and urban-traversability-dataset. Go to a working directory and run:
```shell
git clone git@github.com:juanscelyg/LeSTA.git
git clone git@github.com:Ikhyeon-Cho/urban-traversability-dataset.git
```

Then, go to the `docker` folder in `LeSTA`

```shell
cd LeSTA/docker
```

> Note: All the following commands must be executed in this folder

## Build the image
Build the Docker image running:

```sh
docker compose -f docker-compose-gui-nvidia.yaml build
```

## Run the simulation environment in the container

Start the container in detached mode:

```sh
docker compose -f docker-compose-gui-nvidia.yaml up -d
```

Launch a first bash terminal in the container to start the simulation environment:

```sh
docker compose -f docker-compose-gui-nvidia.yaml exec lesta_nvidia /bin/bash
```

Once in the container, source the `first_run` script to install the WVN package that is mounted automatically when the container starts:

```sh
source first_run.sh
```

Launch the Gazebo simulation environment and an RViz window as the main interface.

```sh
roslaunch wild_visual_navigation_jackal sim.launch 
```
If this doesn't open any window, please check the troubleshooting section below.


Open a new terminal to launch WVN in the same container:

```sh
docker compose -f docker-compose-gui-nvidia.yaml exec lesta_nvidia /bin/bash
```

And then, once you are in the container:
```sh
roslaunch terrain_dataset_player parking_lot.launch 
```

Wait until the Rviz window show the simulation environment. 

 <!-- TODO: Add the instructions to run the training and evaluation of the model -->

## Stop the example

Kill all the terminals as usual (Ctrl + D). Then, stop the container using:

```sh
docker compose -f docker-compose-gui-nvidia.yaml down
```

## Troubleshooting

If RViz doesn't show up or you'll see an error like this in the terminal:

> No protocol specified
> qt.qpa.xcb: could not connect to display :1

To fix it, you might need to allow the X Server to connect before running the container. Stop everything (including the container) and then run:

```sh
xhost +Local:*
xhost
```

Then restart the container and run the other commands. Now RViz should pop up.
