## Setup workspace

### Prerequisites

It is a requirement to have `docker engine` already installed in the host machine.

* See [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed. *Skip this step if you don't have an NVIDIA graphics card*


* Make sure you have the drivers installed:
  ```sh
  nvidia-smi
  ```
* See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

### Build and run container

1. Build

There are two build options:
  1. Option A: Only For Demo!
    - Container will install o3de binaries and compile the andino_o3de project.

  3. Option B: Using o3de binaries. (Development)
     - Container will have the o3de-sdk already installed. The user will only need to build the project

  ```
  ./docker/build.sh binaries
  ```

  3. Option C: Installing o3de from source. (Development)
     - Container will only install the requirements. The user will need to build the project along the engine. (build time ~ 2hs)

  ```
  ./docker/build.sh base
  ```

2. Run

  ```
  ./docker/run.sh
  ```

## Demo!

After building the image using the `demo` option simply initializing the container should launch the Editor.

  ```
  ./docker/run.sh
  ```

## Development: First use


### Using Option B: o3de binaries installed in the container:


1. Register the projects

Export some convenient env vars.
```
export O3DE_VERSION=23.10.2
export USERNAME=$(whoami)
```
Also add it to the .bashrc file for using it in next session.
```
echo "export O3DE_VERSION=23.10.2" >> ~/.bashrc
echo "export USERNAME=$(whoami)" >> ~/.bashrc
```
Finally register the `andino_o3de` in the Engine installation.
```
/opt/O3DE/$O3DE_VERSION/scripts/o3de.sh register --project-path /home/${USERNAME}/andino_o3de_ws/andino_o3de
```

2. Build the project

First let's create a `o3de-packages` folder at the same level so it is mounted and we avoid to download packages in the future.

```
cd /home/$USERNAME/andino_o3de_ws/
mkdir o3de-packages
```

 - Build the andino_o3de project:
```
cd /home/$USERNAME/andino_o3de_ws/andino_o3de
cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_3RDPARTY_PATH=/home/$USERNAME/andino_o3de_ws/o3de-packages
cmake --build build/linux --target andino_o3de.GameLauncher Editor --config profile -j 8
```

3. Run editor or Game launcher.

As we are using sdk engine / binaries we should launch editor from o3de binaries (not from the project)
```
/opt/O3DE/$O3DE_VERSION/bin/Linux/profile/Default/Editor --project-path /home/$USERNAME/andino_o3de_ws/andino_o3de
```

or the game launcher (this from the binaries from the project)
```
cd /home/$USERNAME/andino_o3de_ws/andino_o3de
./build/linux/bin/profile/andino_o3de.GameLauncher
```

4. When exiting the container mind overwriting the current state so initial setup isn't necessary next time you `run.sh` the container.

```
~/andino_o3de_ws/andino_o3de$ exit
exit
access control enabled, only authorized clients can connect
Do you want to overwrite the image called 'andino_o3de_ws_image' with the current changes? [y/n]: y
```

### Using Option C: building engine from source for the project:

TODO: Polish this section. The following is a mere draft.

Clone the sources:
```
cd /home/${USERNAME}/andino_o3de_ws/
git clone o3de
--> Check git lfs
git clone o3de-extras
git clone o3de-rgl-gem
```
Be sure to checkout version 2310.2

```
export O3DE_INSTALL=/home/${USERNAME}/andino_o3de_ws/o3de
```
#### Register sdk engine
```
cd to o3de
scripts/o3de.sh register --this-engine
```
#### Register gems
```
scripts/o3de.sh register --all-gems-path ${O3DE_INSTALL}/../o3de-extras/Gems
scripts/o3de.sh register --all-templates-path ${O3DE_INSTALL}/../o3de-extras/Templates
scripts/o3de.sh register --all-projects-path ${O3DE_INSTALL}/../o3de-extras/Projects
```
```
scripts/o3de.sh register --gem-path ${O3DE_INSTALL}/../o3de-rgl-gem --force
```

## Appendix

### How to create a new project on O3DE using CLI

Go to o3de location folder, whether it is installed via binaries or source.

```
export PROJECT_NAME=my_new_project
scripts/o3de.sh create-project -tn Ros2FleetRobotTemplate -pp /home/$USERNAME/andino_o3de_ws/robot_o3de/$PROJECT_NAME
```

Enable gems that you want in the project
```
scripts/o3de.sh enable-gem -gn RGL -pn $PROJECT_NAME
scripts/o3de.sh enable-gem -gn Terrain -pn $PROJECT_NAME
scripts/o3de.sh enable-gem -gn Vegetation -pn $PROJECT_NAME
scripts/o3de.sh enable-gem -gn RosRobotSample -pn $PROJECT_NAME
scripts/o3de.sh enable-gem -gn ProteusRobot -pn $PROJECT_NAME
scripts/o3de.sh enable-gem -gn WarehouseAssets -pn $PROJECT_NAME
scripts/o3de.sh enable-gem -gn WarehouseAutomation -pn $PROJECT_NAME
scripts/o3de.sh enable-gem -gn WarehouseSample -pn $PROJECT_NAME
```

Then build as you did before.
When building all (engine included) from source you can also add:

```
cd to the project path
```
```
-DLY_DISABLE_TEST_MODULES=ON \
  -DLY_STRIP_DEBUG_SYMBOLS=ON \
  -DAZ_USE_PHYSX5=ON
```
this is :
```
cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_3RDPARTY_PATH=/home/$USERNAME/andino_o3de_ws/robot_o3de/o3de-packages -DLY_DISABLE_TEST_MODULES=ON -DLY_STRIP_DEBUG_SYMBOLS=ON -DAZ_USE_PHYSX5=ON
```
Then:
```
cmake --build build/linux --target $PROJECT_NAME.GameLauncher Editor --config profile -j 8
```

