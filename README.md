# mujoco-urdf-package-uri

Experiments related to https://github.com/google-deepmind/mujoco/issues/1432 .

Run examples with [pixi](https://pixi.sh):

~~~
# Download and build modified version of mujoco
pixi download_mujoco
pixi install_mujoco

# Load in MuJoCo an unmodified URDF model of panda from https://github.com/ros-planning/moveit_resources/tree/ros2/panda_description
pixi install example_panda
# Load in MuJoCo an unmodified URDF model of iCub from https://github.com/robotology/icub-models
pixi install example_icub
~~~
