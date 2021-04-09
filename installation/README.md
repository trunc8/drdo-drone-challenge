## Installation steps for Ardupilot
### Installing Ardupilot and MAVProxy

In home directory (both git clone and update will take time):
```
cd ~/
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-3.6
git submodule update --init --recursive
```

```
sudo apt-get update -y
sudo apt-get install -y ros-melodic-mavros geographiclib-tools python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
sudo geographiclib-get-geoids egm96-5
sudo pip install future pymavlink MAVProxy
```

### Install Gazebo plugin for APM (ArduPilot Master) :
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
```

build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

Add to .bashrc or .zshrc and reload  
(This step is V.IMPORTANT to ensure that gazebo finds the models and your terminal finds the sim_vehicle.py script)
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest:/usr/lib/ccache
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$HOME/ardupilot_gazebo/worlds
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/ardupilot_gazebo/models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/drdo-drone-challenge/interiit21/models
```


### Test installation
In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
