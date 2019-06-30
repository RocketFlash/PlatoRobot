set -o errexit
set -o verbose
cd ~

#1. If not done, init catkin workspace:

mkdir -p ~/plato_ws/src

#2. Download repository
 
git clone https://cordelianew.university.innopolis.ru/gitea/r.yagfarov/plato.git ~/plato_ws/src/plato

#3. Install dependencies

git clone git@cordelia.university.innopolis.ru:bipedal/plato_deps.git ~/plato_ws/src/plato_deps

#4 Install catkin tools if not installed

sudo apt-get install python-catkin-tools

#5. Install ros controllers for work with gazebo simulation.

sudo apt-get update
sudo apt-get install -y ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control

#6. Add path to project to bashrc and add path to Gazebo models

echo 'PYTHONPATH=$PYTHONPATH:~/plato_ws/src/plato' >> ~/.bashrc
echo 'source ~/plato_ws/devel/setup.bash' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/plato_ws/src/plato/plato_gazebo/models:~/.gazebo/models' >> ~/.bashrc

#7. Install additional packages
sudo apt-get install -y ros-melodic-velodyne ros-melodic-move-base ros-melodic-hector-sensors-gazebo

#8. Install gmapping from sources
cd ~/plato_ws/src/plato_deps
git clone https://github.com/ros-perception/openslam_gmapping src/openslam_gmapping
git clone https://github.com/ros-perception/slam_gmapping src/slam_gmapping
rosdep install --from-paths ~/plato_ws/src/plato_deps -i

#9. Build
cd ~/plato_ws
catkin build

#10. Source .bashrc
source ~/.bashrc
