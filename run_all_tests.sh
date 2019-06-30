set -e
python -m unittest discover -p '*_test.py'
rostest plato_gazebo gazebo_test.launch