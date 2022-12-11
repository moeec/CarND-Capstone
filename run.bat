cd /home/workspace 
cd CarND-Capstone 
pip install -r requirements.txt 
cd ros 
catkin_make 
source devel/setup.sh 
roslaunch launch/styx.launch