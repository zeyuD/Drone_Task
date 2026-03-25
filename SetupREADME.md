## install commands and specifications
Ros2 humble
steps here
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


install Px4 branch release 1.16 stable

git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.16 

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

install px4 required messages goes in ros2ws src
git clone https://github.com/PX4/px4_msgs.git -b release/1.16

install Auterion helper library 1.16 release goes in ros2ws src
git clone https://github.com/Auterion/px4-ros2-interface-lib.git --recursive -b release/1.16 


//note will not work without make on the system
install microdds client
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/


Qgroundcontrol

download daily build of qgroundcontrol follow linux instructions from here: https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html

use tis file for the qgc
https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/releases/daily_builds.html

#project start steps ros2 system

create folder for ros2 modules with src subdirectory
copy px4 messages and px4-ros2-interface to src of system.
copy both launch_Files and cmd_process to src. Modify following comments each section.

#research libraries/projects
for later use. needed for camera gazebo linking 
sudo apt install ros-humble-ros-gzharmonic

#running gazebo will run px4 sim. MUST RUN qgc as well or droe will not arm
make px4_sitl gz_x500


run ros2 node for testing ex
ros2 run packagename nodename
