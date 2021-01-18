#!/bin/bash

function install {
    
    #general 
    sudo add-apt-repository -y ppa:x4121/ripgrep
    sudo apt-get -y update
    sudo apt-get install -y autoconf automake cmake grep htop
    sudo apt-get install -y libtool curl make g++ unzip vim wget
    sudo apt-get install -y scons ripgrep

    #install ROS Melodic
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
    sudo apt update
    sudo apt install -y ros-melodic-desktop-full

    #add dependencies to bashrc associated with ROS Melodic
    source /opt/ros/melodic/setup.bash
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

    #build off ROS Melodic dependencies
    sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python3-pip
    sudo -H pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg
    sudo apt install -y python-rosdep
    sudo rosdep init
    rosdep update
    
    #create catkin_workspace
    cd
    mkdir ~/catkin_ws
    cd catkin_ws
    mkdir src
    catkin build
    source $HOME/catkin_ws/devel/setup.bash
    echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc

    #pull down node
    cd ~/catkin_ws/src
    GIT_PULLDOWN_SUCC=128
    while [[ $GIT_PULLDOWN_SUCC -ne 0 ]]
    do
        git clone https://github.com/AscendEngineering/ascend_ros.git
        GIT_PULLDOWN_SUCC=$?
    done
    cp edge/ascend/ascend_ros/third_party/archives/CMakeLists.txt .
    cd edge/ascend/ascend_ros

    #go and install third party
    cd third_party/
    ./install_third_party
    cd ..

    #create folder for logs
    mkdir -p ~/drone_logs

    #build node source
    cd ~/catkin_ws
    catkin build
    source devel/setup.bash

    echo \n***** PLEASE RESTART FOR CHANGES TO TAKE EFFECT *****\n
}

install | tee install.log