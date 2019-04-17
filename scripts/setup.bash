#!/bin/sh
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 [root of new workspace] [FULL path to the velmwheel rosinstall file]" >&2
  exit 1
fi
if ! [ -e "$1" ]; then
  echo "$1 not found" >&2
  exit 1
fi
if ! [ -d "$1" ]; then
  echo "$1 not a directory" >&2
  exit 1
fi

if ! [ -e "$2" ]; then
  echo "$2 not found" >&2
  exit 1
fi
if ! [ -f "$2" ]; then
  echo "$2 not a directory" >&2
  exit 1
fi

# Check for python-wstool package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' python-wstool|grep "install ok installed")
echo Checking for python-wstool: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No python-wstool. Setting up python-wstool."
  sudo apt-get --yes install python-wstool
fi

# Check for python-catkin-tools package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' python-catkin-tools|grep "install ok installed")
echo Checking for python-catkin-tools: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No python-catkin-tools. Setting up python-catkin-tools."
  sudo apt-get --yes install python-catkin-tools
fi

# Check for ruby-dev package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ruby-dev|grep "install ok installed")
echo Checking for ruby-dev: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ruby-dev. Setting up ruby-dev."
  sudo apt-get --yes install ruby-dev 
fi

# Check for libreadline-dev package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libreadline-dev|grep "install ok installed")
echo Checking for libreadline-dev: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No libreadline-dev. Setting up libreadline-dev."
  sudo apt-get --yes install libreadline-dev 
fi

# Check for ros-kinetic-controller-manager package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-kinetic-controller-manager|grep "install ok installed")
echo Checking for ros-kinetic-controller-manager: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-kinetic-controller-manager. Setting up ros-kinetic-controller-manager."
  sudo apt-get --yes install ros-kinetic-controller-manager 
fi

# Check for omniorb package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' omniorb|grep "install ok installed")
echo Checking for omniorb: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No omniorb. Setting up omniorb."
  sudo apt-get --yes install omniorb 
fi
# Check for libomniorb4-dev package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libomniorb4-dev|grep "install ok installed")
echo Checking for libomniorb4-dev: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No libomniorb4-dev. Setting up omniorb."
  sudo apt-get --yes install libomniorb4-dev 
fi
# Check for ros-kinetic-lms1xx package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-kinetic-lms1xx|grep "install ok installed")
echo Checking for ros-kinetic-lms1xx: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-kinetic-lms1xx. Setting up omniorb."
  sudo apt-get --yes install ros-kinetic-lms1xx
fi
# Check for ros-kinetic-rtt-rosparam package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-kinetic-rtt-rosparam|grep "install ok installed")
echo Checking for ros-kinetic-rtt-rosparam: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-kinetic-rtt-rosparam. Setting up omniorb."
  sudo apt-get --yes install ros-kinetic-rtt-rosparam
fi

cd $1
wstool init
wstool merge $2
wstool update

cd underlay_isolated 
catkin config --cmake-args -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_CORE_ONLY=ON -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON
catkin build

cd ../underlay
catkin config --extend ../underlay_isolated/devel/ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF
catkin build

cd ../robot
catkin config --extend ../underlay/devel/ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF
catkin build
