#!/bin/bash
# Run: $ curl -fsSL http://bit.ly/OpenCV-3_2 | sudo [optirun] bash
RESET='\033[0m'
COLOR='\033[1;35m'
function msg {
  echo -e "${COLOR}$(date): $1${RESET}"
}

# Script
msg "Installation Started"
INSTALL_PATH=$HOME/.opencv
DOWNLOAD_PATH=/tmp

msg "Updating System"
sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y dist-upgrade
sudo apt-get -y autoremove
sudo apt-get -y autoclean

msg "Installing build tools"
sudo apt-get install -y build-essential cmake git

msg "Installing GUI"
sudo apt-get install -y qt5-default libvtk6-dev

msg "Installing media I/O componenets"
sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev

msg "Installing video I/O components"
sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev

msg "Installing Linear Algebra and Parallelism libs"
sudo apt-get install -y libtbb-dev libeigen3-dev

msg "Installing Python"
sudo apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy

msg "Installing JDK"
sudo apt-get install -y ant default-jdk

msg "Installing Docs"
sudo apt-get install -y doxygen

msg "Installing LAPACKE libs"
sudo apt-get install -y liblapacke-dev checkinstall

msg "All deps installed. Continuing with installation"

# Getting to temp
cd $DOWNLOAD_PATH

# Downloading
msg "Downloading OpenCV"
git clone https://github.com/opencv/opencv.git
msg "Downloading OpenCV_Contrib Package"
git clone https://github.com/opencv/opencv_contrib.git

cd opencv;
mkdir -p build;
cd build

# Configuring make
msg "Configuring OpenCV"
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D BUILD_PYTHON_SUPPORT=ON \
      -D WITH_XINE=ON \
      -D WITH_OPENGL=ON \
      -D WITH_TBB=ON \
      -D BUILD_EXAMPLES=ON \
      -D BUILD_NEW_PYTHON_SUPPORT=ON \
      -D WITH_V4L=ON \
      -D BUILD_OPENCV_NONFREE=ON \
      -D OPENCV_EXTRA_MODULES_PATH=$DOWNLOAD_PATH/opencv_contrib/modules \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_PATH \
$DOWNLOAD_PATH/opencv

# Making
msg "Building with $(($(nproc)+1)) threads"
make -j $(($(nproc)+1))

# Installing
msg "Installing OpenCV-3.2.0$"
make install

# Finished
msg "Installation finished for OpenCV-3.2.0"
msg "Please Add $INSTALL_PATH/lib/pythonX.X/dist-packages/ to your PYTHONPATH"
