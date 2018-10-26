git clone https://github.com/inivation/libcaer libcaer
cd libcaer

sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev -y

mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=/usr -DENABLE_OPENCV=1 ..
make
sudo make install
