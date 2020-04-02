#!/usr/bin/env bash

sudo apt -y install libboost-atomic-dev libboost-system-dev

# readonly APR_VERSION="1.7.0"
# cd /tmp
# wget https://ftp.tsukuba.wide.ad.jp/software/apache//apr/apr-${APR_VERSION}.tar.gz
# tar -zxvf apr-${APR_VERSION}.tar.gz
# cd apr-${APR_VERSION}
# ./configure --prefix=/usr/local/apr --disable-lfs
# make && sudo make install
# rm -rf /tmp/apr-${APR_VERSION}.tar.gz

cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
bash ./third_party/apr/apr_build.sh

cd build
# PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/apr/lib/pkgconfig/ && export PKG_CONFIG_PATH
cmake ..
make
sudo make install
rm -rf /tmp/Livox-SDK
