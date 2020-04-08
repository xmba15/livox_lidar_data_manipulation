#!/usr/bin/env bash

sudo -l env "PATH=$PATH"
sudo apt -y install pkg-config

readonly LIVOX_SDK_VERSION="v2.1.0"

# apr is dependency of livox sdk and can be installed separately with the following scripts
# readonly APR_VERSION="1.7.0"
# cd /tmp
# wget https://ftp.tsukuba.wide.ad.jp/software/apache//apr/apr-${APR_VERSION}.tar.gz
# tar -zxvf apr-${APR_VERSION}.tar.gz
# cd apr-${APR_VERSION}
# ./configure --prefix=/usr/local/apr --disable-lfs
# make && sudo make install
# rm -rf /tmp/apr-${APR_VERSION}.tar.gz

cd /tmp
git clone -b ${LIVOX_SDK_VERSION} https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK

# install apr with script provided by livox-sdk itself
sudo bash ./third_party/apr/apr_build.sh

cd build
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/apr/lib/pkgconfig/ && export PKG_CONFIG_PATH
cmake ..
make
sudo make install
sudo rm -rf /tmp/Livox-SDK
