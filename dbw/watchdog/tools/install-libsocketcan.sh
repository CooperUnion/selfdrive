#!/bin/bash

git clone https://git.pengutronix.de/cgit/tools/libsocketcan
apt-get -q install -y libtool autoconf
cd libsocketcan
./autogen.sh
./configure
make install
cd ..
rm -rf libsocketcan
echo "/usr/local/lib" > /etc/ld.so.conf.d/libsocketcan.conf
