export CCPREFIX=/usr/bin/arm-unknown-linux-gnueabi-
export MODULES_TEMP=/tmp/modules
ARCH=arm CROSS_COMPILE=${CCPREFIX} make
ARCH=arm CROSS_COMPILE=${CCPREFIX} INSTALL_MOD_PATH=${MODULES_TEMP} make modules_install
