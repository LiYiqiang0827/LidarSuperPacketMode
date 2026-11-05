#!/bin/bash
# This script is sets environment variables requires to use this version of QNX Software Development Platform 7.0
# from the command line. To use the script, you have to "source" it into your shell,
# source qnxsdp-env.sh
# if source command is not available use "." (dot) command instead
#

QNX_BASE=/opt/qos222/
QNX_HOST=$QNX_BASE/host/linux/x86_64
QNX_TARGET=$QNX_BASE/target/qnx7
QNX_CONFIGURATION=$HOME/.qnx
MAKEFLAGS=-I$QNX_BASE/target/qnx7/usr/include
PATH=$QNX_HOST/usr/bin:$QNX_CONFIGURATION/bin:$QNX_BASE/jre/bin:$PATH

export QNX_BASE QNX_TARGET QNX_HOST QNX_CONFIGURATION MAKEFLAGS PATH
unset PYTHONPATH

export QNX_TOOLCHAIN_TRIPLE="aarch64-unknown-nto-qnx7.1.0"
export QNX_TOOLCHAIN_PREFIX="${QNX_HOST}/usr/bin/${QNX_TOOLCHAIN_TRIPLE}-"
export QNX_TOOLCHAIN_FILE=${QNX_BASE}/cmake/Toolchain-QNX.cmake

export CC=${QNX_TOOLCHAIN_PREFIX}gcc
export CXX=${QNX_TOOLCHAIN_PREFIX}g++
export STRIP=${QNX_TOOLCHAIN_PREFIX}strip
export AR=${QNX_TOOLCHAIN_PREFIX}ar
export OBJCOPY=${QNX_TOOLCHAIN_PREFIX}objcopy
export RANLIB=${QNX_TOOLCHAIN_PREFIX}ranlib
export READELF=${QNX_TOOLCHAIN_PREFIX}readelf

echo QNX_HOST=$QNX_HOST
echo QNX_TARGET=$QNX_TARGET
echo MAKEFLAGS=$MAKEFLAGS
echo CFLAFS=$CFLAGS
