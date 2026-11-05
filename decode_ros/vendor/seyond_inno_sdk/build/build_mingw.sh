#!/bin/bash

echo "====== build mingw inno_clientsdk begin... "
cd ../
if [[ ! -d build_mingw ]]; then
    mkdir build_mingw
fi

export ROOT_PATH="/d/win_build_package"

if [[ -n "$1" ]] ;then
export ROOT_PATH=$1
echo "the word you input is $ROOT_PATH"
fi

export MINGW_TOOLCHAIN_PREFIX=$ROOT_PATH"/msys64/mingw64/bin/"
export PATH=$PATH:$MINGW_TOOLCHAIN_PREFIX:$ROOT_PATH"/msys64/usr/bin"
echo $PATH

if [[ ! -d $MINGW_TOOLCHAIN_PREFIX ]]; then
    echo "/mingw64/bin/ not found!"
    exit
fi

export CC=${MINGW_TOOLCHAIN_PREFIX}gcc
export CXX=${MINGW_TOOLCHAIN_PREFIX}g++
export STRIP=${MINGW_TOOLCHAIN_PREFIX}strip
export AR=${MINGW_TOOLCHAIN_PREFIX}ar
export OBJCOPY=${MINGW_TOOLCHAIN_PREFIX}objcopy
export RANLIB=${MINGW_TOOLCHAIN_PREFIX}ranlib
export READELF=${MINGW_TOOLCHAIN_PREFIX}readelf
export make=${MINGW_TOOLCHAIN_PREFIX}mingw32-make.exe
export ARCH_TAG=-mingw64
export MINGW64=1

cd build_mingw
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ..
${make} -j6
${make} install
cd -
if [[ ! -f lib/libinnolidarsdkclient.a ]]; then
    echo "====== build inno_clientsdk failed!!!"
    exit
fi
echo "====== build inno_clientsdk success ^_^"
cd build
