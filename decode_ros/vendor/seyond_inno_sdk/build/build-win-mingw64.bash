#!/bin/bash
set -x
set -e

export MINGW_TOOLCHAIN_PREFIX="/mingw64/bin/"
export CC=${MINGW_TOOLCHAIN_PREFIX}gcc
export CXX=${MINGW_TOOLCHAIN_PREFIX}g++
export STRIP=${MINGW_TOOLCHAIN_PREFIX}strip
export AR=${MINGW_TOOLCHAIN_PREFIX}ar
export OBJCOPY=${MINGW_TOOLCHAIN_PREFIX}objcopy
export RANLIB=${MINGW_TOOLCHAIN_PREFIX}ranlib
export READELF=${MINGW_TOOLCHAIN_PREFIX}readelf
export LINKFLAGS="-pthread -ldl -lwsock32 -lws2_32 -Wl,-Bstatic -static"
export DYNA_LINKFLAGS="-pthread -ldl -lwsock32 -lws2_32 "
export OTHER_CFLAGS="-mno-ms-bitfields -w"
export OTHER_LIB_BUILD_CFLAG="-lwsock32"
export ARCH_TAG=-mingw64
export MINGW64=1

export BOOST_DIR="${MINGW_TOOLCHAIN_PREFIX}.."
export OPENSSL_DIR="${MINGW_TOOLCHAIN_PREFIX}.."
export BOOST_INC="-I${BOOST_DIR}/include"
export BOOST_LIB="-L${BOOST_DIR}/lib -lboost_system-mt"
export OPENSSL_INC="-I${OPENSSL_DIR}/include"
export OPENSSL_LIB="-L${OPENSSL_DIR}/lib -lcrypto -lssl -lcrypt32"

cd ../
mkdir -p result
make $1 -j 10 2>&1 | tee ./result/output.txt
mv *.tgz ./result
exit
