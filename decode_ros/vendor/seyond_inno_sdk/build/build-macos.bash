#!/bin/bash
set -x
set -e

CURRENT_DIR=$(cd $(dirname $0); pwd)
PROJECT_DIR=$(cd $(dirname $CURRENT_DIR); pwd)

##! macos build need install brew(https://brew.sh/) and xcode-select(https://mac.install.guide/commandlinetools/4.html) first
##!   1. xcode-select --install
##!   2. /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
##! brew install gnu-tar boost openssl rsync eigen
##!   3. brew install boost openssl eigen gnu-tar rsync
##! make sure that BOOST and OPENSSL path is right in bitbucket-pipelines-build-arm-macos.bash
##! run this script like:
##!   4. ./build-macos-clang.bash

if [ "$#" -gt 2 ]; then
  echo "Usage: " $0 " <local || pipeline> <internal || public>"
  exit
fi

ROSRELEASE=noetic
export INT_OR_PRI=public # internal or # public
if [ "$#" -ge 2 ]; then
  export INT_OR_PRI=$2 # internal or # public
fi

script_name=$0
script_full_path=$(dirname "$0")
echo $script_full_path
GENERAL_BUILD_SCRIPT=${script_full_path}/bitbucket-pipelines-build.bash

##! need check start
# export BOOST_DIR="/opt/homebrew/Cellar/boost/1.79.0_1"
# export OPENSSL_DIR="/opt/homebrew/Cellar/openssl@3/3.0.5"
export EIGEN_DIR="/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3"
export PYTHON_DIR="/Library/Developer/CommandLineTools/Library/Frameworks/Python3.framework/Versions/3.9/Headers"
# export GPERF_DIR=""

export ARM_TOOLCHAIN_PREFIX="/usr/bin/"
# export CMAKE="/opt/homebrew/bin/cmake"
export PATH=$PATH:/opt/homebrew/bin
export CC=${ARM_TOOLCHAIN_PREFIX}gcc
export CXX=${ARM_TOOLCHAIN_PREFIX}g++
export STRIP=${ARM_TOOLCHAIN_PREFIX}strip
export AR=${ARM_TOOLCHAIN_PREFIX}ar
export OBJCOPY=cp
export RANLIB=${ARM_TOOLCHAIN_PREFIX}ranlib
export READELF=${ARM_TOOLCHAIN_PREFIX}readelf

export
##! need check end

# export BOOST_INC="-I${BOOST_DIR}/include"
# export BOOST_LIB="${BOOST_DIR}/lib/libboost_system.a"
# export OPENSSL_INC="-I${OPENSSL_DIR}/include"
# export OPENSSL_LIB="${OPENSSL_DIR}/lib/libcrypto.a"

# if [[ -n ${GPERF_DIR} ]]; then
#   export GPERF_INC="-D GPERF -I${GPERF_DIR}/include"
#   export GPERF_LIB="${GPERF_DIR}/lib/libtcmalloc_and_profiler.a"
# fi

export NO_ROS=Y
export NO_DEB=Y

BITBUCKET_TAG=${BITBUCKET_TAG}
export ARCH_TAG=-macos
${CURRENT_DIR}/version_for_mac.bash

make -j16
export TARBALL_PUBLIC=YES
export TARBALL_INTERNAL=NO
export ARCH_TAG=-macos
${CURRENT_DIR}/tarball.bash
mkdir result
mv *.tgz ./result
