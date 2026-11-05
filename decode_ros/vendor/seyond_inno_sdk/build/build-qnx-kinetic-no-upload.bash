#!/bin/bash 
set -x
set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: " $0 " build-tag"
    echo "Example: " $0 " release-1.4.0-rc7"
    exit
fi

script_name=$0
script_full_path=$(dirname "$0")
echo $script_full_path
GENERAL_BUILD_SCRIPT=${script_full_path}/bitbucket-pipelines-build.bash

git checkout $1

QNX_INSTALL_PATH=~/qnx700
QNX_ENV_SCRIPT=qnxsdp-env.bash
cp -f ${script_full_path}/../${QNX_ENV_SCRIPT} ${QNX_INSTALL_PATH}
pushd .
cd ${QNX_INSTALL_PATH}
source ${QNX_ENV_SCRIPT}
popd

BITBUCKET_TAG=$1-qnx
MANUAL_CI=Y BITBUCKET_TAG=$BITBUCKET_TAG FORCE_BUILD_SDK=Y ${GENERAL_BUILD_SCRIPT} kinetic public output
MANUAL_CI=Y BITBUCKET_TAG=$BITBUCKET_TAG FORCE_BUILD_SDK=Y ${GENERAL_BUILD_SCRIPT} kinetic internal output

ls -l ${script_full_path}/../output/*${BITBUCKET_TAG}*
