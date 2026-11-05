#!/bin/bash 
set -x
set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: " $0 " <BUILD-TAG>"
    echo "Example: " $0 " release-1.4.0-rc7"
    exit
fi

script_name=$0
script_full_path=$(dirname "$0")
echo $script_full_path
GENERAL_BUILD_SCRIPT=${script_full_path}/bitbucket-pipelines-build.bash
S3_UPLOAD=${script_full_path}/s3_upload.py

git pull
git checkout $1

BITBUCKET_TAG=$1-arm
export NO_ROS_JSK=ON

MANUAL_CI=Y BITBUCKET_TAG=$BITBUCKET_TAG FORCE_BUILD_SDK=Y ${GENERAL_BUILD_SCRIPT} kinetic public output
MANUAL_CI=Y BITBUCKET_TAG=$BITBUCKET_TAG FORCE_BUILD_SDK=Y ${GENERAL_BUILD_SCRIPT} kinetic internal output

ls -l ${script_full_path}/../output/*${BITBUCKET_TAG}*
python ${S3_UPLOAD} --bucket iv-release --artifact output/*${BITBUCKET_TAG}-public.* --bucket_key release/TAG/${BITBUCKET_TAG}
python ${S3_UPLOAD} --bucket iv-bitbucket-pipeline --artifact output/*${BITBUCKET_TAG}-psymbol.* --bucket_key build/TAG/${BITBUCKET_TAG}
python ${S3_UPLOAD} --bucket iv-bitbucket-pipeline --artifact output/*${BITBUCKET_TAG}-internal.* --bucket_key build/TAG/${BITBUCKET_TAG}
