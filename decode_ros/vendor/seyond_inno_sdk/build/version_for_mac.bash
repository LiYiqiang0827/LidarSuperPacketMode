#!/bin/bash

set -e
set -x

RM=${RM:-rm}
TAR=${TAR:-tar}
ARCH_TAG=${ARCH_TAG:-}
TARBALL_PUBLIC=${TARBALL_PUBLIC:-YES}
TARBALL_INTERNAL=${TARBALL_INTERNAL:-YES}

REPO_TOP_DIR=`git rev-parse --show-toplevel` || true

SDK_VERSION=$(git describe --tags $(git rev-list --tags --max-count=1) | sed -E 's/[^0-9.]+//g')
if [[ $SDK_VERSION =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
echo ${SDK_VERSION}
major=$(echo "$SDK_VERSION" | sed -E 's/^([0-9]+)\.([0-9]+)\.([0-9]+)$/\1/')
minor=$(echo "$SDK_VERSION" | sed -E 's/^([0-9]+)\.([0-9]+)\.([0-9]+)$/\2/')
patch=$(echo "$SDK_VERSION" | sed -E 's/^([0-9]+)\.([0-9]+)\.([0-9]+)$/\3/')
sed -i "" "s/^#define INNO_SDK_V_MAJOR .*/#define INNO_SDK_V_MAJOR \"$major\"/" ${REPO_TOP_DIR}/src/sdk_common/inno_lidar_packet.h
sed -i "" "s/^#define INNO_SDK_V_MINOR .*/#define INNO_SDK_V_MINOR \"$minor\"/" ${REPO_TOP_DIR}/src/sdk_common/inno_lidar_packet.h
sed -i "" "s/^#define INNO_SDK_V_DOT .*/#define INNO_SDK_V_DOT \"$patch\"/" ${REPO_TOP_DIR}/src/sdk_common/inno_lidar_packet.h
else
    echo "tag name is not in the right format"
    exit 1
fi

echo ${REPO_TOP_DIR}
eval $(sed -n 's/^#define INNO_SDK_V_ *\([^ ]*\)  *\(.*\) *$/export INNO_SDK_V_\1=\2/p' ${REPO_TOP_DIR}/src/sdk_common/inno_lidar_packet.h)
DATE=`date +%Y%m%d%H%M%S`
SDK_VERSION=${INNO_SDK_V_MAJOR}.${INNO_SDK_V_MINOR}.${INNO_SDK_V_DOT}.${DATE}
echo ${SDK_VERSION}
echo ${SDK_VERSION} > ${REPO_TOP_DIR}/SDK_VERSION

BUILD_TAG=$(git describe --tags $(git rev-list --tags --max-count=1))
DEB_VERSION=`echo ${BUILD_TAG} | sed 's/[^0-9]*//'`

gen_version_gen_h () {
    ver=${DEB_VERSION}-$1

	COPYRIGHT="/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 */
"
	printf "${COPYRIGHT}\nconst char *innovusion_version = \"${ver}\";\n" > $2/apps/tools/lidar_util/version_gen.cpp
	printf "const char *innovusion_build_tag = \"${BUILD_TAG}\";\n" >> $2/apps/tools/lidar_util/version_gen.cpp
	printf "${COPYRIGHT}\nconst static char inno_api_version_g[] = \"${SDK_VERSION}\";\n" > $2/src/sdk_common/version_gen.gen_cc
	printf "const static char inno_api_build_tag_g[] = \"${BUILD_TAG}\";\n" >> $2/src/sdk_common/version_gen.gen_cc
	printf "DYNA_LIB_MAJ=${INNO_SDK_V_MAJOR}\nDYNA_LIB_MIN=${INNO_SDK_V_MINOR}\nDYNA_LIB_BUILD=${INNO_SDK_V_DOT}\n" > $2/src/sdk_common/version_gen.mk
	cp -f $2/src/sdk_common/version_gen.mk $2/src/utils/version_gen.mk
	cat $2/src/sdk_common/version_gen.gen_cc

}

gen_version_gen_h "public" $REPO_TOP_DIR
