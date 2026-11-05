#!/bin/bash

set -e
set -x

RM=${RM:-rm}
TAR=${TAR:-tar}
ARCH_TAG=${ARCH_TAG:-}
TARBALL_PUBLIC=${TARBALL_PUBLIC:-YES}
TARBALL_INTERNAL=${TARBALL_INTERNAL:-YES}

BASE_TARBALL_FILES="README.md Makefile_SDKdeploy CMakeLists.txt"
INTERNAL_TARBALL_FILES=""

# build
BASE_TARBALL_FILES="$BASE_TARBALL_FILES build/cpplint.py build/*.cmake build/build_unix.sh build/build_qnx.bash"

# docs
BASE_TARBALL_FILES="$BASE_TARBALL_FILES docs/check_net.md docs/demo.md docs/get_pcd.md docs/HOW_TO_USE_CLIENT_SDK.md"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES docs/innovusion_lua.md"

# src
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/CMakeLists.txt src/Makefile"

# src/sdk_common
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/sdk_common/*.h src/sdk_common/*.cpp"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/sdk_common/version_gen.mk src/sdk_common/version_gen.gen_cc"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/sdk_common/converter/*.h src/sdk_common/converter/*.cpp"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/sdk_common/Makefile src/sdk_common/CMakeLists.txt"

# src/sdk_client
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/sdk_client/*.h src/sdk_client/*.cpp"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/sdk_client/ring_id_converter/*.h src/sdk_client/ring_id_converter/*.cpp"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/sdk_client/Makefile src/sdk_client/CMakeLists.txt"

# src/utils
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/utils/*.h src/utils/*.cpp"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES src/utils/Makefile src/utils/CMakeLists.txt"

# apps
BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/Makefile"

# apps/example
BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/example/Makefile apps/example/CMakeLists.txt apps/example/*.cpp"

# apps/tools
BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/check_net/check_net.sh"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/get_pcd/Makefile apps/tools/get_pcd/CMakeLists.txt apps/tools/get_pcd/*.cpp"
# BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/http_command_test/*"
BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/innovusion_wireshark_plugin/*"
#BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/lidar_util/innovusion_lidar_util"

# BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/parse/parse_pcap apps/tools/parse/*.cpp apps/tools/parse/*.h apps/tools/parse/*.py"
# BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/parse/*.cpp apps/tools/parse/*.h apps/tools/parse/*.py"
# BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/parse/Makefile"


INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/lidar_util/*.h apps/tools/lidar_util/*.cpp"
INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/lidar_util/Makefile apps/tools/lidar_util/CMakeLists.txt"

INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/lidar_util/split_yaml_raw.bash"
INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/lidar_util/scripts/customer_mode_1/*"
INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/lidar_util/scripts/customer_mode_2/*"
INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/get_frame_rate/get_frame_rate.py"
INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/get_packet_rate/get_packet_loss_rate.py"
INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/latency_diagram/latency_diagram.py"
INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES docs/innovusion_lidar_util.md"

DYNA_LIB_TARBALL_FILES="lib/*.so* src/sdk_common/inno_lidar_api.h src/sdk_common/inno_lidar_other_api.h src/sdk_common/inno_lidar_packet.h SDK_VERSION"
DYNA_LIB_TARBALL_FILES="$DYNA_LIB_TARBALL_FILES src/sdk_common/inno_faults_common.h src/sdk_common/inno_faults_falcon.h src/sdk_common/inno_faults_robinw.h src/sdk_common/inno_faults_robinel.h src/sdk_common/inno_faults_hummingbird.h"
# lib and executive file
if [ "$ARCH_TAG" == "-mingw64" ] || [ "$ARCH_TAG" == "-win" ]
then
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/example/demo.exe apps/example/sphere2xyz.exe apps/tools/get_pcd/get_pcd.exe"
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/lidar_util/innovusion_lidar_util.exe"
	if [ "$ARCH_TAG" == "-win" ]
	then
		BASE_TARBALL_FILES="$BASE_TARBALL_FILES lib/innolidarsdkclient.lib lib/*.dll"
	else
		BASE_TARBALL_FILES="$BASE_TARBALL_FILES lib/*.a"
	fi
elif [ "$ARCH_TAG" == "-macos" ]
then
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/example/demo apps/tools/get_pcd/get_pcd"
	INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/lidar_util/innovusion_lidar_util"
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES lib/*.a lib/*.dylib"
else
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/tools/lidar_util/innovusion_lidar_util"
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES SDK_VERSION"
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES apps/example/demo apps/example/sphere2xyz apps/tools/get_pcd/get_pcd"
	INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/lidar_util/innovusion_lidar_util"
	BASE_TARBALL_FILES="$BASE_TARBALL_FILES lib/*.a lib/*.so* lib/*.so.*"
fi

# python sdk
if [ "$ARCH_TAG" == "-win" ] || [ "$ARCH_TAG" == "" ] || [ "$ARCH_TAG" == "-arm" ]
then
	INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/py_module/dist/*"
	INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/py_module/site/*"
	INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/py_module/example/*"
	INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/py_module/LICENSE"
	INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES apps/tools/py_module/README.MD"
fi

INTERNAL_TARBALL_FILES="$INTERNAL_TARBALL_FILES $BASE_TARBALL_FILES"

if [ $TARBALL_PUBLIC == "YES" ]
then
	$RM -rf inno_lidar_sdk_public.tgz
	if [ "$ARCH_TAG" == "-macos" ]
	then
		$TAR czvf inno_lidar_sdk_macos_public.tgz $BASE_TARBALL_FILES
	elif [ "$ARCH_TAG" == "-win" ]
	then
		BUILD_TAG=$(git describe --tags $(git rev-list --tags --max-count=1))
		$TAR czvf inno-lidar-sdk-${BUILD_TAG}-windows-public.tgz --transform="s/_SDKdeploy$//" $BASE_TARBALL_FILES
	elif [ "$ARCH_TAG" == "-qnx" ]
	then
		BUILD_TAG=$(git describe --tags $(git rev-list --tags --max-count=1))
		$TAR czvf inno-lidar-sdk-${BUILD_TAG}-qnx-public.tgz --transform="s/_SDKdeploy$//" $BASE_TARBALL_FILES
	else
		$TAR czvf inno_lidar_sdk_lib.tgz $DYNA_LIB_TARBALL_FILES
		$TAR czvf inno_lidar_sdk_public.tgz --transform="s/_SDKdeploy$//" $BASE_TARBALL_FILES
	fi
fi

if [ $TARBALL_INTERNAL == "YES" ]
then
	$RM -rf inno_lidar_sdk_internal.tgz
	if [ "$ARCH_TAG" == "-win" ]
	then
		BUILD_TAG=$(git describe --tags $(git rev-list --tags --max-count=1))
		$TAR czvf inno-lidar-sdk-${BUILD_TAG}-windows-internal.tgz --transform="s/_SDKdeploy$//" $INTERNAL_TARBALL_FILES
	else
		$TAR czvf inno_lidar_sdk_internal.tgz --transform="s/_SDKdeploy$//" $INTERNAL_TARBALL_FILES
	fi
fi
