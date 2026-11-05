#!/usr/bin/python

import argparse
import os
import subprocess
import sys


def get_map_v(src, map_prefix, no_original_name=False, need_create=False):
    abs_path = os.path.abspath(src)
    if no_original_name:
        mapped_path = map_prefix
    else:
        mapped_path = os.path.join(map_prefix, os.path.basename(os.path.normpath(abs_path)))
    map_vol = ' -v {}:{} '.format(abs_path, mapped_path)
    if not os.path.exists(abs_path):
        if need_create:
            os.makedirs(abs_path)
        else:
            print("{} doesn't exist".format(abs_path))
            exit(1)
    return mapped_path, map_vol

def main(args):
    map_vol = ""

    _, mapv = get_map_v(args.build_output, "/root/output", no_original_name=True, need_create=True)
    map_vol += mapv

    src_top = os.path.join(argparse._sys.argv[0], "../..")
    _, mapv = get_map_v(src_top, "/root/src_top", no_original_name=True, need_create=True)
    map_vol += mapv
    ADDITIONAL_ENV = 'export FORCE_BUILD_SDK=Y && '
    BOOST_ENV = 'export BOOST_INC=-I/opt/boost1.78/include/ && export BOOST_LIB=-L/opt/boost1.78/lib/ && export CPLUS_INCLUDE_PATH=/opt/boost1.78/include/:$CPLUS_INCLUDE_PATH && '
    if args.arch == 'x86':
        if args.ros_version == 'kinetic':
            docker_image = "ivusw/ros-driver-build:ubuntu1604-kinetic-jsk-ceres"
        elif args.ros_version == 'noetic':
            docker_image = "ivusw/ros-driver-build:ubuntu2004-noetic-jsk-ceres-boost178"
            map_vol += " -v /etc/passwd:/etc/passwd:ro"
            map_vol += " -v /etc/group:/etc/group:ro"
        else:
            docker_image = "ivusw/ros-driver-build:ubuntu1804-melodic-jsk-ceres-boost178"
            # boost is installed in /usr/local
        cmd_template = ADDITIONAL_ENV + BOOST_ENV + "./build/bitbucket-pipelines-build.bash {} {} /root/output && "
        cmd_build = cmd_template.format(args.ros_version, "public")
        cmd_build += cmd_template.format(args.ros_version, "internal")
    elif args.arch == 'mingw64':
        docker_image = "ivusw/ros-driver-build:ubuntu1804-melodic-jsk-ceres-mingw64"
        cmd_template = ADDITIONAL_ENV + "export OUTPUT_ENV=/root/output && ./build/bitbucket-pipelines-build-mingw64.bash {} {} && "
        cmd_build = cmd_template.format(args.ros_version, "public")
    elif args.arch == 'arm':
        if args.ros_version == 'kinetic':
            docker_image = "ivusw/ros-driver-build-cross-compile-arm64:ubuntu1604-kineticjsk"
            ADDITIONAL_ENV = ADDITIONAL_ENV + "export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages && "
        elif args.ros_version == 'melodic':
            docker_image = "ivusw/ros-driver-build-cross-compile-arm64:ubuntu1804-melodicjsk-boost178"
        else:
            print("{} ROS build for ARM not yet supported".format(args.ros_version))
            exit(2)

        build_arm = "./build/bitbucket-pipelines-build-arm.bash {} public".format(args.ros_version)
        build_arm2 = "./build/bitbucket-pipelines-build-arm.bash {} internal".format(args.ros_version)
        cmd_build = ADDITIONAL_ENV + BOOST_ENV + "export OUTPUT_ENV=/root/output && {} && {} && ".format(build_arm, build_arm2)
    elif args.arch == 'qnx':
        if args.ros_version == 'kinetic':
            docker_image = "ivusw/q-driver-build:ubuntu1604"
        else:
            assert False, "qnx build only support kinetic"
        cmd_build = ADDITIONAL_ENV + "export OUTPUT_ENV=/root/output && ./build/bitbucket-pipelines-build-qnx.bash && "
    else:
        assert False, "arch={} not supported".format(args.arch)

    # docker_cmd = 'docker pull {} && docker'.format(docker_image)
    docker_cmd = 'docker'.format(docker_image)
    cmd = "cd /root/src_top; ls -l ..;"
    if args.arch != 'qnx':
        cmd = "source /opt/ros/{}/setup.bash; ".format(args.ros_version) + cmd

    if args.ros_version == 'noetic':
        safe_arg = "--add safe.directory /root/src_top"
        for safe_dir in [
                "", "/apps/tools/parse/3rdparty/pybind11", "/thirdparty/gtest/gtest"
        ]:
            cmd += " git config --global {}{};".format(safe_arg, safe_dir)
    cmd += cmd_build
    cmd += "find /root/output -maxdepth 1 -cmin -10"

    docker_run_cmd = 'time {} run -it --rm '.format(docker_cmd) +\
                     '{map_vol} '+\
                     '{docker_image} ' +\
                     'bash -c "{cmd}"'
    docker_run_cmd = docker_run_cmd.format(map_vol=map_vol, docker_image=docker_image, cmd=cmd)
    print("exec command: {}".format(docker_run_cmd))
    return_code = subprocess.call(docker_run_cmd, shell=True)
    return return_code

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros-version",
                        type=str,
                        default="melodic",
                        choices=["kinetic", "melodic", "noetic"],
                        help="ros version, default is melodic")
    parser.add_argument("--build-output", type=str, default="build_output",
                        help="output dir, default is build_output")
    parser.add_argument("--arch", type=str, default="x86",
                        choices=["x86", "arm", "qnx", "mingw64"],
                        help="arch default is x86")
    args = parser.parse_args()

    main(args)
