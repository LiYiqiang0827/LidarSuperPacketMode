#!/bin/bash

set -e

file_name=""
folder_name=""
current_path=$(pwd)

function check_log() {
    local log_file=$1
    if grep -q "ERROR" "$log_file"; then
        echo "ERROR found in the log file. $log_file"
        grep "ERROR" "$log_file"
        exit 1
    else
        echo "log check success"
    fi
}

function get_pcd_rwg_test() {
    rm -rf $current_path/get_pcd.txt
    ./get_pcd --inno-pc-filename rwg.inno_pc --frame-number -1 --use-xyz-point 2 --output-filename rwg.pcd > $current_path/get_pcd.txt 2>&1
    md5sum_rwg=$(md5sum rwg.pcd | awk '{print $1}')
    if [ "$md5sum_rwg" == "03a4bb86e46e624cad7baab8745c31c8" ]; then
        echo "rwg pcd md5sum check success"
    else
        echo "rwg pcd md5sum check fail, md5sum is $md5sum_rwg"
        exit 1
    fi
    check_log "$current_path/get_pcd.txt"
    rm -rf rwg.pcd
}

function get_pcd_fk_test() {
    rm -rf $current_path/get_pcd.txt
    ./get_pcd --inno-pc-filename fk.inno_pc --frame-number -1 --use-xyz-point 2 --output-filename fk.pcd > $current_path/get_pcd.txt 2>&1
    md5sum_rwg=$(md5sum fk.pcd | awk '{print $1}')
    if [ "$md5sum_rwg" == "093f4ebd4febc6a190e50cc3a96bc710" ]; then
        echo "fk pcd md5sum check success"
    else
        echo "fk pcd md5sum check fail, md5sum is $md5sum_rwg"
        exit 1
    fi
    check_log "$current_path/get_pcd.txt"
    rm -rf fk.pcd
}

function get_pcd_relt_test() {
    rm -rf $current_path/get_pcd.txt
    ./get_pcd --inno-pc-filename relt.inno_pc --frame-number -1 --use-xyz-point 2 --output-filename relt.pcd > $current_path/get_pcd.txt 2>&1
    md5sum_rwg=$(md5sum relt.pcd | awk '{print $1}')
    if [ "$md5sum_rwg" == "8bd08b9ff7206e55167e679f6f1c4f4f" ]; then
        echo "relt pcd md5sum check success"
    else
        echo "relt pcd md5sum check fail, md5sum is $md5sum_rwg"
        exit 1
    fi
    check_log "$current_path/get_pcd.txt"
    rm -rf relt.pcd
}

function get_pcd_live_test() {
    rm -rf $current_path/get_pcd.txt
    rm -rf test.inno_pc
    ./get_pcd --lidar-ip 127.0.0.1 --lidar-udp-port 8010 --lidar-port 8010 --frame-number 2 --output-filename test.inno_pc > $current_path/get_pcd.txt 2>&1
    file_path="test.inno_pc"
    file_size=$(stat -c%s "$file_path")

    if [ $file_size -gt 500000 ]; then
        echo "live pcd record success"
    else
        echo "live pcd record fail"
        exit 1
    fi
    check_log "$current_path/get_pcd.txt"

    rm -rf test.inno_pc
}


function start_fk() {
    local time=$1
    echo "start falcon test"
    (timeout ${time}s ./inno_pc_client --file fk.inno_pc --speed 10000 --rewind -1 --tcp-port 8010 --udp-ip 127.0.0.1 --udp-port-status 8010 --udp-port-message 8010) &
}

function start_rwg() {
    local time=$1
    echo "start rwg test"
    (timeout ${time}s ./inno_pc_client --file rwg.inno_pc --speed 10000 --rewind -1 --tcp-port 8010 --udp-ip 127.0.0.1 --udp-port-status 8010 --udp-port-message 8010) &
}

function start_relt() {
    local time=$1
    echo "start relt live test"
    (timeout ${time}s ./inno_pc_client --file relt.inno_pc --speed 10000 --rewind -1 --tcp-port 8010 --udp-ip 127.0.0.1 --udp-port-status 8010 --udp-port-message 8010) &
}

# delete log txt
rm -rf *.txt


get_pcd_rwg_test
echo ""
get_pcd_fk_test
echo ""
get_pcd_relt_test
echo ""
sleep 5

start_fk "5"
get_pcd_live_test
sleep 5

echo ""

start_rwg "5"
get_pcd_live_test
sleep 5

echo ""

start_relt "5"
get_pcd_live_test
sleep 5

rm -rf *.log

cd $current_path
