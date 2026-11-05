#!/bin/bash 
set -x
set -e

RESULT=result

pushd ${RESULT}
ls -lrt
for i in *.tgz; do echo ${i} && tar tzvf ${i} && echo; done
popd
