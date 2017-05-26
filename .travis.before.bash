#!/bin/bash

 dir=$PWD
 parts=(${dir//\// })
 name=${parts[${#parts[@]} - 1]}

if [[ ${name} ! "ros_opcua_communication" ]]; then
 roscd ros_opcua_communication && cd ..
fi

git submodule update --init
export PYTHONPATH=`rospack find ros_opcua_impl_python_opcua`/python-opcua:$PYTHONPATH
