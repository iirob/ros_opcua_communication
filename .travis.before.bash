#!/bin/bash

git submodule update --init
export PYTHONPATH=$PWD/ros_opcua_impl_python_opcua/python-opcua:$PYTHONPATH
