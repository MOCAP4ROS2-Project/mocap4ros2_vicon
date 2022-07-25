#!/bin/bash

# Script assumes we are working at mocap_vicon_driver source folder.

# Downloaded sdk
SDKFile="ViconDataStreamSDK_1.11.0_128037.zip"
SDK_OS="ViconDataStreamSDK_1.11.0.128037h*"
LinuxFile="ViconDataStreamSDK_1.11.0.128037h__Linux64.zip"
LinuxSourceFile="ViconDataStreamSDKSourceLinux64-source.7z"

# Check if sdk was already downloaded
if [  -d "./vicon_sdk" ] 
then
        echo "SDK already installed."
        exit 1
fi


# Download sources. Static link should point to version 1.11
wget https://app.box.com/shared/static/xpmdwfsc79jim2sh7uvyysxbih3m7sft.zip -O ${SDKFile}

# Uncompressing ( ${LinuxFile} uncompress to 'Linux64' folder)
7za x ${SDKFile}
7za x ${LinuxFile}
mkdir ./Linux64/sources
cd ./Linux64/sources
7za x ../${LinuxSourceFile} 
cd ../..

# Rearrange files
mv ./Linux64/sources/Vicon/CrossMarket ./vicon_sdk
cp ./Linux64/ViconDataStreamSDK_CPPTest.cpp ./src/ViconDataStreamSDK_CPPTest.cpp

# Tell SDK example where DataStreamClient header is now
sed -i '/\#include \"DataStreamClient.h\"/c\#include "ViconDataStreamSDK_CPP/DataStreamClient.h"' ./src/ViconDataStreamSDK_CPPTest.cpp

# Cleanup
rm -rf ./vicon_sdk/DataStream/*Test
rm -rf ./Linux64
rm -rf ./${SDK_OS}
rm -rf ./*.7z