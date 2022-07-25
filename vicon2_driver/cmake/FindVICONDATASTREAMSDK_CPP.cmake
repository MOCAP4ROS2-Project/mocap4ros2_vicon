# Copyright 2020 National Institute of Advanced Industrial Science
# and Technology, Japan
# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Francisco Martín <fmrico@gmail.com>

include(GNUInstallDirs)

find_library(
    LIBVICONDATASTREAM_SDK_LIBRARY
    NAMES ViconDataStreamSDK_CPP)

find_path(
  LIBVICONDATASTREAM_SDK_INCLUDE_DIR
  NAMES DataStreamClient.h
  PATHS /usr/local/include/ViconDataStreamSDK)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(VICONDATASTREAMSDK_CPP DEFAULT_MSG
                                  LIBVICONDATASTREAM_SDK_LIBRARY
                                  LIBVICONDATASTREAM_SDK_INCLUDE_DIR)

mark_as_advanced(LIBVICONDATASTREAM_SDK_LIBRARY LIBVICONDATASTREAM_SDK_INCLUDE_DIR)

if(LIBVICONDATASTREAM_SDK_FOUND AND NOT TARGET ViconDataStreamSDK_CPP::ViconDataStreamSDK_CPP)
  add_library(ViconDataStreamSDK_CPP::ViconDataStreamSDK_CPP SHARED IMPORTED)
  set_target_properties(
    ViconDataStreamSDK_CPP::ViconDataStreamSDK_CPP
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${LIBVICONDATASTREAM_SDK_INCLUDE_DIR}"
      IMPORTED_LOCATION ${LIBVICONDATASTREAM_SDK_LIBRARY})
endif()