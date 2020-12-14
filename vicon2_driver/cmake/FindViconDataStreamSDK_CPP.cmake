include(GNUInstallDirs)

find_library(
    LIBVICONDATASTREAM_SDK_LIBRARY
    NAMES ViconDataStreamSDK_CPP)

find_path(LIBVICONDATASTREAM_SDK_INCLUDE_DIR
  NAMES DataStreamClient.h
  PATHS /usr/local/include/ViconDataStreamSDK
  )

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(ViconDataStreamSDK_CPP DEFAULT_MSG
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