# Vicon DataStream SDK 1.13.0 (source)

Source of the Vicon DataStream SDK 1.13.0+167154h for Linux (released Feb 2026),
as distributed by Vicon Motion Systems Ltd under the MIT license (see LICENSE and
the header of every file).

Only the modules needed by the C++ client are kept (StreamCommon and the
DataStream libraries); makefiles, tests and binaries are not included. The code
is unmodified. It depends on standalone Asio (system package `libasio-dev`,
rosdep key `asio`), not on Boost.

To update: download the SDK from https://www.vicon.com/software/datastream-sdk/,
extract `ViconDataStreamSDKSourceLinux64-source.7z` and replace these
directories with the same names under `Vicon/CrossMarket`.
