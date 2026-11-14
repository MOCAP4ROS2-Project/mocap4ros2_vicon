^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap4r2_vicon_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* Rewritten on MocapSystemNode with the Vicon DataStream SDK 1.13 sources (MIT, no Boost; standalone Asio).
* Frames read in a dedicated thread (pre-fetch, one-frame buffer): no 100 Hz cap and the executor is never blocked.
* Each subject has only its own markers; subjects with several segments are also skeletons; frame number from the SDK; latency compensation; occluded segments flagged.
* Contributors: Francisco Martín Rico, Lorena Bajo Rebollo, Manuel Fernández Carmona, pariaspe, Jiangxuan Chen
