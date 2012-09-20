#common commands for building c++ executables and libraries
rosbuild_add_library(MIP_SDK byteswap_utilities.cpp mip.cpp mip_sdk_3dm.cpp mip_sdk_ahrs.cpp mip_sdk_base.cpp mip_sdk_gps.cpp mip_sdk_interface.cpp mip_sdk_nav.cpp mip_sdk_system.cpp mip_sdk_user_functions.cpp ring_buffer.cpp)

rosbuild_add_executable(GX3_35_Test GX3_35_Test.cpp)

target_link_libraries(GX3_35_Test MIP_SDK)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
