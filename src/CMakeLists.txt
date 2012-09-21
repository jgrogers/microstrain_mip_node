add_subdirectory(MIP_SDK)
rosbuild_add_executable(driver driver.cpp)
rosbuild_add_executable(imu_node imu_node.cpp 3dmgx3.cpp)