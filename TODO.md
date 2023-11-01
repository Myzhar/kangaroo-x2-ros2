# TODO list

* Replace Arduino's "Stream" class with a simple serial port reader. See [here](https://github.com/Myzhar/ldrobot-lidar-ros2/blob/main/ldlidar_component/driver/src/cmd_interface_linux.cpp)
* Replace all the buffer with `std::vector` or `std::array`
* Replace `boolean` with standard `bool`
* Replace all `byte` with `uint8_t`
* Add tests. See [here](https://github.com/westonrobot/ugv_sdk/tree/main/test)
* Move the library to its own repository and add it as submodule to Kangaroo ROS 2 repository