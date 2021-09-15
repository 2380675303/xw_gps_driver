# xw_gps_driver
 
适用星网宇达系列惯导的ros驱动，已适配gpfpd、gprmc、gtimu消息

# 依赖
[serial](https://github.com/wjwwood/serial)

# 扩展
要添加其他NMEA消息类型，
1. 继承NMEA模板类，并定义存储数据的结构体，与转发的消息类型，参考`include/gps_driver/gprmc.h`;
2. 实现decode与parse函数，参考`src/gprmc.cpp`;
3. 在`src/gps_driver_node.cpp`中调用`registerNmea()`注册类型。
