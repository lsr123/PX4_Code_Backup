serial_send_receive是PX4固件 的一个APP 实现了串口的收发，需在PX4代码中使用。更改编译文件，将app写到固件中。
V1.0 
	功能：pixhawk飞控向simulink以固定频率发送数据，simulink以固定频率接收
	int16 占2个字节，如24567的十六进制为 F7 5F 大端
	发送3个int16的数字，24567 3 4 即 F7 5F 03 00 04 00
	代码中：
		线程以100Hz运行，每次运行发送3个数据
	simulink中：
		以变步长运行。
		serial_receive模块设置采样率为0.01/3, 即0.01s运行3次，采3个数据，Data type为int16
		simulation Pace 模块设置 1 sim sec per sec，采样率1/30
		Data type模块为int16
		serial configuration 配置时选择大小端
		
				
