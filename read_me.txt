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
V1.1   matlab发送串口，pixhawk接受串口。刚开始正常，后来出现乱码

V1.2    收发不乱码   
	simulink 发送：
		将发送的数据转换成int16 。如 32767的十六进制位 7F FF。发送时，先发 FF， 再发7F。
	PX4接收：
		每次读8位，以无符号uint8 类型接收。再将两个uint8组合，放到int16类型变量中。此时得到了真实值。
		
				
