# New_Legends_Project
宁工NewLegends战队工程2022赛季电控_张超

工程机器人分使用顶部MCU和底盘MCU两块C型开发板作为控制核心

此代码适用于底盘MCU

CAN_receive         CAN接收，读取电机和编码器返回的数据
card                出卡电机，控制复活卡的M2006
chassis             底盘任务，控制底盘
pid                 方便复制粘贴PID算法，没有实际用处
remote_control      遥控器接收，实质为串口接收，所有需要用到串口的代码都会写在这里
save                救援舵机控制
struct_typedef      数据类型，只是几个typedef，没有实际作用

控制方法

左拨杆      右拨杆      效果

中          下          控制出卡

下          中          控制救援

下          下          底盘断电

下          上          云台模式
