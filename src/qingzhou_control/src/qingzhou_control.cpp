#include "qingzhou_control.h"

// 构造函数，初始化
Actuator::Actuator()
{
    m_baudrate = 115200;
    m_serialport = "/dev/ttyUSB0";
    linearSpeed = 0;    //线速度
    angularSpeed = 0;   //角速度
    batteryVoltage = 0; //电池电压
    ticksPerMeter = 0;  //一米脉冲数
    ticksPer2PI = 0;    //每圈脉冲数
    encoderLeft = 0;    //左编码器
    encoderRight = 0;   //有编码器
    delta_time_ = 0;
    calibrate_lineSpeed = 0;
    x = 0.0;
    y = 0.0;
    th = 0.0;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    memset(&moveBaseControl, 0, sizeof(SmartcarControl));
    moveBaseControl.TargetAngle = 60;

    private_nh.param("mcubaudrate", m_baudrate, m_baudrate);                           //波特率
    private_nh.param("mcuserialport", m_serialport, std::string("/dev/ttyUSB0"));      //定义传输的串口
    private_nh.param("calibrate_lineSpeed", calibrate_lineSpeed, calibrate_lineSpeed); //标定线速度
    private_nh.param("ticksPerMeter", ticksPerMeter, ticksPerMeter);                   //一米脉冲数
    private_nh.param("ticksPer2PI", ticksPer2PI, ticksPer2PI);                         //每圈脉冲数

    try
    { //异常处理
        std::cout << "[dzactuator-->]"
                  << "Serial initialize start!" << std::endl;
        ser.setPort(m_serialport.c_str());
        ser.setBaudrate(m_baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(30);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        std::cout << "[dzactuator-->]"
                  << "Unable to open port!" << std::endl;
    }
    if (ser.isOpen())
    {
        std::cout << "[dzactuator-->]"
                  << "Serial initialize successfully!" << std::endl; //如果串口打开，打印串口初始化成功
    }
    else
    {
        std::cout << "[dzactuator-->]"
                  << "Serial port failed!" << std::endl;
    }

    sub_move_base = nh.subscribe("cmd_vel", 1, &Actuator::callback_move_base, this);
    pub_imu = nh.advertise<sensor_msgs::Imu>("uncalibrated", 5);
    pub_mag = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 5);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 5);
    pub_battery = nh.advertise<std_msgs::Float32>("battery", 10);
    // sub_movebase_angle = handle.subscribe("move_base/currentAngle", 1, &actuator::callback_movebase_angle, this); //订阅move_base/currentAngle话题上的消息。
}

//析构函数
Actuator::~Actuator()
{
}

//定义move_base回调函数
void Actuator::callback_move_base(const geometry_msgs::Twist::ConstPtr &msg) //对应cmd_vel话题，对应geometry_msgs/Twist消息
{
    memset(&moveBaseControl, 0, sizeof(SmartcarControl)); //清零movebase数据存储区

    float v = msg->linear.x;  //move_base算得的线速度
    float w = msg->angular.z; //move_base算得的角速度

    moveBaseControl.TargetSpeed = v * 32 / 0.43;                    //计算目标线速度**********************
    moveBaseControl.TargetAngle = round(atan(w * CARL / v) * 57.3); //计算目标角度**************************************************
    moveBaseControl.TargetAngle += 60;

    printf("%.2f,%.2f,%d,%d\n", msg->linear.x, msg->angular.z,
           abs(moveBaseControl.TargetSpeed), abs(moveBaseControl.TargetAngle));

    if (moveBaseControl.TargetAngle < 0)
    {
        moveBaseControl.TargetAngle = 0; //角度小于0时 等于0
    }
    if (moveBaseControl.TargetAngle > 120)
    {
        moveBaseControl.TargetAngle = 120; //角度最大120
    }

    //linear speed
    if (moveBaseControl.TargetSpeed > 0)
        moveBaseControl.TargetShiftPosition = 0x02; //前进
    else if (moveBaseControl.TargetSpeed < 0)
        moveBaseControl.TargetShiftPosition = 0x01; //后退
    else if (moveBaseControl.TargetSpeed == 0)
        moveBaseControl.TargetShiftPosition = 0x00; //停止

    //期望转向角度符号
    if (moveBaseControl.TargetAngle > 0)
        moveBaseControl.TargetAngleDir = 0x20; //右转（左右跟舵机安装方式有关）
    else if (moveBaseControl.TargetAngle < 0)
        moveBaseControl.TargetAngleDir = 0x10; //左转
    else if (moveBaseControl.TargetAngle == 0)
        moveBaseControl.TargetAngleDir = 0x00; //直行
}

void Actuator::run()
{
    int run_rate = 50;
    ros::Rate rate(run_rate);

    while (ros::ok())
    {
        ros::spinOnce();

        current_time_ = ros::Time::now();
        delta_time_ = (current_time_ - last_time_).toSec(); //转换成秒
        last_time_ = ros::Time::now();

        recvCarInfoKernel(); //接收stm32发来的数据

        processBattery();
        processImu();
        processOdom();

        sendCarInfoKernel();

        rate.sleep();
    }
}

//发送小车数据到下位机
void Actuator::sendCarInfoKernel()
{
    unsigned char buf[23] = {0};
    buf[0] = 0xa5;
    buf[1] = 0x5a;
    buf[2] = 0x06;

    buf[3] = (int)moveBaseControl.TargetAngleDir;      //targetangleDirection 0-->go straight,0x10-->turn left,0x20-->turn right
    buf[4] = (int)abs(moveBaseControl.TargetAngle);    //targetangle 180-->0xb4期望转向角度值
    buf[5] = (int)abs(moveBaseControl.TargetSpeed);    //targetSpeed期望线速度
    buf[6] = (int)moveBaseControl.TargetModeSelect;    //0-->person control,1-->auto control期望模式 人工/自动
    buf[7] = (int)moveBaseControl.TargetShiftPosition; //targetshiftposition  0-->P stop;1-->R;2-->D.期望档位 停止/倒车/前进

    buf[8] = 0;
    unsigned char sum = 0;
    for (int i = 2; i < 19; ++i)
        sum += buf[i];
    buf[9] = (unsigned char)(sum);
    size_t writesize = ser.write(buf, 10);
}

//接收下位机发送来的数据
void Actuator::recvCarInfoKernel()
{
    std::string recvstr;
    unsigned char tempdata, lenrecv;
    unsigned char count, last_data, last_last_data, last_last_last_data;
    unsigned char str[100];
    bool recvflag = false;
    bool recvd_flag = false;
    memset(&str, 0, sizeof(str));
    ros::Time begin_time = ros::Time::now();
    double clustering_time = 0;

    while (1)
    {
        clustering_time = (ros::Time::now() - begin_time).toSec(); //计算时间差，转换成秒

        if (clustering_time > 1)
        {
            recvd_flag = false;
            break;
        }

        recvstr = ser.read(1);
        if ((int)recvstr.size() != 1)
            continue;

        tempdata = recvstr[0];
        if (last_last_last_data == 0xa5 && last_last_data == 0x5a)
        {
            lenrecv = last_data;
            recvflag = true;
            count = 0;
        }
        if (recvflag)
        {
            str[count] = tempdata;
            count++;
            if (count == lenrecv)
            {
                recvflag = false;
                recvd_flag = true;
                break;
            }
        }
        last_last_last_data = last_last_data;
        last_last_data = last_data;
        last_data = tempdata;
    }

    if (recvd_flag)
    {
        carInfo carTemp;
        memcpy(&carTemp.raw[3], str, 30);
        auto &parsed = carTemp.data;

        encoderLeft = parsed.re_Encoder_Left;
        encoderRight = parsed.re_Encoder_Right;
        batteryVoltage = parsed.Voltage;

        accelX = (float)parsed.accelX / 2048 * 9.8; //线加速度处理
        accelY = (float)parsed.accelY / 2048 * 9.8;
        accelZ = (float)parsed.accelZ / 2048 * 9.8;

        gyroX = (float)parsed.gyroX / 16.4 / 57.3; //角速度处理
        gyroY = (float)parsed.gyroY / 16.4 / 57.3;
        gyroZ = (float)parsed.gyroZ / 16.4 / 57.3;

        magX = (float)parsed.magX * 0.14; //磁力计处理
        magY = (float)parsed.magY * 0.14;
        magZ = (float)parsed.magZ * 0.14;

        if (encoderLeft > 220 || encoderLeft < -220)
            encoderLeft = 0; //判断编码器脉冲数是否在正确范围
        if (encoderRight > 220 || encoderRight < -220)
            encoderRight = 0;
        LeftticksPerMeter += encoderLeft;   //获得左轮总脉冲数
        rightticksPerMeter += encoderRight; //获得右轮总脉冲数
    }
}