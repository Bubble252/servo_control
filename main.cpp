#include <iostream>
#include "servo.h"




int main(int argc,char **argv)//这个是主函数
{
    // 初始化舵机对象，假设使用两个舵机
    // 这里的端口和ID需要根据实际情况修改
    // 例如 "/dev/ttyUSB0" 是Linux下的串口设备路径
    // 这里的PID参数可以根据实际需求调整
    // 例如 Kp=4.0f, Ki=0.0f, Kd=0.01f
    Servo servo1(1, "/dev/ttyUSB0", 4.0f, 0.0f, 0.01f);
    Servo servo2(2, "/dev/ttyUSB0", 4.0f, 0.0f, 0.01f);
    
    //这几行其实可用可不用
    // 设置舵机参数
    //servo1.setDefaultAngle(1952); // 设置默认角度
    //servo1.setSpeedMax(1000);
    // 设置加速度
    //servo1.setAcceleration(50);

    std::cout << "stopFlag initial: " << Servo::isStopped() << std::endl;
    while (!Servo::isStopped())// 如果stopFlag为false，则继续工作
    {
    
    LOG_INFO("开始工作...");
            // 启动舵机控制
    servo1.PID_setAngle_control(1952);// PID 控制舵机角度 目标角度是1952

    servo2.PID_setAngle_control(100);// PID 控制舵机角度 目标角度是100
    usleep(5000);  // 加一点延迟，避免疯狂打印
    }

    // 停止舵机
    servo1.stop_servo(1);
    servo2.stop_servo(2);// 停止舵机
    Servo::closeSerial();// 关闭串口

    return 0;
}






