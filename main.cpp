#include <iostream>
#include "servo.h"




int main(int argc,char **argv)//这个是主函数
{
    // 创建舵机实例
    Servo<1, 1952> servo("/dev/ttyUSB0", 1.0f, 0.1f, 0.01f);
    Servo<2, 100> servo2("/dev/ttyUSB0", 1.0f, 0.1f, 0.01f);
    
    //这几行其实可用可不用
    // 设置舵机参数
    //servo.setDefaultAngle(1952); // 设置默认角度
    //servo.setSpeedMax(1000);
    // 设置加速度
    //servo.setAcceleration(50);


    while(!Servo<1, 1952>::isStopped())
    {
            // 启动舵机控制
    servo.PID_setAngle_control(1952);
    
    servo2.PID_setAngle_control(100);
    }

    // 停止舵机
    servo.stop_servo(1);
    servo2.stop_servo(2);

    return 0;
}





