#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cmath>
#include <atomic>
#include <csignal>
#include "SCServo.h"
#include <vector>
#include <algorithm> 

#define LOG_INFO(msg) std::cerr << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl


template<int ServoID, int DefaultAngle>
// 定义舵机控制类
class Servo {
public:


    // 构造函数，接受串口参数
    Servo(const std::string& serial_port,float Kp = 1.0f, float Ki = 0.0f, float Kd = 0.01f)
        : serial_port(serial_port.c_str()), speed_pid(Kp, Ki, Kd) {
        stopFlag = false;// 初始化停止标志
        activeServoIDs.push_back(ServoID); // 将当前实例的 ID 添加到静态容器中
        std::cerr << "Configuring Servo ID " << ServoID 
                  << " with PID: " << Kp << ", " << Ki << ", " << Kd << std::endl;
        feedback.id = ServoID; // 初始化反馈结构体的 ID
        speed_max = 1000; // 设置默认最大速度
        acceleration = 50; // 设置默认加速度



        init(); // 调用初始化函数
        //sm_st.WritePosEx(ServoID, DefaultAngle, speed_max, acceleration); // 设置默认角度
        std::cerr << "[INFO] 舵机初始化完成。" << std::endl;

    }//在此处接收了串口参数，并初始化 PID 控制器 PID控制器的参数来源于模板参数 
    //这个构造函数的调用方法是 Servo<1, 1.0f, 0.0f, 0.0f> servo("/dev/ttyUSB0");


    ~Servo() {
        stopFlag = true;
        auto it = std::remove(activeServoIDs.begin(), activeServoIDs.end(), static_cast<int>(ServoID));
        activeServoIDs.erase(it, activeServoIDs.end()); // 正确使用 erase 和 remove
        std::cerr << "[INFO] 停止舵机..." << std::endl;
        sm_st.WriteSpe(ServoID, 0, 50);
        sm_st.end();
        std::cerr << "[INFO] 已退出程序。" << std::endl;
    }

    void init() {// 初始化函数 会设置舵机的恒速模式，并注册 Ctrl+C 信号处理函数
        LOG_INFO("初始化舵机...");
        if (!sm_st.begin(1000000, serial_port)) {
            LOG_ERROR("串口初始化失败");
            throw std::runtime_error("串口初始化失败");
        }
        sm_st.WheelMode(ServoID);  // 设置恒速模式
        std::signal(SIGINT, signalHandler);  // 注册 Ctrl+C 信号处理
    }


    void setDefaultAngle(int angle) {// 设置默认角度
        default_angle = angle;
        std::cerr << "设置默认角度为: " << default_angle << std::endl;
    }

    void setSpeedMax(int speed) {// 设置最大速度
        speed_max = speed;
        std::cerr << "设置最大速度为: " << speed_max << std::endl;
    }

    void setAcceleration(int acc) {// 设置加速度
        acceleration = acc;
        std::cerr << "设置加速度为: " << acceleration << std::endl;
    }


    void PID_setAngle_control(int target_angle) {//使用 PID 控制方法
        std::cerr << "设置目标角度(PID): " << target_angle << std::endl;
        ServoFeedback feedback = get_feedback(ServoID);
        update_servo_control_speed_pos_loop(target_angle, feedback.pos);

    }

    void normally_setAngle_control(int target_angle) {//使用常规方法
        std::cerr << "设置目标角度(常规): " << target_angle << std::endl;
        sm_st.WritePosEx(ServoID, target_angle,speed_max, acceleration);
    }


    void stop_servo(int id) {// 停止指定 ID 的舵机
        std::cerr << "停止舵机 ID: " << id << std::endl;
        sm_st.WriteSpe(id, 0, acceleration);
    }

    void terminateAllServos() {// 停止所有舵机
        std::cerr << "停止所有舵机..." << std::endl;
        for (int id : activeServoIDs) {
            sm_st.WriteSpe(id, 0, acceleration);
        }
        stopFlag = true;
    }

    // 静态方法，用于检查是否停止
    static bool isStopped() {
        return stopFlag;
    }

private:

    static inline std::vector<int> activeServoIDs; // 静态容器，存储所有实例的 ID

    struct PID {//PID控制器结构体
        // PID 控制器参数
        float Kp, Ki, Kd;
        float integral = 0.0f;
        float prev_error = 0.0f;
        float output = 0.0f;
        float out_max = 0.0f;
        float integral_limit = 1000.0f;

        PID(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {}
    };


    PID speed_pid;  // PID 控制器实例

    const char* serial_port;// 串口参数

    int default_angle = DefaultAngle; // 默认角度

    int speed_max; // 最大速度

    int acceleration; // 加速度


    struct ServoFeedback {
        int id = 1;// 舵机 ID

        int pos = -1;
        int speed = -1;
        int current = -1;
        int load = -1;
        int move = -1;
        int voltage = -1;
        int temper = -1; // 温度


        bool success = false;// 是否成功获取反馈
    };

    ServoFeedback feedback;// 舵机反馈结构体实例

    ServoFeedback get_feedback(int id) {// 获取舵机反馈
        ServoFeedback fb;
        if (sm_st.FeedBack(id) != -1) {
            fb.pos = sm_st.ReadPos(id);
            fb.speed = sm_st.ReadSpeed(id);
            fb.current = sm_st.ReadCurrent(id);
            fb.load = sm_st.ReadLoad(id);
            fb.move = sm_st.ReadMove(id);
            fb.voltage = sm_st.ReadVoltage(id);
            std::cout << id << "," << fb.pos << "," << fb.speed << ","// 打印舵机反馈信息
                      << fb.load << "," << fb.voltage << ","
                      << fb.temper << "," << fb.move << ","
                      << fb.current << std::endl;
            fb.success = true;// 设置成功标志
        } else {
            std::cerr << "FeedBack error for ID " << id << std::endl;
            fb.success = false;
        }
        return fb;
    }

    float pid_calculate(PID &pid, float target, float current, float out_max) {// PID 计算函数
        float error = target - current;
        pid.integral += error;

        if (pid.integral > pid.integral_limit) pid.integral = pid.integral_limit;
        if (pid.integral < -pid.integral_limit) pid.integral = -pid.integral_limit;

        float derivative = error - pid.prev_error;
        pid.output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
        pid.prev_error = error;

        pid.out_max = out_max;
        if (pid.output > out_max) pid.output = out_max;
        if (pid.output < -out_max) pid.output = -out_max;

        if (std::abs(error) < 4.5f) {// 如果误差小于 4.5，重置 PID 控制器,并时使舵机停止
            pid.output = 0.0f;
            pid.integral = 0.0f;
        }

        std::cout << "----------------------\n";//打印 PID 调试信息
        std::cout << "Target    : " << target << "\n";
        std::cout << "Error     : " << error << "\n";
        std::cout << "P (Kp*e)  : " << pid.Kp * error << "\n";
        std::cout << "I (Ki*∑e) : " << pid.Ki * pid.integral << "\n";
        std::cout << "D (Kd*Δe) : " << pid.Kd * derivative << "\n";
        std::cout << "Output    : " << pid.output << "\n";

        return pid.output;
    }


    // 更新舵机控制
    // 这里使用 PID 控制器来计算目标速度，并发送给舵机
    // parameters:
    // - target_pos: 目标位置
    // - current_pos: 当前舵机位置
    void update_servo_control_speed_pos_loop(float target_pos, float current_pos) {// 更新舵机控制
        float target_speed = pid_calculate(speed_pid, target_pos, current_pos, speed_max);
        sm_st.WriteSpe(ServoID, int(target_speed), acceleration);
    }






    static inline std::atomic<bool> stopFlag;// 停止标志
    // 静态成员变量，用于存储 ServoController 的状态
    static SMS_STS sm_st;//这个是 SCServo 的状态管理类实例

    //这些变量是静态的，因为它们需要在所有 Servo 实例之间共享
    // 这样可以避免每个实例都创建自己的 SMS_STS 实例，从而节省内存和资源



    // 信号处理函数，用于捕捉 Ctrl+C 信号
    static void signalHandler(int signum) {// 信号处理函数
        std::cout << "\n[INFO] 捕捉到 Ctrl+C，准备退出...\n";
        stopFlag = true;
    }
};

// 静态成员变量定义
template<int ServoID, int DefaultAngle>
SMS_STS Servo<ServoID, DefaultAngle>::sm_st;

#endif // SERVO_CONTROLLER_H

