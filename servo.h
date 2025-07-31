#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cmath>
#include <atomic>
#include <csignal>
#include <vector>
#include <algorithm>
#include "SCServo.h"

#define LOG_INFO(msg) std::cerr << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl

class Servo {
public:
    Servo(int id, const std::string& port, float Kp = 1.0f, float Ki = 0.0f, float Kd = 0.01f)
        : ServoID(id), serial_port(port), speed_pid(Kp, Ki, Kd)
    {

        activeServoIDs.push_back(ServoID);
        feedback.id = ServoID;
        speed_max = 1000;
        acceleration = 50;

        if (!initialized) {
            LOG_INFO("初始化串口...");
            if (!sm_st.begin(1000000, serial_port.c_str())) {
                LOG_ERROR("串口初始化失败");
                throw std::runtime_error("串口初始化失败");
            }
            
            std::signal(SIGINT, signalHandler);
            initialized = true;
        }

        LOG_INFO("舵机初始化完成。");
        sm_st.WheelMode(ServoID);  // 设置恒速模式
    }

~Servo() {
stopFlag=true;
    auto it = std::remove(activeServoIDs.begin(), activeServoIDs.end(), ServoID);
    activeServoIDs.erase(it, activeServoIDs.end());
    LOG_INFO("舵机对象销毁。");
}


    void setDefaultAngle(int angle) { default_angle = angle; }
    void setSpeedMax(int speed) { speed_max = speed; }
    void setAcceleration(int acc) { acceleration = acc; }

    void PID_setAngle_control(int target_angle) {
        ServoFeedback feedback = get_feedback(ServoID);
        update_servo_control_speed_pos_loop(target_angle, feedback.pos);
    }

    void normally_setAngle_control(int target_angle) {
        sm_st.WritePosEx(ServoID, target_angle, speed_max, acceleration);
    }

    void stop_servo(int id) {
        sm_st.WriteSpe(id, 0, acceleration);
    }

    static void terminateAllServos() {
        for (int id : activeServoIDs) {
            sm_st.WriteSpe(id, 0, 50);
        }
        stopFlag = true;
    }

    static bool isStopped() {
        return stopFlag;
    }
    
        // 2. 增加单独函数关闭串口，程序退出时调用
    static void closeSerial() {
    if (initialized) {
        sm_st.end();
        initialized = false;
        LOG_INFO("串口关闭完成。");
       }
    }

private:
    int ServoID;
    std::string serial_port;
    int default_angle = 512;
    int speed_max;
    int acceleration;

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

    struct ServoFeedback {
        int id = 1;
        int pos = -1;
        int speed = -1;
        int current = -1;
        int load = -1;
        int move = -1;
        int voltage = -1;
        int temper = -1;
        bool success = false;
    } feedback;

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

    static inline bool initialized = false;
    static inline std::atomic<bool> stopFlag = false;
    static inline std::vector<int> activeServoIDs;
    static inline SMS_STS sm_st;

static void signalHandler(int signum) {
    const char msg[] = "\n[INFO] 捕捉到 Ctrl+C，准备退出...\n";
    write(STDOUT_FILENO, msg, sizeof(msg) - 1);
    stopFlag = true;
}
    
    

};

#endif // SERVO_CONTROLLER_H

