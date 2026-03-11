/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 轴空间阻抗运动（s规划）
 */

#include <cmath>
#include <functional>

#include <iostream>
#include "robot.h"
#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "duration.h"
#include "move.h"

#include <unistd.h>

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[])
{
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file))
    {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO,false );
    sleep(3);


    // ==========================================
    // 💥 加上我们的工业级“清道夫”补丁！
    // ==========================================
    std::cout << "-> 正在清除机器人底层历史错误..." << std::endl;
    robot.automaticErrorRecovery();
    sleep(1);
    
    std::cout << "-> 正在强行终止残留运动状态..." << std::endl;
    try {
        robot.stopMove();
    } catch (...) {} // 忽略本身没有运动时的报错
    sleep(1);
    // ==========================================

    std::cout << "-> 正在给电机上使能..." << std::endl;
    robot.setMotorPower(1);

    sleep(3);   
    
    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    std::cout << "-> 探针 A：正在读取当前姿态..." << std::endl;
    q_init = robot.receiveRobotState().q;
    std::cout << "-> 移动到初始位置..." << std::endl;
    MOVEJ(0.1,q_init,q_drag,robot);
    std::cout << "✅ MOVEJ 执行完毕！机械臂已到达初始位置。" << std::endl;
    sleep(3);

    robot.setJointImpedance({{1000, 1000, 1000, 1000, 100, 100, 100}});

    std::cout << "-> 检查一下设备状态..." << std::endl;
    auto s = robot.receiveRobotState();
    std::cout << "robot_mode=" << static_cast<int>(s.robot_mode)
            << ", controller_mode=" << static_cast<int>(s.controller_mode)
            << ", motion_generator_mode=" << static_cast<int>(s.motion_generator_mode)
            << ", cmd_rate=" << s.control_command_success_rate << std::endl;

    for (size_t i = 0; i < s.errors.size(); ++i) {
        if (s.errors[i]) {
            std::cout << "error bit: " << i << std::endl;
        }
    }

    std::cout << "-> 探针 D：正在下发阻抗模式 StartMove 指令..." << std::endl;
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointImpedance, 
        RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    std::cout << "✅ 阻抗模式开启成功！" << std::endl;

    std::array<double, 7> init_position;
    static bool init = true;
    double time = 0;

    JointControl joint_position_callback;
    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }

        double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

        JointPositions output = {{init_position[0] + delta_angle, init_position[1] + delta_angle,
                                            init_position[2] + delta_angle, init_position[3] - delta_angle,
                                            init_position[4] + delta_angle, init_position[5] - delta_angle,
                                            init_position[6] + delta_angle}}; 
        time += 0.001; 
        if(time>20){
            std::cout<<"运动结束："<<std::endl;
            return MotionFinished(output);
        }
        return output;        
    };

    robot.Control(joint_position_callback);
    return 0;
}
