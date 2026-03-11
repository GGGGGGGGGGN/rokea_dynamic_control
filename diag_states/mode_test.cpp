#include <iostream>
#include <unistd.h>
#include <functional>
#include "robot.h"
#include "rci_data/robot_datas.h"

using namespace xmate;

// 辅助函数：读取并打印当前底层真实模式
void print_state(xmate::Robot& robot, const std::string& mode_name) {
    try {
        auto s = robot.receiveRobotState();
        std::cout << "   [验证 " << mode_name << "] 底层真实反馈 -> "
                  << "robot_mode=" << static_cast<int>(s.robot_mode)
                  << ", controller=" << static_cast<int>(s.controller_mode)
                  << ", motion_gen=" << static_cast<int>(s.motion_generator_mode)
                  << std::endl;
    } catch (...) {
        std::cout << "   [验证 " << mode_name << "] ❌ 读取 UDP 状态失败！" << std::endl;
    }
}

int main() {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(1);

    robot.automaticErrorRecovery();
    sleep(1);
    try { robot.stopMove(); } catch (...) {}
    sleep(1);
    
    robot.setMotorPower(1);
    sleep(2);

    std::cout << "===============================================" << std::endl;
    std::cout << "🔄 机器人底层模式切换测试" << std::endl;
    std::cout << "===============================================" << std::endl;

    // ==========================================
    // 测试 1: 纯位置模式
    // ==========================================
    std::cout << "\n-> 测试 1: 尝试开启 [位置模式] (kJointPosition + kJointPosition)..." << std::endl;
    try {
        robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition, 
                        RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
        sleep(1);
        print_state(robot, "位置模式");
        robot.stopMove(); // 💥 必须先停下来，才能切下一个！
        sleep(1);
    } catch (const std::exception& e) { std::cout << "❌ 被拒绝: " << e.what() << std::endl; }

    // ==========================================
    // 测试 2: 纯力矩模式
    // ==========================================
    std::cout << "\n-> 测试 2: 尝试开启 [力矩模式] (kTorque + kIdle)..." << std::endl;
    try {
        robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque, 
                        RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);
        sleep(1);
        print_state(robot, "力矩模式");
        robot.stopMove();
        sleep(1);
    } catch (const std::exception& e) { std::cout << "❌ 被拒绝: " << e.what() << std::endl; }

    // ==========================================
    // 测试 3: 官方内置阻抗模式
    // ==========================================
    std::cout << "\n-> 测试 3: 尝试开启 [阻抗模式] (kJointImpedance + kJointPosition)..." << std::endl;
    try {
        // 先配置刚度，否则大概率报错
        robot.setJointImpedance({{1000, 1000, 1000, 1000, 100, 100, 100}});
        robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointImpedance, 
                        RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
        sleep(1);
        print_state(robot, "阻抗模式");
        robot.stopMove();
        sleep(1);
    } catch (const std::exception& e) { std::cout << "❌ 被拒绝: " << e.what() << std::endl; }

    std::cout << "\n===============================================" << std::endl;
    std::cout << "-> 测试完毕！正在安全下电..." << std::endl;
    robot.setMotorPower(0);
    return 0;
}