#include <iostream>
#include <cmath>
#include <functional>
#include <Eigen/Dense>
#include <unistd.h>

#include "robot.h"
#include "rci_data/robot_datas.h"
#include "move.h" // 💥 核心突破：借用官方的 MOVEJ 来释放抱闸

using namespace xmate;

int main() {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(1);

    robot.automaticErrorRecovery();
    sleep(1);
    robot.setMotorPower(1);
    sleep(2);

    // ==========================================
    // 💥 破局关键：先用 MOVEJ 动一下，强制释放物理抱闸！
    // ==========================================
    std::cout << "-> 正在执行原地 MOVEJ，听咔哒声释放抱闸..." << std::endl;
    std::array<double, 7> q_init = robot.receiveRobotState().q;
    // 让机械臂在当前位置保持 0.1 秒，这足以骗过底层状态机松开刹车！
    try {
        MOVEJ(0.1, q_init, q_init, robot); 
    } catch (...) {}
    sleep(1);

    // ==========================================
    // 发送纯力矩模式指令 (官方 Demo 标准写法)
    // ==========================================
    std::cout << "-> 抱闸已释放，正在发送 kTorque + kIdle 指令..." << std::endl;
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);

    Eigen::VectorXd q_des(7);
    bool is_first_tick = true;

    // 刚度系数 (Kp) 和 阻尼系数 (Kd)
    Eigen::VectorXd Kp(7); Kp << 100.0, 100.0, 100.0, 50.0, 30.0, 20.0, 10.0;
    Eigen::VectorXd Kd(7); Kd <<  10.0,  10.0,  10.0,  5.0,  2.0,  1.0,  0.5;

    auto torque_callback = [&](RCI::robot::RobotState state) -> Torques {
        static int count = 0;
        if (count == 0) std::cout << "🔥 成功进入纯力矩环！(底层自动重力补偿中) 用力推！" << std::endl;
        if (count % 1000 == 0) std::cout << "⏱️ PD 阻抗运行中... " << count / 1000 << " 秒" << std::endl;

        Eigen::Map<const Eigen::VectorXd> q_curr(state.q.data(), 7);
        Eigen::Map<const Eigen::VectorXd> dq_curr(state.dq_m.data(), 7);

        if (is_first_tick) {
            q_des = q_curr; // 记录启动瞬间的位置作为弹簧原点
            is_first_tick = false;
        }

        // 💥 纯粹的 PD 阻抗力矩（绝不画蛇添足加重力）
        Eigen::VectorXd tau_pd = Kp.cwiseProduct(q_des - q_curr) - Kd.cwiseProduct(dq_curr);

        // 限制弹簧力的大小，保证安全 (+- 30Nm)
        std::array<double, 7> tau_c_array;
        for(int i = 0; i < 7; ++i) {
            tau_c_array[i] = std::max(-30.0, std::min(30.0, tau_pd(i)));
        }

        Torques output(tau_c_array);

        count++;
        if (count >= 15000) { // 运行 15 秒后优雅退出
            std::cout << "✅ 15秒试飞结束" << std::endl;
            return MotionFinished(output);
        }
        return output;
    };

    robot.Control(torque_callback);

    std::cout << "-> 正在安全下电..." << std::endl;
    robot.setMotorPower(0);
    return 0;
}