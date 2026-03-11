#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <unistd.h>
#include <functional>

// 💥 引入 Pinocchio 与 Eigen
#include <Eigen/Dense>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "ini.h"
#include "robot.h"
#include "rci_data/robot_datas.h"
#include "move.h"

using namespace xmate;

struct RecordData {
    uint64_t msg_id;
    std::array<double, 7> q;
    std::array<double, 7> dq;
    std::array<double, 7> tau_m;   // 实际电机力矩
    std::array<double, 7> tau_cmd; // 我们下发的指令力矩
};

int main() {
    // ==========================================
    // 1. 初始化 Pinocchio 动力学模型
    // ==========================================
    std::string urdf_filename = "/home/chen/Desktop/rci/ROKAE_Control_Interface/assests/xmate_pro7.urdf";
    pinocchio::Model model;
    std::cout << "正在加载 URDF: " << urdf_filename << " ..." << std::endl;
    try {
        pinocchio::urdf::buildModel(urdf_filename, model);
    } catch (const std::exception& e) {
        std::cerr << "❌ 解析 URDF 失败！\n" << e.what() << std::endl;
        return -1;
    }
    pinocchio::Data data(model);
    std::cout << "✅ Pinocchio 动力学引擎加载完毕！" << std::endl;

    // ==========================================
    // 2. 初始化网络与机械臂连接
    // ==========================================
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;
    INIParser ini;
    if (ini.ReadINI("../../xmate.ini")) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    std::cout << "正在连接机械臂 IP: " << ipaddr << " 端口: " << port << " ..." << std::endl;
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(3);
    robot.automaticErrorRecovery();
    sleep(2);
    robot.setMotorPower(1);
    sleep(2);

    // ==========================================
    // 3. 配置控制器参数 (核心区)
    // ==========================================
    const int MAX_RECORD_TICKS = 10000; // 运行 10 秒
    std::vector<RecordData> data_log;
    data_log.reserve(MAX_RECORD_TICKS);

    // 状态标志位
    bool is_first_tick = true;
    Eigen::VectorXd q_des(7);
    Eigen::VectorXd dq_des = Eigen::VectorXd::Zero(7);

    // 💥 PD 弹簧参数 (刚度 Kp, 阻尼 Kd) 
    // 初始值给得比较柔和，保证绝对安全！后续可按需放大
    Eigen::VectorXd Kp(7); Kp << 60, 60, 60, 40, 15, 15, 10;
    Eigen::VectorXd Kd(7); Kd << 15, 15, 15, 10,  4,  4,  3;

    // 🛡️ 危险开关：是否将 Pinocchio 的重力叠加到下发力矩中？
    // 第一把测试强烈建议设为 false！先测试 PD 的回弹力！
    const bool ENABLE_PINOCCHIO_GRAVITY = false; 

    std::cout << "\n================ 🚀 试飞提示 ================" << std::endl;
    std::cout << "即将进入【PD 位置保持控制模式】！" << std::endl;
    std::cout << "当前 Pinocchio 前馈重力叠加状态: " << (ENABLE_PINOCCHIO_GRAVITY ? "【开启】" : "【关闭】") << std::endl;
    std::cout << "请用手轻轻推拽机械臂，感受 PD 控制器的弹簧刚度！" << std::endl;
    std::cout << "一手放在急停按钮上！3秒后开始..." << std::endl;
    sleep(3);

    int count = 0;
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque, 
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
                    
    // ==========================================
    // 4. 1kHz 实时动力学控制回调
    // ==========================================
    auto torque_callback = [&](RCI::robot::RobotState robot_state) -> Torques {
        // 映射数据为 Eigen 格式，方便矩阵计算
        Eigen::VectorXd q_curr = Eigen::Map<const Eigen::VectorXd>(robot_state.q.data(), 7);
        Eigen::VectorXd dq_curr = Eigen::Map<const Eigen::VectorXd>(robot_state.dq_m.data(), 7);

        // 记录初始位姿作为我们的目标位姿 (仅在第1个毫秒触发)
        if (is_first_tick) {
            q_des = q_curr;
            is_first_tick = false;
        }

        // --- 核心算力 ---
        // 1. 计算 Pinocchio 重力补偿 G(q)
        pinocchio::computeGeneralizedGravity(model, data, q_curr);
        Eigen::VectorXd tau_g = data.g;

        // 2. 计算 PD 控制力矩 (弹簧 + 阻尼)
        Eigen::VectorXd tau_pd = Kp.cwiseProduct(q_des - q_curr) + Kd.cwiseProduct(dq_des - dq_curr);

        // 3. 融合总指令
        Eigen::VectorXd tau_cmd = tau_pd;
        if (ENABLE_PINOCCHIO_GRAVITY) {
            tau_cmd += tau_g; 
        }

        // 🛡️ 4. 终极安全限幅 (极其重要，防止发散把人打伤)
        std::array<double, 7> tau_out;
        for(int i=0; i<7; ++i) {
            // 限制我们自己下发的力矩波动范围在 +/- 60Nm 之内
            double t = tau_cmd(i);
            t = std::max(-60.0, std::min(60.0, t)); 
            tau_out[i] = t;
        }

        // 记录数据
        if (count < MAX_RECORD_TICKS) {
            RecordData rd;
            rd.msg_id = robot_state.message_id;
            rd.q = robot_state.q;
            rd.dq = robot_state.dq_m;
            rd.tau_m = robot_state.tau_m;
            rd.tau_cmd = tau_out;
            data_log.push_back(rd);
            count++;
        }

        // 打包输出
        Torques output(tau_out);
        if (count >= MAX_RECORD_TICKS) {
            return MotionFinished(output);
        }
        return output;
    };

    // 开跑！
    robot.Control(torque_callback);

    std::cout << "\n10秒 PD 控制体验结束！机械臂已恢复锁定。" << std::endl;
    robot.setMotorPower(0);
    sleep(1);

    // ==========================================
    // 5. 保存数据，用于离线辨识和图表分析
    // ==========================================
    std::ofstream outfile("pd_gravity_data.csv");
    outfile << "msg_id,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,tau_m1,tau_m2,tau_m3,tau_m4,tau_m5,tau_m6,tau_m7,tau_cmd1,tau_cmd2,tau_cmd3,tau_cmd4,tau_cmd5,tau_cmd6,tau_cmd7\n";

    for (const auto& rd : data_log) {
        outfile << rd.msg_id << ",";
        for(int j=0; j<7; ++j) outfile << rd.q[j] << ",";
        for(int j=0; j<7; ++j) outfile << rd.dq[j] << ",";
        for(int j=0; j<7; ++j) outfile << rd.tau_m[j] << ",";
        for(int j=0; j<7; ++j) outfile << rd.tau_cmd[j] << (j==6 ? "\n" : ",");
    }
    outfile.close();

    std::cout << "数据已安全保存至: pd_gravity_data.csv" << std::endl;
    return 0;
}