
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <unistd.h>
#include <functional>

#include "ini.h"
#include "robot.h"
#include "rci_data/robot_datas.h"
#include "move.h" // 包含 MotionFinished 函数

using namespace xmate;

// 定义一个结构体，用于在内存中极速暂存每一毫秒的数据
struct RecordData {
    uint64_t msg_id;            // 控制器发来的心跳包编号
    std::array<double, 7> q;    // 关节角度
    std::array<double, 7> dq;   // 关节速度
    std::array<double, 7> tau_m;// 关节力矩
    double success_rate;        // 控制器端统计的通信成功率
};

int main() {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    std::cout << "正在连接机械臂 IP: " << ipaddr << " 端口: " << port << " ..." << std::endl;
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(2);

    robot.automaticErrorRecovery();
    sleep(1);

    robot.setMotorPower(1);
    sleep(2);

    // ==========================================
    // 导师秘籍 1：预分配内存，杜绝实时循环里的动态分配
    // ==========================================
    const int MAX_RECORD_TICKS = 10000; // 记录 10 秒钟 (10000次 * 1ms = 10s)
    std::vector<RecordData> data_log;
    data_log.reserve(MAX_RECORD_TICKS); // 一次性在内存里把 10000 个位置占好

    std::cout << "准备开始 1kHz 高频数据采集 (时长 10 秒)..." << std::endl;
    std::cout << "采集中请勿拔掉网线，你可以尝试轻轻触碰机械臂。" << std::endl;

    // ==========================================
    // 导师秘籍 2：设置安全挂挡模式
    // 我们只是采数据不想动，所以使用“关节阻抗模式”并保持在原地，这最安全
    // ==========================================
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointImpedance, 
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    std::array<double, 7> q_init;
    bool is_init = false;
    int count = 0;

    // ==========================================
    // 核心 1kHz 实时回调函数
    // ==========================================
    auto joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        if (!is_init) {
            q_init = robot_state.q; // 记住初始位置
            is_init = true;
        }

        // 极速拷贝当前毫秒的数据到内存 Vector 中
        if (count < MAX_RECORD_TICKS) {
            RecordData rd;
            rd.msg_id = robot_state.message_id;
            rd.q = robot_state.q;
            rd.dq = robot_state.dq_m;
            rd.tau_m = robot_state.tau_m;
            rd.success_rate = robot_state.control_command_success_rate;
            data_log.push_back(rd);
            count++;
        }

        // 生成控制指令：保持在原地不动
        // JointPositions output;
        // output.q_c = q_init; 

        JointPositions output(q_init);

        // 达到设定时间，优雅退出 1kHz 循环
        if (count >= MAX_RECORD_TICKS) {
            return MotionFinished(output);
        }
        return output;
    };

    // 正式把控制权交给底层，阻塞运行 10 秒
    robot.Control(joint_position_callback);

    std::cout << "实时采集结束！正在将数据保存到文件，并进行链路分析..." << std::endl;

    // 安全下电
    robot.setMotorPower(0);
    sleep(1);

    // ==========================================
    // 导师秘籍 3：退出高频循环后，再进行耗时的磁盘写入
    // ==========================================
    std::ofstream outfile("dynamics_data.csv");
    // 写入 CSV 表头
    outfile << "msg_id,success_rate,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,tau1,tau2,tau3,tau4,tau5,tau6,tau7\n";

    int drop_count = 0;
    for (size_t i = 0; i < data_log.size(); ++i) {
        auto& rd = data_log[i];
        
        // 统计丢包情况：如果这一次的心跳包编号比上一次的差值大于 1，说明中间有 UDP 丢包
        if (i > 0) {
            uint64_t id_diff = rd.msg_id - data_log[i-1].msg_id;
            if (id_diff > 1) {
                drop_count += (id_diff - 1);
            }
        }

        // 写入一行数据
        outfile << rd.msg_id << "," << rd.success_rate << ",";
        for(int j=0; j<7; ++j) outfile << rd.q[j] << ",";
        for(int j=0; j<7; ++j) outfile << rd.dq[j] << ",";
        for(int j=0; j<7; ++j) outfile << rd.tau_m[j] << (j==6 ? "\n" : ",");
    }
    outfile.close();

    std::cout << "\n================ 链路稳定性报告 ================" << std::endl;
    std::cout << "数据已安全保存至: build/dynamics_data.csv" << std::endl;
    std::cout << "总计接收数据帧数: " << data_log.size() << " 帧 (理论应为 " << MAX_RECORD_TICKS << ")" << std::endl;
    std::cout << "系统检测到丢包数: " << drop_count << " 帧" << std::endl;
    std::cout << "底层评估指令成功率: " << data_log.back().success_rate * 100 << " %" << std::endl;
    std::cout << "================================================" << std::endl;

    return 0;
}