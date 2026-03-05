#include <iostream>
#include <array>
#include <string>
#include <unistd.h> // 提供 sleep 函数
#include <functional> // 解决 std::function 报错

#include "ini.h" // 引入 ini 解析头文件
#include "robot.h"
#include "rci_data/robot_datas.h"

using namespace xmate;

// ==========================================
// 【导师秘籍 3：C++ 模板打印函数】
// 专门用来优雅地打印各种不同长度和类型的 std::array
// ==========================================
template <typename T, std::size_t N>
void printArray(const std::string& name, const std::array<T, N>& arr) {
    std::cout << name << ": [ ";
    for (std::size_t i = 0; i < N; ++i) {
        std::cout << arr[i] << " ";
    }
    std::cout << "]" << std::endl;
}

int main() {
    // 默认 IP 和 端口
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    // 从 xmate.ini 配置文件自动读取实际的 IP 和端口
    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    std::cout << "正在连接机械臂 IP: " << ipaddr << " 端口: " << port << " ..." << std::endl;
    
    // 实例化机器人对象
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(2);

    // 强行清错
    std::cout << "尝试清除历史遗留错误报警..." << std::endl;
    robot.automaticErrorRecovery();
    sleep(1); 

    int res = robot.getMotorState();
    std::cout << "连接成功！初始上电状态：" << res << std::endl;

    // 给机械臂上电
    robot.setMotorPower(1);
    sleep(2);
    
    std::cout << "机械臂已处于上电保持状态，开始读取底层全息反馈数据...\n" << std::endl;

    // 循环读取 10 次状态
    for (int i = 0; i < 10; ++i) {
        RCI::robot::RobotState state = robot.receiveRobotState();

        std::cout << "\n========== 第 " << i + 1 << " 次读取 (Message ID: " << state.message_id << ") ==========" << std::endl;
        
        // 1. 关节运动学状态 (7维)
        std::cout << "--- 关节运动学数据 ---" << std::endl;
        printArray("关节角度 q_m [rad]", state.q);
        printArray("指令关节角度 q_c [rad]", state.q_c);
        printArray("关节速度 dq_m [rad/s]", state.dq_m);
        printArray("指令关节速度 dq_c [rad/s]", state.dq_c);
        printArray("指令关节加速度 ddq_c [rad/s^2]", state.ddq_c);

        // 2. 笛卡尔空间状态 (16维齐次变换矩阵 & 6维速度加速度)
        std::cout << "--- 笛卡尔空间数据 (末端法兰) ---" << std::endl;
        printArray("实际末端位姿矩阵 toolTobase_pos_m", state.toolTobase_pos_m);
        printArray("指令末端位姿矩阵 toolTobase_pos_c", state.toolTobase_pos_c);
        printArray("指令末端速度 toolTobase_vel_c", state.toolTobase_vel_c);
        printArray("指令末端加速度 toolTobase_acc_c", state.toolTobase_acc_c);

        // 3. 冗余臂角状态 (7轴机械臂特有)
        std::cout << "--- 冗余臂角数据 ---" << std::endl;
        std::cout << "实际臂角 psi_m: " << state.psi_m << std::endl;
        std::cout << "指令臂角 psi_c: " << state.psi_c << std::endl;
        std::cout << "指令臂角速度 psi_vel_c: " << state.psi_vel_c << std::endl;
        std::cout << "指令臂角加速度 psi_acc_c: " << state.psi_acc_c << std::endl;

        // 4. 动力学与力矩状态 (极其重要)
        std::cout << "--- 动力学与力矩数据 ---" << std::endl;
        printArray("关节实际力矩 tau_m [Nm]", state.tau_m);
        printArray("关节滤波力矩 tau_m_filtered [Nm]", state.tau_m_filtered);
        printArray("指令关节力矩 tau_c [Nm]", state.tau_c);
        printArray("指令力矩微分 tau_vel_c", state.tau_vel_c);
        printArray("基坐标系外部力 tau_ext_in_base", state.tau_ext_in_base);
        printArray("力控坐标系外部力 tau_ext_in_stiff", state.tau_ext_in_stiff);

        // 5. 电机底层状态
        std::cout << "--- 电机底层数据 ---" << std::endl;
        printArray("电机位置 theta_m", state.theta_m);
        printArray("电机速度 theta_vel_m", state.theta_vel_m);
        printArray("电机转矩 motor_tau", state.motor_tau);
        printArray("电机滤波转矩 motor_tau_filtered", state.motor_tau_filtered);

        // 6. 系统运行状态
        std::cout << "--- 系统运行状态 ---" << std::endl;
        printArray("硬件错误位 errors (1为报错)", state.errors);
        std::cout << "指令接收成功率: " << state.control_command_success_rate * 100 << "%" << std::endl;
        
        // 枚举类型强制转换为 int 打印
        std::cout << "运动发生模式 (MotionGeneratorMode): " << static_cast<int>(state.motion_generator_mode) << std::endl;
        std::cout << "底层控制模式 (ControllerMode): " << static_cast<int>(state.controller_mode) << std::endl;
        std::cout << "机器人整体状态 (RobotMode): " << static_cast<int>(state.robot_mode) << std::endl;

        std::cout << "=========================================================\n" << std::endl;

        // 暂停 1 秒再读下一次
        sleep(1); 
    }

    // 优雅下电
    std::cout << "正在下电并优雅断开连接..." << std::endl;
    robot.setMotorPower(0);
    sleep(1); 

    std::cout << "读取完毕，程序安全退出。" << std::endl;
    return 0;
}