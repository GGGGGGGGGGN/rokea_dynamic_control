#include <iostream>
#include <unistd.h>
#include <functional>
#include "robot.h"
#include "ini.h"
#include "rci_data/robot_datas.h"

using namespace xmate;

int main() {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(2);

    robot.automaticErrorRecovery();
    sleep(1);
    try { robot.stopMove(); } catch (...) {}
    sleep(1);

    robot.setMotorPower(1);
    sleep(2); // 等待抱闸松开

    // ==========================================
    // 💥 补丁 1：强制设置有效负载，唤醒动力学引擎！
    // 参数: 质量(kg), 质心坐标(m), 惯量张量
    // ==========================================
    std::cout << "-> 正在配置动力学负载参数 (setLoad)..." << std::endl;
    robot.setLoad(1.0, {0.0, 0.0, 0.05}, {0.01, 0.01, 0.01});
    sleep(1);

    // ==========================================
    // 💥 补丁 2：使用 kIdle 待命模式，防止状态机冲突死锁！
    // ==========================================
    std::cout << "-> 正在下发 kTorque + kIdle 启动指令 (startMove)..." << std::endl;
    // 注意这里第二个参数改成了 kIdle！！！
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque, 
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);
    
    std::cout << "✅ 成功冲破 startMove 阻塞！" << std::endl;

    // ==========================================
    // 挂载一个空力矩回调，让看门狗保持开心
    // ==========================================
    int count = 0;
    auto torque_callback = [&](RCI::robot::RobotState state) -> Torques {
        if (count == 0) std::cout << "🔥 成功进入 1kHz UDP 实时力矩循环！" << std::endl;
        count++;
        
        // 下发 0 附加力矩，让底层保持它自己的重力补偿
        std::array<double, 7> tau_d = {0,0,0,0,0,0,0};
        Torques output(tau_d);

        if (count > 5000) return MotionFinished(output); // 跑 5 秒后优雅结束
        return output;
    };

    robot.Control(torque_callback);

    std::cout << "✅ 运行结束，正在安全下电..." << std::endl;
    robot.setMotorPower(0);
    return 0;
}