#include <iostream>
#include <unistd.h>
#include <functional> // 💥 FIX 1: 必须在引入 robot.h 之前加上这个！
#include "robot.h"
#include "ini.h"
#include "rci_data/robot_datas.h"

using namespace xmate;

int main() {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(2);

    std::cout << "===========================================" << std::endl;
    std::cout << "🔍 机械臂底层状态 X 光扫描报告" << std::endl;
    std::cout << "===========================================" << std::endl;

    // 1. 检查网络 UDP
    RCI::robot::RobotState state;
    try {
        state = robot.receiveRobotState();
        std::cout << "[网络] UDP 数据包接收正常!" << std::endl;
    } catch (...) {
        std::cout << "[网络] ❌ UDP 接收失败！请检查 HMI 上的 PC IP 设置！" << std::endl;
        return -1;
    }

    // 2. 检查急停状态 (1:无急停, 2:ESTOP, 3:GSTOP)
    // 💥 FIX 2: 使用 static_cast 强制转换枚举类
    int safety = static_cast<int>(robot.getSafetyStopState());
    std::cout << "[安全] 急停状态代码: " << safety;
    if (safety == 1) std::cout << " (正常)" << std::endl;
    else if (safety == 2) std::cout << " ❌ (物理急停按钮被按下 ESTOP)" << std::endl;
    else if (safety == 3) std::cout << " ❌ (全局急停 GSTOP)" << std::endl;
    else std::cout << " ⚠️ (其他未知急停码)" << std::endl;

    // 3. 检查上电状态
    robot.setMotorPower(1);
    sleep(2); // 等待抱闸
    int power = static_cast<int>(robot.getMotorState());
    std::cout << "[使能] 电机上电状态代码: " << power;
    if (power == 1) std::cout << " (正常，已上电)" << std::endl;
    else std::cout << " ❌ (上电失败，抱闸未松开)" << std::endl;

    // 4. 检查 20 个底层错误位
    std::cout << "[错误] 底层 Error 标志位扫描:" << std::endl;
    bool has_error = false;
    for (int i = 0; i < 20; ++i) {
        if (state.errors[i]) {
            std::cout << "       ❌ 发现严重错误！错误代码位 (Error bit): " << i << std::endl;
            has_error = true;
        }
    }
    if (!has_error) {
        std::cout << "       ✅ 无任何错误标志位。" << std::endl;
    }

    std::cout << "===========================================" << std::endl;
    if (safety != 1 || power != 1 || has_error) {
        std::cout << "🚨 结论：机械臂当前状态异常，StartMove 必定会被拒绝！" << std::endl;
        std::cout << "👉 请根据上面的 ❌ 提示去 HMI 示教器上排查问题。" << std::endl;
    } else {
        std::cout << "✅ 结论：底层状态完美，不应该被拒绝。如果再报错，说明是参数配置问题！" << std::endl;
    }

    return 0;
}