#include <iostream>
#include <vector>
#include <array>
#include <unistd.h>
#include <functional> 

#include "ini.h"
#include "robot.h"
#include "rci_data/robot_datas.h"
#include "model.h" 

using namespace xmate;

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << "🚀 启动纯数学动力学引擎 (无需连接真实机械臂)" << std::endl;
    std::cout << "============================================================" << std::endl;

    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    std::cout << "正在初始化 Robot 通信实例 (安全模式：不上电)..." << std::endl;
    // 实例化 Robot，不上电 (false)
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
    sleep(3);

    // ==========================================
    // 💥 终极秒杀：传入 robot 的指针和机器人型号！
    // ==========================================
    xmate::XmateModel model(&robot, XmateType::XMATE7_PRO); 

    // 捏造 3 个测试姿态 (单位: rad)
    std::vector<std::array<double, 7>> test_poses = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                  // 姿态 1：零位 (直立)
        {0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0},                  // 姿态 2：常见弯曲姿态
        {0.785, 0.785, 0.0, 1.57, 0.0, 0.785, 0.0}            // 姿态 3：极度扭曲姿态
    };

    // 静态测试，速度和加速度全设为 0
    std::array<double, 7> dq_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 7> ddq_zero = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 用于接收原厂黑盒计算结果的容器
    std::array<double, 7> trq_full, trq_inertial, trq_coriolis, trq_gravity;

    for (size_t i = 0; i < test_poses.size(); ++i) {
        auto q = test_poses[i];

        // ----------------------------------------------------
        // 调用我们揪出来的神秘接口 GetTauNoFriction
        // ----------------------------------------------------
        model.GetTauNoFriction(q, dq_zero, ddq_zero, trq_full, trq_inertial, trq_coriolis, trq_gravity);

        std::cout << "--- 实验组 " << i + 1 << " ---" << std::endl;
        std::cout << "输入姿态 q:   [";
        for(int j=0; j<7; ++j) std::cout << q[j] << (j==6 ? "" : ", ");
        std::cout << "]\n";

        std::cout << "原厂重力矩 g: [";
        for(int j=0; j<7; ++j) std::cout << trq_gravity[j] << (j==6 ? "" : ", ");
        std::cout << "]\n\n";
    }

    std::cout << "✅ 计算完毕！快拿去和 Pinocchio 算出来的误差做个减法吧！\n";
    return 0;
}