// 读出来的数据发现有问题，首先在hmi上面配置过负载和安装之后，他的计算结果保持不变，说明参数是写在他的代码里面了。
// 其次目前待代码算的动力学参数很不对，没办法用

#include <array>
#include <fstream>
#include <functional>
#include <iostream>
#include <unistd.h>
#include <vector>

#include "ini.h"
#include "move.h" // 包含 MotionFinished 函数
#include "rci_data/robot_datas.h"
#include "robot.h"

using namespace xmate;

// 定义暂存数据的结构体
struct RecordData {
  uint64_t msg_id;             // 时间戳
  std::array<double, 7> q;     // 关节角度
  std::array<double, 7> dq;    // 关节速度
  std::array<double, 7> tau_m; // 测量的关节总力矩
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

  std::cout << "正在连接机械臂 IP: " << ipaddr << " 端口: " << port << " ..."
            << std::endl;
  xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
  sleep(3);

  robot.automaticErrorRecovery();
  sleep(2);

  robot.setMotorPower(1);
  sleep(2);

  // 预分配 10 秒钟的数据内存 (10000 毫秒)
  const int MAX_RECORD_TICKS = 5000;
  std::vector<RecordData> data_log;
  data_log.reserve(MAX_RECORD_TICKS);

  std::cout << "\n================ 警告 & 提示 ================" << std::endl;
  std::cout << "即将进入【纯力矩透传 - 零重力拖动模式】！" << std::endl;
  std::cout << "机械臂将失去位置保持刚度，请将手放在机械臂下方做好保护。"
            << std::endl;
  std::cout << "你有 10 秒钟的时间可以像在太空一样拨动它！" << std::endl;
  std::cout << "3秒后开始..." << std::endl;
  sleep(3);

  int count = 0;

  robot.startMove(
      RCI::robot::StartMoveRequest::ControllerMode::kTorque,
      RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

  // ==========================================
  // 核心：1kHz 纯力矩控制回调函数
  // ==========================================
  auto torque_callback = [&](RCI::robot::RobotState robot_state) -> Torques {
    // 1. 极速记录数据
    if (count < MAX_RECORD_TICKS) {
      RecordData rd;
      rd.msg_id = robot_state.message_id;
      rd.q = robot_state.q;
      rd.dq = robot_state.dq_m;
      rd.tau_m = robot_state.tau_m;
      data_log.push_back(rd);
      count++;
    }

    // 2. 算法核心：下发附加目标力矩为 0 Nm
    // 这意味着我们不施加任何额外的力，全权交给控制器的底层动力学去抵消重力
    std::array<double, 7> tau_d = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Torques output(tau_d);

    // 3. 达到设定时间，优雅退出 1kHz 循环
    if (count >= MAX_RECORD_TICKS) {
      return MotionFinished(output);
    }

    return output;
  };

  // 将力矩回调函数交给底层，开始执行！
  robot.Control(torque_callback);

  std::cout << "\n10秒零重力体验结束！机械臂已恢复锁定。" << std::endl;
  std::cout << "正在将交互数据保存到文件..." << std::endl;

  // 优雅下电
  robot.setMotorPower(0);
  sleep(1);

  // ==========================================
  // 写入 CSV 数据文件
  // ==========================================
  std::ofstream outfile("zero_gravity_data.csv");
  outfile << "msg_id,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,tau1,"
             "tau2,tau3,tau4,tau5,tau6,tau7\n";

  for (size_t i = 0; i < data_log.size(); ++i) {
    auto &rd = data_log[i];
    outfile << rd.msg_id << ",";
    for (int j = 0; j < 7; ++j)
      outfile << rd.q[j] << ",";
    for (int j = 0; j < 7; ++j)
      outfile << rd.dq[j] << ",";
    for (int j = 0; j < 7; ++j)
      outfile << rd.tau_m[j] << (j == 6 ? "\n" : ",");
  }
  outfile.close();

  std::cout << "数据已安全保存至: build/zero_gravity_data.csv" << std::endl;
  std::cout << "程序安全退出。" << std::endl;

  return 0;
}