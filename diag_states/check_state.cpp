#include <iostream>
#include <unistd.h>
#include <functional> // 💥 补丁：必须放在 robot.h 前面擦屁股！
#include "robot.h"
#include "rci_data/robot_datas.h"

using namespace xmate;

int main() {
  std::string ipaddr = "192.168.0.160";
  uint16_t port = 1337;
  xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO, false);
  sleep(1);

  std::cout << "-> 正在清除机器人底层错误..." << std::endl;
  robot.automaticErrorRecovery();
  sleep(1);

  std::cout << "-> 正在给电机上使能 (听咔哒声)..." << std::endl;
  robot.setMotorPower(1);
  sleep(2);

  std::cout << "-> 正在发送 TCP 启动指令 (kTorque + kIdle)..." << std::endl;
  try {
      robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque, 
                      RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);
  } catch (const std::exception& e) {
      std::cout << "❌ startMove 报错: " << e.what() << std::endl;
      return -1;
  }
  
  // 💥 关键：给底层状态机 1 秒钟的时间去切换状态
  sleep(1); 

  std::cout << "===============================================" << std::endl;
  std::cout << "🔍 机器人底层状态深度核查 (startMove 之后)" << std::endl;
  std::cout << "===============================================" << std::endl;

  try {
      // 读取最新的底层状态包
      auto s = robot.receiveRobotState();
      
      std::cout << "robot_mode=" << static_cast<int>(s.robot_mode)
                << ", controller_mode=" << static_cast<int>(s.controller_mode)
                << ", motion_generator_mode=" << static_cast<int>(s.motion_generator_mode)
                << ", cmd_rate=" << s.control_command_success_rate << std::endl;

      bool has_error = false;
      for (size_t i = 0; i < s.errors.size(); ++i) {
          if (s.errors[i]) {
              std::cout << "❌ 发现底层隐藏错误位 (error bit): " << i << std::endl;
              has_error = true;
          }
      }
      if (!has_error) {
          std::cout << "✅ errors 数组全为 0，无底层暗病。" << std::endl;
      }
  } catch (const std::exception& e) {
      std::cout << "❌ 获取状态 UDP 包失败: " << e.what() << std::endl;
  }

  std::cout << "-> 测试结束！机械臂已恢复下电。" << std::endl;
  robot.setMotorPower(0);
  
  return 0;
}