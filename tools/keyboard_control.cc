/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/common/util/message_util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/control_cmd.pb.h"

// keyboardinput
#include <stdio.h>
#include <termio.h>

namespace keyboard {

using apollo::canbus::Chassis;
using apollo::common::time::Clock;
using apollo::control::ControlCommand;
using apollo::cyber::CreateNode;
using apollo::cyber::Node;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class KeyControlTerminal {
 public:
  KeyControlTerminal() : node_(CreateNode("KeyControlTerminal")) {}

  void init() {
    // get_keyboard_value();
    chassis_reader_ = node_->CreateReader<Chassis>(
        FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
          on_chassis(*chassis);
        });
    cmd_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    control_command.set_gear_location(Chassis::GEAR_DRIVE);
    control_command.set_steering_rate(100);
    terminal_thread_.reset(new std::thread([this] { terminal_thread_func(); }));
  }

  void get_keyboard_value() {
    int i = 0;
    while (i < 4) {
      KeyValues[i] = scanKeyboard();
      std::cout << "KeyValues[" << i << "]=" << KeyValues[i];
      i++;
    }
  }

  void send() {
    timestamp = Clock::NowInSeconds();
    control_command.mutable_header()->set_timestamp_sec(timestamp);
    cmd_writer_->Write(std::make_shared<ControlCommand>(control_command));
  }

  void on_chassis(const Chassis &chassis) {
    control_command.set_gear_location(chassis.gear_location());
  }

  void terminal_thread_func() {
    // AINFO << "GO TO THREAD";
    bool exit = false;
    int keyboardinput = 0;
    double throttle = 0;

    system("stty -icanon");//GetInput()所需，关闭缓冲区，无需enter键
    while (!exit) {
      // keyboardinput = scanKeyboard();
      keyboardinput = GetInput();
      // AINFO << keyboardinput;
      switch (keyboardinput) {
        case 56:  // UP
          throttle = control_command.throttle() + 10;
          if (throttle >= 100) {
            control_command.set_throttle(100);
          } else {
            control_command.set_throttle(throttle);
          }
          control_command.set_brake(0);
          send();
          keyboardinput = 0;
          // AINFO << "command:UP";
          break;
        case 53:  // DOWN
          control_command.set_throttle(0);
          control_command.set_brake(50);
          send();
          keyboardinput = 0;
          // AINFO << "command:DWON";
          break;
        case 52:  // LEFT
          control_command.set_steering_target(
              control_command.steering_target() + 5);
          send();
          keyboardinput = 0;
          // AINFO << "command:LEFT";
          break;
        case 54:  // RIGHT
          control_command.set_steering_target(
              control_command.steering_target() - 5);
          send();
          keyboardinput = 0;
          // AINFO << "command:RIGHT";
          break;
        case 55:
          exit = true;
          AINFO << "EXIT terminal_thread_func()!";
          break;
        default:
          control_command.set_throttle(0);
          control_command.set_brake(0);
          control_command.set_steering_target(0);
          // AINFO << "Default";
          break;
      }
    }
  }

  void stop() { terminal_thread_->join(); }

  // https://blog.csdn.net/u013467442/article/details/51173441
  //功能：实现无enter键输入
  //问题：无法实现无阻塞输入
  int scanKeyboard() {
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    in = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
  }

  // https://blog.csdn.net/boyixuanbo/article/details/80775807
  int GetInput() {
    // fd_set 为long型数组,其每个元素都能和打开的文件句柄建立联系
    fd_set rfds;
    struct timeval tv;
    int c = 0;
    //将　rfds数组清零
    FD_ZERO(&rfds);
    //将rfds的第0位置为１，这样fd=1的文件描述符就添加到了rfds中
    //最初　rfds为00000000,添加后变为10000000
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 150000;  //设置等待超时时间,s+us
    //检测键盘是否有输入
    //由内核根据io状态修改rfds的内容，来判断执行了select的进程哪个句柄可读
    if (select(1, &rfds, NULL, NULL, &tv) > 0) {
      c = getchar();
      return c;
    }
    //没有数据返回n
    return 0;
  }

 private:
  std::unique_ptr<std::thread> terminal_thread_;
  std::shared_ptr<Writer<ControlCommand>> cmd_writer_;
  std::shared_ptr<Reader<Chassis>> chassis_reader_;
  std::shared_ptr<Node> node_;
  int KeyValues[4];
  apollo::control::ControlCommand control_command;
  double timestamp = 0, consumetime = 0;
};

}  // namespace keyboard

int main(int argc, char **argv) {
  apollo::cyber::Init("keyboard_control_terminal");
  FLAGS_alsologtostderr = true;
  keyboard::KeyControlTerminal key_control_terminal;
  key_control_terminal.init();
  // key_control_terminal.info();
  apollo::cyber::WaitForShutdown();
  key_control_terminal.stop();
  AINFO << "EXIT Keyboard Control Terminal!";
  return 0;
}