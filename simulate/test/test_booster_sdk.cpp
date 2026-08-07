#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>

#include <iostream>
#include <unistd.h>

#define TOPIC_LOWSTATE "rt/low_state"
#define TOPIC_LOWCMD "rt/joint_ctrl"

using namespace booster::robot;
using namespace booster::common;

void LowStateHandler(const void *msg)
{
    const booster_interface::msg::LowState *s = static_cast<const booster_interface::msg::LowState *>(msg);

    std::cout << "motor_state_serial size: " << s->motor_state_serial().size() << std::endl;
    std::cout << "Rpy: "
              << s->imu_state().rpy()[0] << " "
              << s->imu_state().rpy()[1] << " "
              << s->imu_state().rpy()[2] << " " << std::endl;
    std::cout << "motor[0] q: " << s->motor_state_serial()[0].q()
              << " dq: " << s->motor_state_serial()[0].dq()
              << " tau_est: " << s->motor_state_serial()[0].tau_est() << std::endl;
}

int main(int argc, char *argv[])
{
    std::string interface = "lo";
    if (argc >= 2) {
        interface = argv[1];
    }
    ChannelFactory::Instance()->Init(1, interface);

    ChannelSubscriber<booster_interface::msg::LowState> lowstate_suber(TOPIC_LOWSTATE);
    lowstate_suber.InitChannel(LowStateHandler);

    ChannelPublisher<booster_interface::msg::LowCmd> low_cmd_puber(b1::kTopicJointCtrl);
    low_cmd_puber.InitChannel();

    while (true)
    {
        booster_interface::msg::LowCmd low_cmd;
        std::vector<booster_interface::msg::MotorCmd> motor_cmds;
        for (int i = 0; i < int(b1::kJointCnt); i++)
        {
            booster_interface::msg::MotorCmd motor_cmd;
            motor_cmd.q() = 0;
            motor_cmd.kp() = 0;
            motor_cmd.dq() = 0;
            motor_cmd.kd() = 0;
            motor_cmd.tau() = 1;
            motor_cmds.push_back(motor_cmd);
        }
        low_cmd.cmd_type(booster_interface::msg::PARALLEL);
        low_cmd.motor_cmd(motor_cmds);
        low_cmd_puber.Write(&low_cmd);
        usleep(2000);
    }

    return 0;
}
