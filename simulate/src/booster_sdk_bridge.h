#pragma once

// booster_mujoco DDS bridge
//
// Adaptation of unitree_mujoco's unitree_sdk2_bridge.h for the
// booster_robotics_sdk. The code structure is kept identical to the original;
// only the SDK API calls are replaced:
//   - unitree::robot::ChannelFactory           -> booster::robot::ChannelFactory
//   - unitree::robot::ChannelSubscriber/Pub... -> booster::robot::ChannelSubscriber/ChannelPublisher
//   - unitree::robot::go2::publisher::LowState -> booster_interface::msg::LowState
//   - unitree::common::UnitreeJoystick         -> booster::common::BoosterJoystick
//   - unitree::common::RecurrentThread         -> booster::common::RecurrentThread
//
// Topics (same semantics as unitree_mujoco, booster b1 naming):
//   subscribe: rt/joint_ctrl  (booster_interface::msg::LowCmd)
//   publish:   rt/low_state   (booster_interface::msg::LowState)
//   publish:   rt/remote_controller_state (booster_interface::msg::RemoteControllerState)

#include <mujoco/mujoco.h>

#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/idl/b1/MotorState.h>
#include <booster/idl/b1/RemoteControllerState.h>
#include <booster/idl/nav_msgs/Odometry.h>
#include <booster/idl/geometry_msgs/Point.h>
#include <booster/idl/geometry_msgs/Quaternion.h>
#include <booster/idl/geometry_msgs/Vector3.h>
#include <booster/idl/std_msgs/Header.h>

#include "param.h"
#include "physics_joystick.h"
#include "utils/recurrent_thread.hpp"

#define MOTOR_SENSOR_NUM 3

// The booster_robotics_sdk does not define a topic for the remote controller
// state; the simulator publishes it on the following topic.
static const std::string kTopicRemoteControllerState = "rt/remote_controller_state";

class BoosterSDKBridgeBase
{
public:
    BoosterSDKBridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "yahboom") {
                joystick  = std::make_shared<YahboomJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "betop") {
                joystick  = std::make_shared<BeTopJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };

        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });

        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    int have_imu_ = false;
    int have_frame_sensor_ = false;

    std::shared_ptr<booster::common::BoosterJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

        for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
        {
            const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
            if (strcmp(name, "imu_quat") == 0) {
                have_imu_ = true;
            }
            if (strcmp(name, "frame_pos") == 0) {
                have_frame_sensor_ = true;
            }
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public BoosterSDKBridgeBase
{
using RemoteController_t = booster_interface::msg::RemoteControllerState;
using Odometry_t = nav_msgs::msg::Odometry;

public:
    RobotBridge(mjModel *model, mjData *data) : BoosterSDKBridgeBase(model, data)
    {
        lowcmd = std::make_shared<booster::robot::ChannelSubscriber<LowCmd_t>>(
            booster::robot::b1::kTopicJointCtrl,
            [this](const void *msg){
                if(!mj_data_) return;
                const LowCmd_t *msg_ = static_cast<const LowCmd_t*>(msg);
                const std::vector<booster_interface::msg::MotorCmd> &motor_cmd = msg_->motor_cmd();
                int n = std::min<int>(motor_cmd.size(), num_motor_);
                for(int i(0); i<n; i++) {
                    const auto & m = motor_cmd[i];
                    mj_data_->ctrl[i] = m.tau() +
                                        m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                        m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
                }
            });
        lowcmd->InitChannel();

        lowstate = std::make_shared<booster::robot::ChannelPublisher<LowState_t>>(booster::robot::b1::kTopicLowState);
        lowstate->InitChannel();
        lowstate_msg_.motor_state_serial().resize(num_motor_);

        if(joystick) {
            remote_controller = std::make_shared<booster::robot::ChannelPublisher<RemoteController_t>>(kTopicRemoteControllerState);
            remote_controller->InitChannel();
        }

        if(have_frame_sensor_) {
            odometry = std::make_shared<booster::robot::ChannelPublisher<Odometry_t>>(booster::robot::b1::kTopicRosOdometer);
            odometry->InitChannel();
            odometry_msg_.header().frame_id() = "odom";
            odometry_msg_.child_frame_id() = "base_link";
        }

        thread_ = std::make_shared<booster::common::RecurrentThread>(
            "booster_bridge", 1, 2000, std::bind(&RobotBridge::run, this));
    }

    void run()
    {
        if(!mj_data_) return;
        if(joystick) { joystick->update(); }

        // lowstate
        {
            auto &motor_state = lowstate_msg_.motor_state_serial();
            for(int i(0); i<num_motor_; i++) {
                motor_state[i].q() = mj_data_->sensordata[i];
                motor_state[i].dq() = mj_data_->sensordata[i + num_motor_];
                motor_state[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            if(have_frame_sensor_) {
                auto &imu_state = lowstate_msg_.imu_state();

                double w = mj_data_->sensordata[dim_motor_sensor_ + 0];
                double x = mj_data_->sensordata[dim_motor_sensor_ + 1];
                double y = mj_data_->sensordata[dim_motor_sensor_ + 2];
                double z = mj_data_->sensordata[dim_motor_sensor_ + 3];

                imu_state.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                imu_state.rpy()[1] = asin(2 * (w * y - z * x));
                imu_state.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

                imu_state.gyro()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
                imu_state.gyro()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
                imu_state.gyro()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

                imu_state.acc()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
                imu_state.acc()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
                imu_state.acc()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
            }
            lowstate->Write(&lowstate_msg_);
        }
        // odometry (high state: base position/velocity/orientation)
        if(odometry && have_frame_sensor_) {
            odometry_msg_.pose().pose().position().x() = mj_data_->sensordata[dim_motor_sensor_ + 10];
            odometry_msg_.pose().pose().position().y() = mj_data_->sensordata[dim_motor_sensor_ + 11];
            odometry_msg_.pose().pose().position().z() = mj_data_->sensordata[dim_motor_sensor_ + 12];
            odometry_msg_.twist().twist().linear().x() = mj_data_->sensordata[dim_motor_sensor_ + 13];
            odometry_msg_.twist().twist().linear().y() = mj_data_->sensordata[dim_motor_sensor_ + 14];
            odometry_msg_.twist().twist().linear().z() = mj_data_->sensordata[dim_motor_sensor_ + 15];
            odometry_msg_.pose().pose().orientation().w() = mj_data_->sensordata[dim_motor_sensor_ + 0];
            odometry_msg_.pose().pose().orientation().x() = mj_data_->sensordata[dim_motor_sensor_ + 1];
            odometry_msg_.pose().pose().orientation().y() = mj_data_->sensordata[dim_motor_sensor_ + 2];
            odometry_msg_.pose().pose().orientation().z() = mj_data_->sensordata[dim_motor_sensor_ + 3];
            odometry_msg_.twist().twist().angular().x() = mj_data_->sensordata[dim_motor_sensor_ + 4];
            odometry_msg_.twist().twist().angular().y() = mj_data_->sensordata[dim_motor_sensor_ + 5];
            odometry_msg_.twist().twist().angular().z() = mj_data_->sensordata[dim_motor_sensor_ + 6];
            odometry->Write(&odometry_msg_);
        }
        // remote controller
        if(joystick && remote_controller) {
            fillRemoteControllerState(remote_msg_);
            remote_controller->Write(&remote_msg_);
        }
    }

    std::shared_ptr<booster::robot::ChannelSubscriber<LowCmd_t>> lowcmd;
    std::shared_ptr<booster::robot::ChannelPublisher<LowState_t>> lowstate;
    std::shared_ptr<booster::robot::ChannelPublisher<RemoteController_t>> remote_controller;
    std::shared_ptr<booster::robot::ChannelPublisher<Odometry_t>> odometry;

private:
    void fillRemoteControllerState(RemoteController_t &msg)
    {
        msg.lx(joystick->lx());
        msg.ly(-joystick->ly()); // 与实机遥控器的ly方向定义对齐
        msg.rx(joystick->rx());
        msg.ry(joystick->ry());

        msg.a(joystick->A() != 0);
        msg.b(joystick->B() != 0);
        msg.x(joystick->X() != 0);
        msg.y(joystick->Y() != 0);
        msg.lb(joystick->LB() != 0);
        msg.rb(joystick->RB() != 0);
        msg.lt(joystick->LT() > 0.5f);
        msg.rt(joystick->RT() > 0.5f);
        msg.ls(joystick->LS() != 0);
        msg.rs(joystick->RS() != 0);
        msg.back(joystick->back() != 0);
        msg.start(joystick->start() != 0);

        bool hat_u = joystick->up() != 0;
        bool hat_d = joystick->down() != 0;
        bool hat_l = joystick->left() != 0;
        bool hat_r = joystick->right() != 0;
        msg.hat_u(hat_u);
        msg.hat_d(hat_d);
        msg.hat_l(hat_l);
        msg.hat_r(hat_r);
        msg.hat_lu(hat_u && hat_l);
        msg.hat_ld(hat_d && hat_l);
        msg.hat_ru(hat_u && hat_r);
        msg.hat_rd(hat_d && hat_r);

        using RemoteControllerEvent = booster::robot::b1::RemoteControllerEvent;
        uint32_t ev = RemoteControllerEvent::NONE;
        if (joystick->A.on_pressed || joystick->B.on_pressed || joystick->X.on_pressed ||
            joystick->Y.on_pressed || joystick->LB.on_pressed || joystick->RB.on_pressed ||
            joystick->LS.on_pressed || joystick->RS.on_pressed ||
            joystick->back.on_pressed || joystick->start.on_pressed ||
            joystick->up.on_pressed || joystick->down.on_pressed ||
            joystick->left.on_pressed || joystick->right.on_pressed) {
            ev = RemoteControllerEvent::BUTTON_DOWN;
        } else if (joystick->A.on_released || joystick->B.on_released || joystick->X.on_released ||
            joystick->Y.on_released || joystick->LB.on_released || joystick->RB.on_released ||
            joystick->LS.on_released || joystick->RS.on_released ||
            joystick->back.on_released || joystick->start.on_released ||
            joystick->up.on_released || joystick->down.on_released ||
            joystick->left.on_released || joystick->right.on_released) {
            ev = RemoteControllerEvent::BUTTON_UP;
        } else if (std::fabs(msg.lx() - last_remote_.lx()) > 0.01 ||
                   std::fabs(msg.ly() - last_remote_.ly()) > 0.01 ||
                   std::fabs(msg.rx() - last_remote_.rx()) > 0.01 ||
                   std::fabs(msg.ry() - last_remote_.ry()) > 0.01) {
            ev = RemoteControllerEvent::AXIS;
        }
        msg.event(ev);
        last_remote_ = msg;
    }

    LowState_t lowstate_msg_;
    RemoteController_t remote_msg_;
    RemoteController_t last_remote_;
    Odometry_t odometry_msg_;

    booster::common::RecurrentThreadPtr thread_;
};

using B1Bridge = RobotBridge<
    booster_interface::msg::LowCmd,
    booster_interface::msg::LowState>;
