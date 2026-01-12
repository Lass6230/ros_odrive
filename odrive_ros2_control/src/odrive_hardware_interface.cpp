
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"
#include "transmission_interface/simple_transmission.hpp"
#include "transmission_interface/transmission_loader.hpp"
#include "transmission_interface/handle.hpp"
#include <unistd.h>

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);

    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
    
    // Transmission interface
    std::vector<std::shared_ptr<transmission_interface::SimpleTransmission>> transmissions_;
    std::shared_ptr<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>> transmission_loader_;
    
    // Actuator space (motor revolutions - what ODrive uses)
    std::vector<double> actuator_positions_;
    std::vector<double> actuator_velocities_;
    std::vector<double> actuator_efforts_;
    
    // Joint space (radians - what controllers use)
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    // Commands (ros2_control => ODrives) - in ACTUATOR space (motor revolutions)
    double pos_setpoint_ = 0.0;  // [revolutions]
    double vel_setpoint_ = 0.0;  // [rev/s]
    double torque_setpoint_ = 0.0; // [Nm]

    // State (ODrives => ros2_control) - in ACTUATOR space (motor revolutions)
    double pos_estimate_ = NAN;    // [revolutions]
    double vel_estimate_ = NAN;    // [rev/s]
    double torque_target_ = NAN;   // [Nm]
    double torque_estimate_ = NAN; // [Nm]

    // Indicates which controller inputs are enabled
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    template <typename T>
    void send(const T& msg) {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);
        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];

    // Initialize transmission loader
    transmission_loader_ = std::make_shared<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>(
        "transmission_interface", 
        "transmission_interface::TransmissionLoader"
    );

    // Create axes (one per joint)
    for (const auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
    }

    // Initialize actuator and joint space vectors
    size_t num_joints = info_.joints.size();
    actuator_positions_.resize(num_joints, 0.0);
    actuator_velocities_.resize(num_joints, 0.0);
    actuator_efforts_.resize(num_joints, 0.0);
    
    joint_positions_.resize(num_joints, 0.0);
    joint_velocities_.resize(num_joints, 0.0);
    joint_efforts_.resize(num_joints, 0.0);

    // Load transmissions from URDF
    for (size_t i = 0; i < info_.transmissions.size(); ++i) {
        const auto& trans_info = info_.transmissions[i];
        
        try {
            // Load the transmission using pluginlib
            auto loader = transmission_loader_->createSharedInstance(trans_info.type);
            auto transmission_ptr = loader->load(trans_info);
            
            // Cast to SimpleTransmission (we know it's simple from our URDF)
            auto simple_trans = std::dynamic_pointer_cast<transmission_interface::SimpleTransmission>(transmission_ptr);
            if (!simple_trans) {
                RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), 
                    "Failed to cast transmission %s to SimpleTransmission", trans_info.name.c_str());
                return CallbackReturn::ERROR;
            }
            
            // Create handles for the transmission
            std::vector<transmission_interface::JointHandle> joint_handles;
            std::vector<transmission_interface::ActuatorHandle> actuator_handles;
            
            // Joint handles (controllers work in joint space)
            joint_handles.push_back(transmission_interface::JointHandle(
                trans_info.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
            joint_handles.push_back(transmission_interface::JointHandle(
                trans_info.joints[0].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
            joint_handles.push_back(transmission_interface::JointHandle(
                trans_info.joints[0].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
            
            // Actuator handles (hardware works in actuator space)
            actuator_handles.push_back(transmission_interface::ActuatorHandle(
                trans_info.actuators[0].name, hardware_interface::HW_IF_POSITION, &actuator_positions_[i]));
            actuator_handles.push_back(transmission_interface::ActuatorHandle(
                trans_info.actuators[0].name, hardware_interface::HW_IF_VELOCITY, &actuator_velocities_[i]));
            actuator_handles.push_back(transmission_interface::ActuatorHandle(
                trans_info.actuators[0].name, hardware_interface::HW_IF_EFFORT, &actuator_efforts_[i]));
            
            // Configure the transmission with handles
            simple_trans->configure(joint_handles, actuator_handles);
            
            transmissions_.push_back(simple_trans);
            
            RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), 
                "Loaded transmission %s with reduction=%.3f, offset=%.3f",
                trans_info.name.c_str(),
                simple_trans->get_actuator_reduction(),
                simple_trans->get_joint_offset());
                
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), 
                "Failed to load transmission %s: %s", trans_info.name.c_str(), ex.what());
            return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, std::placeholders::_1))) {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Failed to initialize SocketCAN on %s", can_intf_name_.c_str());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    for (auto& axis : axes_) {
        Set_Axis_State_msg_t msg;
        msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(msg);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Export JOINT space interfaces (controllers see joint space)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &joint_positions_[i]
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &joint_velocities_[i]
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &joint_efforts_[i]
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Export JOINT space interfaces (controllers work in joint space)
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &joint_positions_[i]
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &joint_velocities_[i]
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &joint_efforts_[i]
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    // Disable all interfaces that are being stopped
    for (size_t i = 0; i < axes_.size(); i++) {
        auto& axis = axes_[i];
        const std::string joint_name = info_.joints[i].name;

        std::map<std::string, bool*> interfaces = {
            {joint_name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
            {joint_name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
            {joint_name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_},
        };

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            Set_Controller_Mode_msg_t msg;
            if (axis.pos_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to position control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
                msg.Input_Mode = INPUT_MODE_POS_FILTER;
            } else if (axis.vel_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to velocity control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            } else {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to torque control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            }

            bool any_enabled = axis.pos_input_enabled_ || axis.vel_input_enabled_ || axis.torque_input_enabled_;

            if (any_enabled) {
                axis.send(msg);
            }

            Clear_Errors_msg_t msg1;
            msg1.Identify = 0;
            axis.send(msg1);

            Set_Axis_State_msg_t msg2;
            msg2.Axis_Requested_State = any_enabled ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE;
            axis.send(msg2);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    // Read CAN messages
    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    // Convert actuator space to joint space using transmissions
    for (size_t i = 0; i < transmissions_.size(); ++i) {
        // Copy actuator state from axes
        actuator_positions_[i] = axes_[i].pos_estimate_;
        actuator_velocities_[i] = axes_[i].vel_estimate_;
        actuator_efforts_[i] = axes_[i].torque_estimate_;
        
        // Transform actuator → joint space
        transmissions_[i]->actuator_to_joint();
        
        // joint_positions_[i], joint_velocities_[i], joint_efforts_[i] 
        // are now updated by the transmission
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    // Convert joint space commands to actuator space using transmissions
    for (size_t i = 0; i < transmissions_.size(); ++i) {
        // joint_positions_[i], joint_velocities_[i], joint_efforts_[i]
        // contain commands from controllers (in joint space)
        
        // Transform joint → actuator space
        transmissions_[i]->joint_to_actuator();
        
        // Copy actuator commands to axes
        axes_[i].pos_setpoint_ = actuator_positions_[i];
        axes_[i].vel_setpoint_ = actuator_velocities_[i];
        axes_[i].torque_setpoint_ = actuator_efforts_[i];
    }

    // Send commands to ODrives (now in actuator space - motor revolutions)
    for (auto& axis : axes_) {
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = axis.pos_setpoint_;  // Already in motor revolutions
            msg.Vel_FF = axis.vel_input_enabled_ ? axis.vel_setpoint_ : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = axis.vel_setpoint_;  // Already in rev/s
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;  // Already in Nm
            axis.send(msg);
        }
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < TMsg::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                // Store in ACTUATOR space (motor revolutions)
                pos_estimate_ = msg.Pos_Estimate;
                vel_estimate_ = msg.Vel_Estimate;
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                // Store in ACTUATOR space
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
