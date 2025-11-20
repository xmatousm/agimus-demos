// (c) 2025-05 Martin Matousek, Czech Technical University of Prague

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "agimus_demos_msgs/msg/robot_hw_state.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace agimus_debug_controllers {
    const int NUM_JOINTS = 7;

    class EffortPassThroughStatePublisher : public controller_interface::ChainableControllerInterface {
    public:
        using vector = std::array<double, NUM_JOINTS>;

        [[nodiscard]] controller_interface::InterfaceConfiguration
        command_interface_configuration()
        const override;

        [[nodiscard]] controller_interface::InterfaceConfiguration
        state_interface_configuration()
        const override;

        controller_interface::return_type update_and_write_commands(const rclcpp::Time &time,
                                                                    const rclcpp::Duration &period) final;

        std::vector<hardware_interface::CommandInterface>
        on_export_reference_interfaces() final;

        controller_interface::return_type update_reference_from_subscribers() final;

        CallbackReturn on_init() override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    private:
        // parameters
        std::string joint_prefix_;
        double report_period_;
        double elapsed_time_{0.0};
        double last_time_{0.0};

        // state variables
        vector q_;
        vector dq_;
        vector effort_;

        // inputs
        vector req_effort_;

        void updateJointStates();

        rclcpp::Publisher<agimus_demos_msgs::msg::RobotHWState>::SharedPtr state_publisher_;
    };
} // namespace agimus_debug_controllers


namespace agimus_debug_controllers {
    controller_interface::InterfaceConfiguration
    EffortPassThroughStatePublisher::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type =
                controller_interface::interface_configuration_type::INDIVIDUAL;

        for (int i = 1; i <= NUM_JOINTS; ++i) {
            config.names.push_back(joint_prefix_ + std::to_string(i) + "/"
                                   + hardware_interface::HW_IF_EFFORT);
        }

        return config;
    }

    controller_interface::InterfaceConfiguration
    EffortPassThroughStatePublisher::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type =
                controller_interface::interface_configuration_type::INDIVIDUAL;
        for (int i = 1; i <= NUM_JOINTS; ++i) {
            config.names.push_back(joint_prefix_ + std::to_string(i) + "/"
                                   + hardware_interface::HW_IF_POSITION);
            config.names.push_back(joint_prefix_ + std::to_string(i) + "/"
                                   + hardware_interface::HW_IF_VELOCITY);
            config.names.push_back(joint_prefix_ + std::to_string(i) + "/"
                                   + hardware_interface::HW_IF_EFFORT);
        }
        return config;
    }

    std::vector<hardware_interface::CommandInterface>
    EffortPassThroughStatePublisher::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.clear();
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            reference_interfaces.push_back(hardware_interface::CommandInterface(
                get_node()->get_name(),
                joint_prefix_ + std::to_string(i+1) + "/" + hardware_interface::HW_IF_EFFORT,
                &reference_interfaces_[i]));
        }

        return reference_interfaces;
    }

    controller_interface::return_type EffortPassThroughStatePublisher::update_reference_from_subscribers() {
        return controller_interface::return_type::OK;
    }

     controller_interface::return_type EffortPassThroughStatePublisher::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration & period) {

        updateJointStates();

        double dt = period.seconds();
        elapsed_time_ = elapsed_time_ + dt;

        for (int i = 0; i < NUM_JOINTS; ++i) {
            req_effort_[i] = reference_interfaces_[i];
            command_interfaces_[i].set_value(req_effort_[i]);
        }

        if (report_period_ >= 0.0)
            if (elapsed_time_ - last_time_ > report_period_) {
                last_time_ = elapsed_time_;
                auto message = agimus_demos_msgs::msg::RobotHWState();
                message.commanded_torque = req_effort_;
                message.measured_torque = effort_;
                message.commanded_joint_position = q_;
                message.measured_joint_position = q_;
                state_publisher_->publish(message);
            }

        return controller_interface::return_type::OK;
    }

    CallbackReturn EffortPassThroughStatePublisher::on_init() {
        try {
            auto_declare<std::string>("joint_prefix", "");
            auto_declare<double>("report_period", -1.0);

        } catch (const std::exception &e) {
            fprintf(stderr, "on_init: failed: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        reference_interfaces_.resize(NUM_JOINTS,
                                     std::numeric_limits<double>::quiet_NaN());

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EffortPassThroughStatePublisher::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_prefix_ = get_node()->get_parameter("joint_prefix").as_string();

        report_period_ = get_node()->get_parameter("report_period").as_double();

        state_publisher_ = get_node()->create_publisher<
            agimus_demos_msgs::msg::RobotHWState>(
            "robot_state", 10);

        RCLCPP_INFO(get_node()->get_logger(), "on_configure: SUCCESS");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn EffortPassThroughStatePublisher::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        updateJointStates();
        RCLCPP_INFO(get_node()->get_logger(), "on_activate: SUCCESS");

        return CallbackReturn::SUCCESS;
    }

    void EffortPassThroughStatePublisher::updateJointStates() {
        for (auto i = 0; i < NUM_JOINTS; ++i) {
            const auto &position_interface = state_interfaces_[3 * i];
            const auto &velocity_interface = state_interfaces_[3 * i + 1];
            const auto &effort_interface = state_interfaces_[3 * i + 2];

            assert(position_interface.get_interface_name() == "position");
            assert(velocity_interface.get_interface_name() == "velocity");
            assert(velocity_interface.get_interface_name() == "effort");

            q_[i] = position_interface.get_value();
            dq_[i] = velocity_interface.get_value();
            effort_[i] = effort_interface.get_value();
        }
    }
} // namespace agimus_debug_controllers


#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE

PLUGINLIB_EXPORT_CLASS(agimus_debug_controllers::EffortPassThroughStatePublisher,
                       controller_interface::ChainableControllerInterface)
