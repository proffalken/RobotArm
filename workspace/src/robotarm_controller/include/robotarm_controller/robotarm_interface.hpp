#ifndef ROBOTARM_INTERFACE_H
#define ROBOTARM_INTERFACE_H
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>
#include <libserial/SerialStream.h>

namespace robotarm_controller
{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RobotarmInterface : public hardware_interface::SystemInterface
    {
        public:
            RobotarmInterface();
            virtual ~RobotarmInterface();
            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;


            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


        private:
            void waitForUnlockResponse();
            void waitForOkResponse();
            LibSerial::SerialStream arduino_;
            std::string port_;
            std::vector<double> position_commands_;
            std::vector<double> previous_position_commands_;
            std::vector<double> position_states_;
           
    };
}

#endif
