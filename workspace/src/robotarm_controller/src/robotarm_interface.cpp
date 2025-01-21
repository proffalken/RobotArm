#include "robotarm_controller/robotarm_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace robotarm_controller {

    std::string compensateZeros(int value)
    {
        std::string compensate_zeros = "";
        if(value < 10){
            compensate_zeros = "00";
        } else if (value < 100){
            compensate_zeros = "0";
        } else {
            compensate_zeros = "0";
        }
        return compensate_zeros;
    }

    RobotarmInterface::RobotarmInterface() {


    }

    RobotarmInterface::~RobotarmInterface() {
        if (arduino_.IsOpen()){
            try
            {
                arduino_.Close();
            } 
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst closing the connection with port " << port_);
            }
        }


    }

    CallbackReturn RobotarmInterface::on_init(const hardware_interface::HardwareInfo & hardware_info) {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try{
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "No Serial Port provided! Aborting!");
                return CallbackReturn::FAILURE;
        }
        position_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        previous_position_commands_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS;

    }


    std::vector<hardware_interface::StateInterface> RobotarmInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }

        return state_interfaces;
    }


    std::vector<hardware_interface::CommandInterface> RobotarmInterface::export_command_interfaces() 
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }

        return command_interfaces;

    }



    CallbackReturn RobotarmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Starting robot hardware");
        position_commands_ = {0.0, 0.0, 0.0, 0.0};
        previous_position_commands_ = {0.0, 0.0, 0.0, 0.0};
        position_states_ = {0.0, 0.0, 0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "There was an error when trying to connect to " << port_);
            return CallbackReturn::FAILURE;
        }


        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Connection successful, ready for commands");
        // Intialise the controller
        std::string msg;
        msg = "$X";
        try{
            arduino_.Write(msg);
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to send " << msg << " to port " << port_);
            return CallbackReturn::FAILURE;
        }
        msg = "G21";
        try{
            arduino_.Write(msg);
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to send " << msg << " to port " << port_);
            return CallbackReturn::FAILURE;
        }
        msg = "G91";
        try{
            arduino_.Write(msg);
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to send " << msg << " to port " << port_);
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotarmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Stopping the robot");

        if (arduino_.IsOpen())
        {
            try{
                arduino_.Close();
            }
            catch(...){
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "There was an error while closing the connection on " << port_);
                return CallbackReturn::FAILURE;
            }

        }
        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Hardware stopped");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RobotarmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // We don't have decoders, so we just assume that everything is fine
        position_states_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotarmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if(position_commands_ == previous_position_commands_)
        {
            // Command is same as previous, so just return OK as there's nothing to do
            return hardware_interface::return_type::OK;
        }

        int base = static_cast<int>(((position_commands_.at(0)) * 180));
//        int base = static_cast<int>((position_commands_.at(0) + 0 ));
        int shoulder = static_cast<int>(position_commands_.at(1));
        std::string msg;
        msg.append("G21 F250 X");
        msg.append(std::to_string(base));
        msg.append(" Y");
        msg.append(std::to_string(shoulder));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RobotarmInterface"), "Message sent: " << msg);
//        int base = static_cast<int>(((position_commands_.at(0) + (M_PI/2)) * 180) / M_PI);
//        msg.append("b");
//        msg.append(compensateZeros(base));
//        msg.append(std::to_string(base));
//        msg.append(",");
//        msg.append("s");
//        msg.append(compensateZeros(shoulder));
//        msg.append(std::to_string(shoulder));
//        msg.append(",");
//        int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI/2)) * 180) / M_PI);
//        msg.append("e");
//        msg.append(compensateZeros(elbow));
//        msg.append(std::to_string(elbow));
//        msg.append(",");
//        int gripper = static_cast<int>((-position_commands_.at(3) * 180) / (M_PI / 2));
//        msg.append("g");
//        msg.append(compensateZeros(gripper));
//        msg.append(std::to_string(gripper));
//        msg.append(",");

        try{
            arduino_.Write(msg);
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to send " << msg << " to port " << port_);
            return hardware_interface::return_type::ERROR;
        }

        previous_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;

    }

    PLUGINLIB_EXPORT_CLASS(robotarm_controller::RobotarmInterface, hardware_interface::SystemInterface)
}
