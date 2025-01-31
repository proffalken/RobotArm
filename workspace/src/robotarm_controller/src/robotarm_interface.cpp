#include "robotarm_controller/robotarm_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <unistd.h>
#include <iostream>
#include <string>

using namespace LibSerial;
using std::cout;
using std::string;


namespace robotarm_controller {
    void RobotarmInterface::waitForUnlockResponse()
    {
        string response;
        while (true)
        {
            char c;
            arduino_.get(c);
            if (c == '\n')
            {
                cout << "Response: " << response << "\n";
                if (response.find("nlock") != string::npos)
                    break;
                response.clear();
            }
            else
            {
                response += c;
            }
        }
    }

    void RobotarmInterface::waitForOkResponse()
    {
        string response;
        while (true)
        {
            char c;
            arduino_.get(c);
            if (c == '\n')
            {
                cout << "Response: " << response << "\n";
                if (response.find("ok") != string::npos)
                    break;
                response.clear();
            }
            else
            {
                response += c;
            }
        }
    }

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
            // Wait for the initial "Grbl" response or ready signal
            waitForUnlockResponse();
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "There was an error when trying to connect to " << port_);
            return CallbackReturn::FAILURE;
        }


        RCLCPP_INFO(rclcpp::get_logger("RobotarmInterface"), "Connection successful, ready for commands");
        // Intialise the controller
        try{
            // Send unlock command
            cout << "Sending unlock command...\n";
            arduino_.write("$X\n", 3);
            arduino_.DrainWriteBuffer();
            waitForUnlockResponse();

        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to UNLOCK port " << port_);
            return CallbackReturn::FAILURE;
        }
        try{
            arduino_.write("G21\n", 4);
            waitForOkResponse();
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to send G21 to port " << port_);
            return CallbackReturn::FAILURE;
        }
        try{
            arduino_.write("G91\n", 4);
            waitForOkResponse();
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to send G91 to port " << port_);
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

        RCLCPP_INFO_STREAM(rclcpp::get_logger("RobotarmInterface"), "Base Value: " << position_commands_.at(0));
        float base = static_cast<float>(position_commands_.at(0));
        float shoulder = static_cast<float>(position_commands_.at(1));
        std::ostringstream msg;
        msg << "G21 F250 X" << base << " Y" << shoulder <<"\n";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RobotarmInterface"), "Message sent: " << msg.str());

        try{
            arduino_.write(msg.str().c_str(), msg.str().length());
            arduino_.DrainWriteBuffer();
            waitForOkResponse();
        }
        catch (...) {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotarmInterface"), "Something went wrong whilst trying to send " << msg.str() << " to port " << port_);
            return hardware_interface::return_type::ERROR;
        }

        previous_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;

    }

    PLUGINLIB_EXPORT_CLASS(robotarm_controller::RobotarmInterface, hardware_interface::SystemInterface)
}
