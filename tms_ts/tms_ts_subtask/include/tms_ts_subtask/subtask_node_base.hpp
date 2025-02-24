#ifndef SUBTASK_NODE_BASE_HPP
#define SUBTASK_NODE_BASE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <map>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tms_msg_db/srv/tmsdb_get_parameter.hpp"

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

class SubtaskNodeBase : public rclcpp::Node
{
public:
    SubtaskNodeBase(const std::string& node_name_);

    template <typename K, typename T>
    std::map<K, T> CustomGetParamFromDB(std::string model_name, std::string record_name, 
                                        std::enable_if_t<std::is_same_v<K, std::string>, bool> = true)
    {
        auto request = std::make_shared<tms_msg_db::srv::TmsdbGetParameter::Request>();
        request->model_name = model_name;
        request->record_name = record_name;

        while (!client_->wait_for_service(std::chrono::seconds(3))) 
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service to become available...");
        }

        auto future = client_->async_send_request(request);
        std::map<K, T> dataMap;

        try 
        {
            auto response = future.get();
            for (size_t i = 0; i < response->keys.size(); ++i) 
            {
                std::string value_str = response->values[i];
                if constexpr (std::is_same_v<T, double>) {
                    dataMap[response->keys[i]] = std::stod(value_str);
                } else if constexpr (std::is_same_v<T, int>) {
                    dataMap[response->keys[i]] = std::stoi(value_str);
                } else {
                    dataMap[response->keys[i]] = value_str;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Received parameters from service.");
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }

        return dataMap;
    }

    template <typename K, typename T>
    std::map<K, T> CustomGetParamFromDB(std::string model_name, std::string record_name, 
                                        std::enable_if_t<std::is_same_v<K, std::pair<std::string, std::string>>, bool> = true)
    {
        auto request = std::make_shared<tms_msg_db::srv::TmsdbGetParameter::Request>();
        request->model_name = model_name;
        request->record_name = record_name;

        while (!client_->wait_for_service(std::chrono::seconds(3))) 
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service to become available...");
        }

        auto future = client_->async_send_request(request);
        std::map<K, T> dataMap;

        try 
        {
            auto response = future.get();
            for (size_t i = 0; i < response->keys.size(); ++i) 
            {
                std::string param_name, index;
                size_t pos = response->keys[i].find(':');
                if (pos != std::string::npos) 
                {
                    param_name = response->keys[i].substr(0, pos);
                    index = response->keys[i].substr(pos + 1);
                    std::string value_str = response->values[i];
                    if constexpr (std::is_same_v<T, double>) {
                        dataMap[{param_name, index}] = std::stod(value_str);
                    } else if constexpr (std::is_same_v<T, int>) {
                        dataMap[{param_name, index}] = std::stoi(value_str);
                    } else {
                        dataMap[{param_name, index}] = value_str;
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "Received 2D parameters from service.");
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }

        return dataMap;
    }

private:
    static mongocxx::instance inst;
    rclcpp::Client<tms_msg_db::srv::TmsdbGetParameter>::SharedPtr client_;
};


#endif // SUBTASK_NODE_BASE_HPP
