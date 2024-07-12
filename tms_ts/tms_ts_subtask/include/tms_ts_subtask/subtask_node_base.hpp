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

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

class SubtaskNodeBase : public rclcpp::Node
{
public:
  SubtaskNodeBase(const std::string& node_name_);

  static mongocxx::instance inst;

  // std::map<std::string, float> GetParamFromDB(std::string model_name, std::string record_name);

  template <typename K, typename T>
  std::map<K, T> CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::string>, bool> = true);

  template <typename K, typename T>
  std::map<K, T> CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::pair<std::string, std::string>>, bool> = true);

private:
};

// This function is to get array-type parameters from the database. (This function only supports 2D arrays.)
template <typename K, typename T>
std::map<K, T> SubtaskNodeBase::CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::pair<std::string, std::string>>, bool>) {
  mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
  mongocxx::database db = client["rostmsdb"];
  mongocxx::collection collection = db["parameter"];
  bsoncxx::builder::stream::document filter_builder;
  filter_builder << "model_name" << model_name << "record_name" << record_name;
  auto filter = filter_builder.view();
  auto result = collection.find_one(filter);
  if (result)
  {
    std::map<K,T> dataMap;

    for (auto&& element : result->view()) {
        int index = 0;
        std::string key = element.key().to_string();
        if (key != "_id" && key != "model_name" && key != "type" && key != "record_name") {
            auto array = element.get_array().value;
            for (auto&& item : array) { 
                dataMap[std::make_pair(element.key().to_string(), std::to_string(index))] = static_cast<T>(item.get_double());
                index++;
            }
        }
    }

    return dataMap;
  }
  else
  {
    std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
    return std::map<K, T>();
  }
}

// This function is to get non-array-type parameters from the database.
template <typename K, typename T>
std::map<K, T> SubtaskNodeBase::CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::string>, bool>) {
  mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
  mongocxx::database db = client["rostmsdb"];
  mongocxx::collection collection = db["parameter"];

  // Query to MongoDB
  bsoncxx::builder::stream::document filter_builder;
  filter_builder << "model_name" << model_name << "record_name" << record_name;
  auto filter = filter_builder.view();
  auto result = collection.find_one(filter);
  if (result)
  {
    std::map<K,T> dataMap;

    for (auto&& element : result->view())
    {
      std::string key = element.key().to_string();
      if (key != "_id" && key != "model_name" && key != "type" && key != "record_name")
      {
        if (element.type() == bsoncxx::type::k_double)
        {
          T value = static_cast<T>(element.get_double());
          dataMap[key] = value;
        }
        else if (element.type() == bsoncxx::type::k_int32)
        {
          T value = static_cast<T>(element.get_int32().value);
          dataMap[key] = value;
        }
        else if (element.type() == bsoncxx::type::k_int64)
        {
          T value = static_cast<T>(element.get_int64().value);
          dataMap[key] = value;
        }
      }
    }

    return dataMap;
  }
  else
  {
    std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
    return std::map<K, T>();
  }
}

#endif // SUBTASK_NODE_BASE_HPP
