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

  template <typename T>
  bool CustomUpdateParamInDB(std::string model_name, std::string record_name, const std::string& target_key, const std::vector<T>& new_values);

private:
};

static inline std::string bson_type_name(bsoncxx::type t) {
  switch (t) {
    case bsoncxx::type::k_double:   return "double";
    case bsoncxx::type::k_utf8:     return "string";
    case bsoncxx::type::k_array:    return "array";
    case bsoncxx::type::k_int32:    return "int32";
    case bsoncxx::type::k_int64:    return "int64";
    default:                        return "unknown";
  }
}


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
    auto view = result->view();
    std::cout << "Loaded parameter data:\n" << bsoncxx::to_json(view) << "\n\n";

    for (auto&& element : result->view()) {
        int index = 0;
        std::string key = element.key().to_string();
        if (key != "_id" && key != "model_name" && key != "type" && key != "record_name") {
            auto array = element.get_array().value;
            for (auto&& item : array) { 
              if (item.type() == bsoncxx::type::k_double)
              {
                T value = static_cast<T>(item.get_double());
                dataMap[std::make_pair(element.key().to_string(), std::to_string(index))] = value;
                index++;
                std::cout << value << std::endl;
              }
              else if (item.type() == bsoncxx::type::k_int32)
              {
                T value = static_cast<T>(item.get_int32());
                dataMap[std::make_pair(element.key().to_string(), std::to_string(index))] = value;
                index++;
                std::cout << value << std::endl;
              }
              else if (item.type() == bsoncxx::type::k_int64)
              {
                T value = static_cast<T>(item.get_int64());
                dataMap[std::make_pair(element.key().to_string(), std::to_string(index))] = value;
                index++;
                std::cout << value << std::endl;
              }else{
                std::cout << "Type error" << std::endl;
                bsoncxx::builder::basic::document tmp_doc{};
                tmp_doc.append(bsoncxx::builder::basic::kvp(key, element.get_value()));
                std::string type_name = bson_type_name(element.type());
                bsoncxx::builder::basic::document tmp{};
                tmp.append(bsoncxx::builder::basic::kvp(key, element.get_value()));
                std::cout << "[TypeError] key=\"" << key
                          << "\"  type=" << type_name
                          << "  raw_value=" << bsoncxx::to_json(tmp.view())
                          << "\n";
                std::cout << "This node only supports array type. Types such as int32, int64, and double cannot be used, so please rewrite your parameter data accordingly." << std::endl;
              }
            }  
            // for (auto&& item : array) { 
            //     dataMap[std::make_pair(element.key().to_string(), std::to_string(index))] = static_cast<T>(item.get_double());
            //     index++;
            // }
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
    auto view = result->view();
    std::cout << "Loaded parameter data:\n" << bsoncxx::to_json(view) << "\n\n";

    for (auto&& element : result->view())
    {
      std::string key = element.key().to_string();
      if (key != "_id" && key != "model_name" && key != "type" && key != "record_name")
      {
        if (element.type() == bsoncxx::type::k_double)
        {
          T value = static_cast<T>(element.get_double());
          dataMap[key] = value;
          std::cout << value << std::endl;
        }
        else if (element.type() == bsoncxx::type::k_int32)
        {
          T value = static_cast<T>(element.get_int32().value);
          dataMap[key] = value;
          std::cout << value << std::endl;
        }
        else if (element.type() == bsoncxx::type::k_int64)
        {
          T value = static_cast<T>(element.get_int64().value);
          dataMap[key] = value;
          std::cout << value << std::endl;
        }else{
          std::cout << "Type error" << std::endl;
        bsoncxx::builder::basic::document tmp_doc{};
        tmp_doc.append(bsoncxx::builder::basic::kvp(key, element.get_value()));
        std::string type_name = bson_type_name(element.type());
        bsoncxx::builder::basic::document tmp{};
        tmp.append(bsoncxx::builder::basic::kvp(key, element.get_value()));
        std::cout << "[TypeError] key=\"" << key
                  << "\"  type=" << type_name
                  << "  raw_value=" << bsoncxx::to_json(tmp.view())
                  << "\n";
        std::cout << "This node only supports int32, int64, and double types. Types such as arrays cannot be used, so please rewrite your parameter data accordingly." << std::endl;
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

template <typename T>
bool SubtaskNodeBase::CustomUpdateParamInDB(std::string model_name, std::string record_name, const std::string& target_key, const std::vector<T>& new_values)
{
  try {
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];

    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "model_name" << model_name << "record_name" << record_name;
    auto filter = filter_builder.view();

    bsoncxx::builder::basic::array array_builder;
    for (const auto& val : new_values) {
      array_builder.append(val);
    }

    bsoncxx::builder::stream::document update_builder;
    update_builder << "$set" << bsoncxx::builder::stream::open_document
                   << target_key << array_builder.view()
                   << bsoncxx::builder::stream::close_document;

    auto result = collection.update_one(filter, update_builder.view());

    if (result && result->modified_count() > 0) {
      RCLCPP_INFO(this->get_logger(), "Successfully updated \"%s\" field.", target_key.c_str());
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "No document updated. (model_name: %s, record_name: %s)", model_name.c_str(), record_name.c_str());
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during MongoDB update: %s", e.what());
    return false;
  }
}
#endif // SUBTASK_NODE_BASE_HPP