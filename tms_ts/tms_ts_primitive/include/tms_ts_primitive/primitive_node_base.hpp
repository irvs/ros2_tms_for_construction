#ifndef PRIMITIVE_NODE_BASE_HPP
#define PRIMITIVE_NODE_BASE_HPP

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

class PrimitiveNodeBase : public rclcpp::Node
{
public:
  PrimitiveNodeBase(const std::string& node_name_);

  static mongocxx::instance inst;

  // std::map<std::string, float> GetParamFromDB(std::string model_name, std::string record_name);

  // 統合版: 全てのパラメータ型を扱える関数
  std::map<std::string, std::string> GetParamFromDBAsJson(std::string model_name, std::string record_name);
  
  // 後方互換性のための既存関数（配列型）
  template <typename K, typename T>
  std::map<K, T> CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::pair<std::string, std::string>>, bool> = true);

  // 後方互換性のための既存関数（非配列型）
  template <typename K, typename T>
  std::map<K, T> CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::string>, bool> = true);

  template <typename T>
  bool CustomUpdateParamInDB(std::string model_name, std::string record_name, const std::string& target_key, const std::vector<T>& new_values);

  // JSON文字列またはBSONドキュメントを直接保存する関数
  bool UpdateParamInDBFromJson(std::string model_name, std::string record_name, const std::string& target_key, const std::string& json_str);

private:
};

static inline std::string bson_type_name(bsoncxx::type t) {
  switch (t) {
    case bsoncxx::type::k_double:   return "double";
    case bsoncxx::type::k_utf8:     return "string";
    case bsoncxx::type::k_array:    return "array";
    case bsoncxx::type::k_int32:    return "int32";
    case bsoncxx::type::k_int64:    return "int64";
    case bsoncxx::type::k_binary:   return "binary";
    default:                        return "unknown";
  }
}

// 統合版: 全てのパラメータ型（配列、ドキュメント、スカラー値）をJSON文字列として取得
inline std::map<std::string, std::string> PrimitiveNodeBase::GetParamFromDBAsJson(std::string model_name, std::string record_name) {
  mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
  mongocxx::database db = client["rostmsdb"];
  mongocxx::collection collection = db["parameter"];

  bsoncxx::builder::stream::document filter_builder;
  filter_builder << "model_name" << model_name << "record_name" << record_name;
  auto filter = filter_builder.view();
  auto result = collection.find_one(filter);

  std::map<std::string, std::string> dataMap;

  if (result) {
    auto view = result->view();
    // std::cout << "Loaded parameter data:\n" << bsoncxx::to_json(view) << "\n\n";

    for (auto&& element : view) {
      std::string key = element.key().to_string();
      if (key != "_id" && key != "model_name" && key != "record_name") {
        // 各要素を個別のJSONドキュメントとして保存
        bsoncxx::builder::basic::document doc;
        
        if (element.type() == bsoncxx::type::k_array) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_array().value));
        } else if (element.type() == bsoncxx::type::k_document) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_document().value));
        } else if (element.type() == bsoncxx::type::k_double) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_double().value));
        } else if (element.type() == bsoncxx::type::k_int32) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_int32().value));
        } else if (element.type() == bsoncxx::type::k_int64) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_int64().value));
        } else if (element.type() == bsoncxx::type::k_bool) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_bool().value));
        } else if (element.type() == bsoncxx::type::k_utf8) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_utf8().value));
        } else if (element.type() == bsoncxx::type::k_binary) {
          doc.append(bsoncxx::builder::basic::kvp(key, element.get_binary()));
        } else {
          std::cout << "Unsupported type for key \"" << key << "\": " << bson_type_name(element.type()) << std::endl;
          continue;
        }
        
        std::string json_str = bsoncxx::to_json(doc.view());
        dataMap[key] = json_str;
        // std::cout << "Stored: " << key << " = " << json_str << std::endl;
      }
    }
  } else {
    std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
  }

  return dataMap;
}

// This function is to get array-type parameters from the database. (This function only supports 2D arrays.)
template <typename K, typename T>
std::map<K, T> PrimitiveNodeBase::CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::pair<std::string, std::string>>, bool>) {
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
        std::string key = element.key().to_string();
        if (key != "_id" && key != "model_name" && key != "type" && key != "record_name") {
            
            // 配列型の場合
            if (element.type() == bsoncxx::type::k_array) {
                int index = 0;
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
                    std::cout << "Type error in array element" << std::endl;
                    std::string type_name = bson_type_name(item.type());
                    bsoncxx::builder::basic::document tmp{};
                    if (element.type() == bsoncxx::type::k_utf8) {
                      tmp.append(bsoncxx::builder::basic::kvp(key, element.get_utf8().value.to_string()));
                    } else if (element.type() == bsoncxx::type::k_bool) {
                        tmp.append(bsoncxx::builder::basic::kvp(key, element.get_bool().value));
                    } else {
                        tmp.append(bsoncxx::builder::basic::kvp(key, "unsupported_type"));
                    }   
                    std::cout << "[TypeError] array_item_type=" << type_name
                              << "  raw_value=" << bsoncxx::to_json(tmp.view())
                              << "\n";
                  }
                }
            }
            // 配列型でない場合（スカラー値）
            else {
                if (element.type() == bsoncxx::type::k_double)
                {
                  T value = static_cast<T>(element.get_double());
                  dataMap[std::make_pair(element.key().to_string(), "")] = value;
                  std::cout << "Scalar value: " << value << std::endl;
                }
                else if (element.type() == bsoncxx::type::k_int32)
                {
                  T value = static_cast<T>(element.get_int32());
                  dataMap[std::make_pair(element.key().to_string(), "")] = value;
                  std::cout << "Scalar value: " << value << std::endl;
                }
                else if (element.type() == bsoncxx::type::k_int64)
                {
                  T value = static_cast<T>(element.get_int64());
                  dataMap[std::make_pair(element.key().to_string(), "")] = value;
                  std::cout << "Scalar value: " << value << std::endl;
                }
                else {
                  std::cout << "Unsupported scalar type" << std::endl;
                  std::string type_name = bson_type_name(element.type());
                  bsoncxx::builder::basic::document tmp{};
                  if (element.type() == bsoncxx::type::k_utf8) {
                    tmp.append(bsoncxx::builder::basic::kvp(key, element.get_utf8().value.to_string()));
                  } else if (element.type() == bsoncxx::type::k_bool) {
                      tmp.append(bsoncxx::builder::basic::kvp(key, element.get_bool().value));
                  } else {
                      tmp.append(bsoncxx::builder::basic::kvp(key, "unsupported_type"));
                  }           
                  std::cout << "[TypeError] key=\"" << key
                            << "\"  type=" << type_name
                            << "  raw_value=" << bsoncxx::to_json(tmp.view())
                            << "\n";
                }
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
std::map<K, T> PrimitiveNodeBase::CustomGetParamFromDB(std::string model_name, std::string record_name, std::enable_if_t<std::is_same_v<K, std::string>, bool>) {
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
bool PrimitiveNodeBase::CustomUpdateParamInDB(std::string model_name, std::string record_name, const std::string& target_key, const std::vector<T>& new_values)
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
    update_builder << "$set" << bsoncxx::builder::stream::open_document;

    if (new_values.size() == 1) {
      // スカラー値として保存
      update_builder << target_key << new_values[0];
    } else {
      // 配列として保存
      bsoncxx::builder::basic::array array_builder;
      for (const auto& val : new_values) {
        array_builder.append(val);
      }
      update_builder << target_key << array_builder.view();
    }

    update_builder << bsoncxx::builder::stream::close_document;

    auto result = collection.update_one(filter, update_builder.view());
    if (result) {
      RCLCPP_INFO(this->get_logger(),
        "update_one: matched=%lld modified=%lld upserted=%s",
        (long long)result->matched_count(),
        (long long)result->modified_count(),
        result->upserted_id() ? "yes" : "no");
    }

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

// JSON文字列を直接MongoDBに保存する関数
inline bool PrimitiveNodeBase::UpdateParamInDBFromJson(std::string model_name, std::string record_name, const std::string& target_key, const std::string& json_str)
{
  try {
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];

    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "model_name" << model_name << "record_name" << record_name;
    auto filter = filter_builder.view();

    // JSON文字列をBSONドキュメントに変換
    auto bson_value = bsoncxx::from_json(json_str);
    
    bsoncxx::builder::stream::document update_builder;
    update_builder << "$set" << bsoncxx::builder::stream::open_document
                   << target_key << bson_value
                   << bsoncxx::builder::stream::close_document;

    mongocxx::options::update update_options;
    update_options.upsert(true);

    auto result = collection.update_one(filter, update_builder.view(), update_options);

    if (result && result->modified_count() > 0) {
      RCLCPP_INFO(this->get_logger(), "Successfully updated \"%s\" field from JSON.", target_key.c_str());
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "No document updated. (model_name: %s, record_name: %s)", model_name.c_str(), record_name.c_str());
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during MongoDB update from JSON: %s", e.what());
    return false;
  }
}

#endif // PRIMITIVE_NODE_BASE_HPP
