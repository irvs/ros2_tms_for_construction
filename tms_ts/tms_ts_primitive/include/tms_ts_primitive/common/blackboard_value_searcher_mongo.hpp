// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
 
//      http://www.apache.org/licenses/LICENSE-2.0
 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BLACKBOARD_VALUE_SEARCHER_MONGO_NODE_HPP
#define BLACKBOARD_VALUE_SEARCHER_MONGO_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/pool.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sstream>
#include <vector>
#include <cctype>

using namespace BT;

class BlackboardValueSearcherMongo : public SyncActionNode
{
public:
    BlackboardValueSearcherMongo(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config), pool_(mongocxx::uri{})
    {
        node_ = rclcpp::Node::make_shared("blackboard_value_sercher_mongo");
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~BlackboardValueSearcherMongo()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    static PortsList providedPorts()
    {
        return { OutputPort<bool>("output_port"), InputPort<std::string>("mongo_param_name"), InputPort<std::string>("mongo_value") };
    }

    static std::vector<std::string> split(const std::string& s, char delimiter)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(s);
        std::string item;

        while (std::getline(ss, item, delimiter))
        {
            // 空白除去（必要なら）
            item.erase(0, item.find_first_not_of(" "));
            item.erase(item.find_last_not_of(" ") + 1);
            tokens.push_back(item);
        }
        return tokens;
    }

    std::string expandBlackboardVar(const std::string &val) {
        std::string result = val;
        // {var_name} の形なら blackboard から取得
        if (!val.empty() && val.front() == '{' && val.back() == '}') {
            std::string key = val.substr(1, val.size() - 2);
            Optional<std::string> bb_val = config().blackboard->get<std::string>(key);
            if (bb_val) {
                result = bb_val.value();
                std::cout << "[BlackboardValueSearcherMongo]" << key << " : Get from blackboard" << std::endl;
            } else {
                std::cout << "[BlackboardValueSearcherMongo] WARNING: Blackboard key [" << key << "] not found!" << std::endl;
            }
        }
        return result;
    }


    bsoncxx::types::bson_value::value parseValue(const std::string& value)
    {
        // bool
        if (value == "true")
            return bsoncxx::types::bson_value::value{bsoncxx::types::b_bool{true}};
        if (value == "false")
            return bsoncxx::types::bson_value::value{bsoncxx::types::b_bool{false}};
        // int
        bool is_int = !value.empty() &&
        std::all_of(value.begin(), value.end(), ::isdigit);
        if (is_int)
        {
            return bsoncxx::types::bson_value::value{
                bsoncxx::types::b_int32{std::stoi(value)}
            };
        }
        // double
        try
        {
            size_t idx;
            double d = std::stod(value, &idx);
            if (idx == value.size())
            {
                return bsoncxx::types::bson_value::value{
                    bsoncxx::types::b_double{d}
                };
            }
        }
        catch (...) {}
        // string（デフォルト）
        return bsoncxx::types::bson_value::value{
            bsoncxx::types::b_utf8{value}
        };
    }


    NodeStatus tick() override
    {
       // Optional<std::string> key1 = getInput<std::string>("output_port");
        Optional<std::string> key2 = getInput<std::string>("mongo_param_name");
        Optional<std::string> key3 = getInput<std::string>("mongo_value");

        if ( !key2 || !key3)
        {
            std::cout << "[BlackboardValueSearcherMongo] missing required input." << std::endl;
            std::cout << "[BlackboardValueSearcherMongo] input param [" << key2.value() << "]" << std::endl;
            std::cout << "[BlackboardValueSearcherMongo] input value [" << key3.value() << "]" << std::endl;
            return NodeStatus::FAILURE;
        }

        auto param_names  = split(key2.value(), ',');  
        auto param_values_raw = split(key3.value(), ','); 

        if (param_names.size() != param_values_raw.size()) {
            std::cout << "[BlackboardValueSearcherMongo] size mismatch" << std::endl;
            return NodeStatus::FAILURE;
        }

        // 展開後の値を作る
        std::vector<std::string> param_values;
        for (auto &v : param_values_raw) {
            param_values.push_back(expandBlackboardVar(v));
        }

        mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
        mongocxx::database db = client["rostmsdb"];
        mongocxx::collection collection = db["parameter"];
        bsoncxx::builder::stream::document filter_builder;

        for (size_t i = 0; i < param_names.size(); ++i)
        {
            std::cout << "Filter: " << param_names[i] << " = " << param_values[i] << std::endl;
            filter_builder << param_names[i] << parseValue(param_values[i]);
        }
        std::cout << "Final BSON: " << bsoncxx::to_json(filter_builder.view()) << std::endl;

        auto filter = filter_builder.view();
        std::cout << bsoncxx::to_json(filter) << std::endl;
        auto doc = collection.find_one(filter);
        auto cursor = collection.find(filter);
        for (auto&& doc : cursor) {
            std::cout << bsoncxx::to_json(doc) << std::endl;
        }

        if (!doc)
        {
            setOutput("output_port", false);
            std::cout << "[BlackboardValueSearcherMongo]  Stored blackboard parameter [" << key2.value() << " == " << key3.value() << "] : " << "false" << std::endl;          
            return NodeStatus::SUCCESS;
        }
        
        try
        {
            setOutput("output_port", true);
            std::cout << "[BlackboardValueSearcherMongo]  Stored blackboard parameter [" << key2.value() << " == " << key3.value() << "] : true" << std::endl;
            return NodeStatus::SUCCESS;
        }

        catch (const std::exception& e)
        {
            std::cout << "[BlackboardValueSearcherMongo]  Exception caught: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    mongocxx::pool pool_; // Create a pool of connections.
};

#endif
