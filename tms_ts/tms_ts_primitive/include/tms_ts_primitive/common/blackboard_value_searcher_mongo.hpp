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
#include <mongocxx/pool.hpp>
#include "behaviortree_cpp_v3/action_node.h"
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
        node_ = rclcpp::Node::make_shared("blackboard_value_searcher_mongo");
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~BlackboardValueSearcherMongo()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable()) spin_thread_.join();
    }

    static PortsList providedPorts()
    {
        return {
            OutputPort<bool>("output_port"),
            InputPort<std::string>("mongo_param_name"),
            InputPort<std::string>("mongo_value")
        };
    }

    static std::vector<std::string> split(const std::string& s, char delimiter)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(s);
        std::string item;

        while (std::getline(ss, item, delimiter))
        {
            item.erase(0, item.find_first_not_of(" "));
            item.erase(item.find_last_not_of(" ") + 1);
            tokens.push_back(item);
        }
        return tokens;
    }

    // =========================
    // Blackboard → BSON変換（型保持）
    // =========================
    bsoncxx::types::bson_value::value toBson(const std::string &val)
    {
        if (!val.empty() && val.front() == '\\' && val.back() == '\\')
        {
            std::string key = val.substr(1, val.size() - 2);
            auto bb = config().blackboard;
            BT::Any* any = bb->getAny(key);

            if (!any)
                throw std::runtime_error("Blackboard key not found: " + key);

            if (any->type() == typeid(bool))
                return bsoncxx::types::b_bool{any->cast<bool>()};

            if (any->type() == typeid(int))
                return bsoncxx::types::b_int32{any->cast<int>()};

            if (any->type() == typeid(double))
                return bsoncxx::types::b_double{any->cast<double>()};

            if (any->type() == typeid(std::string))
                return bsoncxx::types::b_utf8{any->cast<std::string>()};

            throw std::runtime_error("Unsupported Blackboard type");
        }

        // literal fallback
        return parseValue(val);
    }

    // =========================
    // literal string → BSON推定
    // =========================
    bsoncxx::types::bson_value::value parseValue(const std::string& value)
    {
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
        {
            return bsoncxx::types::b_utf8{value.substr(1, value.size() - 2)};
        }

        if (value == "true")  return bsoncxx::types::b_bool{true};
        if (value == "false") return bsoncxx::types::b_bool{false};

        bool is_int =
            !value.empty() &&
            (std::isdigit(value[0]) || value[0] == '-') &&
            std::all_of(value.begin() + 1, value.end(), ::isdigit);

        if (is_int)
            return bsoncxx::types::b_int32{std::stoi(value)};

        try {
            size_t idx;
            double d = std::stod(value, &idx);
            if (idx == value.size())
                return bsoncxx::types::b_double{d};
        } catch (...) {}

        return bsoncxx::types::b_utf8{value};
    }

    // =========================
    // tick
    // =========================
    NodeStatus tick() override
    {
        auto key2 = getInput<std::string>("mongo_param_name");
        auto key3 = getInput<std::string>("mongo_value");

        if (!key2 || !key3)
            return NodeStatus::FAILURE;

        std::string mongo_value = key3.value();

        if (!mongo_value.empty() && mongo_value.front() == '=')
            mongo_value.erase(0, 1);

        auto param_names = split(key2.value(), ',');
        auto param_values_raw = split(mongo_value, ',');

        if (param_names.size() != param_values_raw.size())
            return NodeStatus::FAILURE;

        mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
        auto db = client["rostmsdb"];
        auto collection = db["parameter"];

        bsoncxx::builder::stream::document filter;

        for (size_t i = 0; i < param_names.size(); ++i)
        {
            auto bson_val = toBson(param_values_raw[i]);

            // デバッグ表示（安全）
            bsoncxx::builder::stream::document dbg;
            dbg << "v" << bson_val;
            std::cout << bsoncxx::to_json(dbg.view()) << std::endl;

            // Mongo filter
            filter << param_names[i] << bson_val;
        }


        auto result = collection.find_one(filter.view());

        setOutput("output_port", result.has_value());

        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    mongocxx::pool pool_;
};

#endif