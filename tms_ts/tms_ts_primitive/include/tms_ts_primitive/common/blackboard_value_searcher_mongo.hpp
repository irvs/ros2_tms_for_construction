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
        return { InputPort<std::string>("output_port"), InputPort<std::string>("mongo_param_name"), InputPort<std::string>("mongo_value") };
    }

    NodeStatus tick() override
    {
        Optional<std::string> key1 = getInput<std::string>("output_port");
        //Optional<std::string> key2 = getInput<std::string>("mongo_record_name");
        Optional<std::string> key2 = getInput<std::string>("mongo_param_name");
        //Optional<std::string> key3 = getInput<std::string>("mongo_param_name");
        Optional<std::string> key3 = getInput<std::string>("mongo_value");
        if (!key1 || !key2 || !key3)
        {
            std::cout << "[BlackboardValueSearcherMongo] missing required input. Please fill key or value parameters." << std::endl;
            return NodeStatus::FAILURE;
        }

        mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
        mongocxx::database db = client["rostmsdb"];
        mongocxx::collection collection = db["parameter"];
        bsoncxx::builder::stream::document filter_builder;
        filter_builder << key2.value() << key3.value();
       // filter_builder << "record_name" << key2.value();
        auto filter = filter_builder.view();
        auto doc = collection.find_one(filter);

        if (!doc)
        {
            config().blackboard->set(key1.value(), false);
            std::cout << "[BlackboardValueSearcherMongo]  Stored blackboard parameter [" << key3.value() << "] : " << "false" << std::endl;
            
            return NodeStatus::SUCCESS;
        }

        try
        {
            std::cout << "[BlackboardValueSearcherMongo]  Stored blackboard parameter [" << key3.value() << "] : " << "true" << std::endl;
            config().blackboard->set(key1.value(), true);

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
