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

#ifndef VALUE_WRITER_MONGO_NODE_HPP
#define VALUE_WRITER_MONGO_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <iostream>

#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/stream/document.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/pool.hpp>

#include <bsoncxx/json.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/logger.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;
using bsoncxx::builder::stream::open_document;
using bsoncxx::builder::stream::close_document;

class MongoValueWriter : public SyncActionNode
{
public:
    MongoValueWriter(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config), pool_(mongocxx::uri{})
    {
        node_ = rclcpp::Node::make_shared("value_writer_mongo");
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~MongoValueWriter()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable())
            spin_thread_.join();
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("input_value"),
            InputPort<std::string>("mongo_record_name"),
            InputPort<std::string>("mongo_param_name")
        };
    }

    NodeStatus tick() override
{
    auto input_str   = getInput<std::string>("input_value");
    auto record_name = getInput<std::string>("mongo_record_name");
    auto param_name  = getInput<std::string>("mongo_param_name");

    if (!input_str || !record_name || !param_name)
    {
        std::cout << "[MongoValueWriter] missing input" << std::endl;
        return NodeStatus::FAILURE;
    }

    auto client_entry = pool_.acquire();
    auto db = (*client_entry)["rostmsdb"];
    auto collection = db["parameter"];

    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "record_name" << record_name.value();
    auto filter = filter_builder.view();

    bsoncxx::builder::stream::document update_doc;
    update_doc << "$set" << open_document;

    bool type_ok = false;
    const std::string& v = input_str.value();

    // ---------- Blackboard value ----------
    if (v.size() >= 2 && v.front() == '{' && v.back() == '}')
    {
        std::string bb_key = v.substr(1, v.size() - 2);
        auto bb = config().blackboard;

        BT::Any* any = bb->getAny(bb_key);
        if (!any)
        {
            std::cout << "[MongoValueWriter] Blackboard key not found: "<< bb_key << std::endl;
            return NodeStatus::FAILURE;
        }

        try {
            update_doc << param_name.value() << bsoncxx::types::b_bool{any->cast<bool>()};
            type_ok = true;
        } catch (...) {}

        if (!type_ok) {
            try {
                update_doc << param_name.value() << bsoncxx::types::b_int32{any->cast<int>()};
                type_ok = true;
            } catch (...) {}
        }

        if (!type_ok) {
            try {
                update_doc << param_name.value() << bsoncxx::types::b_double{any->cast<double>()};
                type_ok = true;
            } catch (...) {}
        }

        if (!type_ok) {
            try {
                update_doc << param_name.value() << bsoncxx::types::b_utf8{any->cast<std::string>()};
                type_ok = true;
            } catch (...) {}
        }
    }
    // ---------- Literal value ----------
    else
    {
        if (v == "true" || v == "false")
        {
            update_doc << param_name.value() << bsoncxx::types::b_bool{v == "true"};
            type_ok = true;
        }
        else
        {
            try {
                int i = std::stoi(v);
                update_doc << param_name.value() << bsoncxx::types::b_int32{i};
                type_ok = true;
            }
            catch (...) {
                try {
                    double d = std::stod(v);
                    update_doc << param_name.value() << bsoncxx::types::b_double{d};
                    type_ok = true;
                }
                catch (...) {
                    update_doc << param_name.value() << bsoncxx::types::b_utf8{v};
                    type_ok = true;
                }
            }
        }
    }

    if (!type_ok)
    {
        std::cout << "[MongoValueWriter] Unsupported value type" << std::endl;
        return NodeStatus::FAILURE;
    }

    update_doc << close_document;

    update_doc << "$currentDate" << open_document << "timestamps" << true << close_document;

    auto update = update_doc.view();

    try
    {
        auto result = collection.update_one(filter, update);
        if (result && result->matched_count() > 0)
            return NodeStatus::SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cout << "[MongoValueWriter] Mongo exception: " << e.what() << std::endl;
    }

    return NodeStatus::FAILURE;
}


private:
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    mongocxx::pool pool_;
};

#endif
