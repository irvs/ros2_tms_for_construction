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
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/pool.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <mongocxx/logger.hpp>
#include <iostream>

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
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
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
        Optional<std::string> input_value_str = getInput<std::string>("input_value");
        Optional<std::string> record_name = getInput<std::string>("mongo_record_name");
        Optional<std::string> param_name = getInput<std::string>("mongo_param_name");
        if (!input_value_str || !record_name || !param_name)
        {
            std::cout << "[MongoValueWriter] missing required input. Please fill key or value parameters." << std::endl;
            return NodeStatus::FAILURE;
        }
        bsoncxx::builder::stream::document update_builder;
        try {
            if (input_value_str.value() == "true" || input_value_str.value() == "false") {
                update_builder << param_name.value() << bsoncxx::types::b_bool{input_value_str.value() == "true"};
            } else if (input_value_str.value().find('.') != std::string::npos) {
                update_builder << param_name.value() << bsoncxx::types::b_double{std::stod(input_value_str.value())};
            } else {
                try {
                    update_builder << param_name.value() << bsoncxx::types::b_int32{std::stoi(input_value_str.value())};
                } catch (const std::invalid_argument&) {
                    update_builder << param_name.value() << bsoncxx::types::b_utf8{input_value_str.value()};
                }
            }
        } catch (const std::exception& e) {
            std::cout << "[MongoValueWriter] Error converting input_value: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
        auto client_entry = pool_.acquire();
        auto db = (*client_entry)["rostmsdb"];
        auto collection = db["parameter"];
        bsoncxx::builder::stream::document filter_builder;
        filter_builder << "record_name" << record_name.value();
        auto filter = filter_builder.view();
        bsoncxx::builder::stream::document update_doc;
        update_doc << "$set" << update_builder.view()  << "$currentDate" << open_document << "timestamps" << true << close_document;
        auto update = update_doc.view();
        try
        {
          const int kMaxNotFoundRetries = 5;
          int not_found_count = 0;
          while (true)
          {
            auto result = collection.update_one(filter, update);
            if (!result)
            {
              std::cout << "[MongoValueWriter] update_one returned null result." << std::endl;
              return NodeStatus::FAILURE;
            }
            auto matched = result->matched_count();
            auto modified = result->modified_count();
            if (matched > 0 && modified > 0)
            {
              std::cout << "[MongoValueWriter] Successfully updated record_name [" << record_name.value()
                        << "], set [" << param_name.value() << "] to [" << input_value_str.value() << "]" << std::endl;
              return NodeStatus::SUCCESS;
            }
            not_found_count++;
            if (not_found_count >= kMaxNotFoundRetries)
            {
              std::cout << "[MongoValueWriter] Target record_name [" << record_name.value()
                        << "] not found after " << not_found_count << " retries. Giving up." << std::endl;
              return NodeStatus::FAILURE;
            }
            std::cout << "[MongoValueWriter] Target record_name [" << record_name.value()
                      << "] not found (retry " << not_found_count << "/" << kMaxNotFoundRetries << "). Retrying..."
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }
        catch (const std::exception &e)
        {
          std::cout << "[MongoValueWriter] Exception caught: " << e.what() << std::endl;
          return NodeStatus::FAILURE;
        }
      }
private:
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    mongocxx::pool pool_;
};
#endif