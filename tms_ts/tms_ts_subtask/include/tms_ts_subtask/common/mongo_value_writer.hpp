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

using namespace BT;

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

        mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
        mongocxx::database db = client["rostmsdb"];
        mongocxx::collection collection = db["parameter"];

        bsoncxx::builder::stream::document filter_builder;
        filter_builder << "record_name" << record_name.value();
        auto filter = filter_builder.view();

        bsoncxx::builder::stream::document update_doc;
        update_doc << "$set" << update_builder.view();
        auto update = update_doc.view();

        try 
        {
            auto result = collection.update_one(filter, update);
            if (result)
            {
                std::cout << "[MongoValueWriter] Successfully updated the document with record_name [" << record_name.value() << "] and set [" << param_name.value() << "] to [" << input_value_str.value() << "]" << std::endl;
                return NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "[MongoValueWriter] Failed to update the document with record_name [" << record_name.value() << "]." << std::endl;
                return NodeStatus::FAILURE;
            }
        }
        catch (const std::exception& e)
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
