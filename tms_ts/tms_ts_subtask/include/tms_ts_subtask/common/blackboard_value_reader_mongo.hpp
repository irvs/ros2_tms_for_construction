#ifndef BLACKBOARD_VALUE_READER_MONGO_NODE_HPP
#define BLACKBOARD_VALUE_READER_MONGO_NODE_HPP

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

class BlackboardValueReaderMongo : public SyncActionNode
{
public:
    BlackboardValueReaderMongo(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config), pool_(mongocxx::uri{})
    {
        node_ = rclcpp::Node::make_shared("blackboard_value_reader_mongo");
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~BlackboardValueReaderMongo()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("output_port"), InputPort<std::string>("mongo_record_name"), InputPort<std::string>("mongo_param_name") };
    }

    NodeStatus tick() override
    {
        Optional<std::string> key1 = getInput<std::string>("output_port");
        Optional<std::string> key2 = getInput<std::string>("mongo_record_name");
        Optional<std::string> key3 = getInput<std::string>("mongo_param_name");
        if (!key1 || !key2 || !key3)
        {
            std::cout << "[BlackboardValueReaderMongo] missing required input. Please fill key or value parameters." << std::endl;
            return NodeStatus::FAILURE;
        }

        mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
        mongocxx::database db = client["rostmsdb"];
        mongocxx::collection collection = db["parameter"];
        bsoncxx::builder::stream::document filter_builder;
        filter_builder << "record_name" << key2.value();
        auto filter = filter_builder.view();
        auto doc = collection.find_one(filter);

        if (!doc)
        {
            std::cout << "[BlackboardValueReaderMongo]  Couldn't find parameter data containing " << key2.value() << " as record_name in parameter collection" << std::endl;
            return NodeStatus::FAILURE;
        }

        try
        {
            auto value = doc->view()[key3.value()];
            switch (value.type())
            {
                case bsoncxx::type::k_utf8:
                    config().blackboard->set(key1.value(), value.get_utf8().value.to_string());
                    std::cout << "[BlackboardValueReaderMongo]  Stored blackboard parameter [" << key1.value() << "] : " << value.get_utf8().value.to_string() << std::endl;
                    break;
                case bsoncxx::type::k_int32:
                    config().blackboard->set(key1.value(), std::to_string(value.get_int32().value));
                    std::cout << "[BlackboardValueReaderMongo]  Stored blackboard parameter [" << key1.value() << "] : " << value.get_int32().value << std::endl;
                    break;
                case bsoncxx::type::k_int64:
                    config().blackboard->set(key1.value(), std::to_string(value.get_int64().value));
                    std::cout << "[BlackboardValueReaderMongo]  Stored blackboard parameter [" << key1.value() << "] : " << value.get_int64().value << std::endl;
                    break;
                case bsoncxx::type::k_double:
                    config().blackboard->set(key1.value(), std::to_string(value.get_double().value));
                    std::cout << "[BlackboardValueReaderMongo]  Stored blackboard parameter [" << key1.value() << "] : " << value.get_double().value << std::endl;
                    break;
                case bsoncxx::type::k_bool:
                    config().blackboard->set(key1.value(), value.get_bool().value ? "true" : "false");
                    std::cout << "[BlackboardValueReaderMongo]  Stored blackboard parameter [" << key1.value() << "] : " << (value.get_bool().value ? "true" : "false") << std::endl;
                    break;
                default:
                    std::cout << "[BlackboardValueReaderMongo]  Unsupported BSON type: " << bsoncxx::to_string(value.type()) << std::endl;
                    return NodeStatus::FAILURE;
            }

            return NodeStatus::SUCCESS;
        }
        catch (const std::exception& e)
        {
            std::cout << "[BlackboardValueReaderMongo]  Exception caught: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    mongocxx::pool pool_; // Create a pool of connections.
};

#endif
