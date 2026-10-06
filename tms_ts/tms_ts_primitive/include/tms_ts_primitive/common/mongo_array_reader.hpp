// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MONGO_ARRAY_READER_HPP
#define MONGO_ARRAY_READER_HPP

#include "rclcpp/rclcpp.hpp"

#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <regex>

#include <bsoncxx/types.hpp>
#include <bsoncxx/json.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/pool.hpp>
#include <mongocxx/uri.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;


class MongoArrayReader : public SyncActionNode
{
public:

    MongoArrayReader(
        const std::string& name,
        const NodeConfiguration& config)
        : SyncActionNode(name, config),
          pool_(mongocxx::uri{})
    {
        node_ =
            rclcpp::Node::make_shared("mongo_array_reader");

        spin_thread_ =
            std::thread([this]()
            {
                rclcpp::spin(node_);
            });
    }


    ~MongoArrayReader()
    {
        rclcpp::shutdown();

        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }


    static PortsList providedPorts()
    {
        return
        {
            InputPort<std::string>("output_port"),
            InputPort<std::string>("mongo_record_name"),
            InputPort<std::string>("mongo_param_name"),

            // "array" : array全体
            // "len"   : arrayの長さ
            // "[n]"   : n番目の要素
            // "[-n]"  : 後ろからn番目の要素
            InputPort<std::string>("output_type")
        };
    }


    NodeStatus tick() override
    {
        //---------------------------------------------
        // Input
        //---------------------------------------------

        auto output_port =
            getInput<std::string>("output_port");

        auto record_name =
            getInput<std::string>("mongo_record_name");

        auto param_name =
            getInput<std::string>("mongo_param_name");

        auto output_type =
            getInput<std::string>("output_type");


        if (!output_port ||
            !record_name ||
            !param_name ||
            !output_type)
        {
            std::cout
                << "[MongoArrayReader] "
                << "missing required input."
                << std::endl;

            return NodeStatus::FAILURE;
        }


        //---------------------------------------------
        // MongoDB
        //---------------------------------------------

        try
        {
            auto client = pool_.acquire();

            auto db =
                (*client)["rostmsdb"];

            auto collection =
                db["parameter"];


            bsoncxx::builder::stream::document filter;

            filter
                << "record_name"
                << record_name.value();


            auto doc =
                collection.find_one(filter.view());


            if (!doc)
            {
                std::cout
                    << "[MongoArrayReader] "
                    << "Couldn't find parameter data containing "
                    << record_name.value()
                    << " as record_name in parameter collection"
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            //---------------------------------------------
            // Parameter
            //---------------------------------------------

            auto element =
                doc->view()[param_name.value()];


            if (!element)
            {
                std::cout
                    << "[MongoArrayReader] "
                    << "Parameter ["
                    << param_name.value()
                    << "] was not found."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            //---------------------------------------------
            // Check BSON type
            //---------------------------------------------

            if (element.type() != bsoncxx::type::k_array)
            {
                std::cout
                    << "[MongoArrayReader] "
                    << "Parameter ["
                    << param_name.value()
                    << "] is not an array. "
                    << "BSON type: "
                    << bsoncxx::to_string(element.type())
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            //---------------------------------------------
            // Read array
            //---------------------------------------------

            auto array =
                element.get_array().value;


            std::vector<std::string> string_array;

            for (auto&& value : array)
            {
                switch (value.type())
                {
                    //-------------------------------------
                    // string
                    //-------------------------------------

                    case bsoncxx::type::k_utf8:
                    {
                        string_array.push_back(
                            value.get_utf8().value.to_string());

                        break;
                    }


                    //-------------------------------------
                    // int32
                    //-------------------------------------

                    case bsoncxx::type::k_int32:
                    {
                        string_array.push_back(
                            std::to_string(
                                value.get_int32().value));

                        break;
                    }


                    //-------------------------------------
                    // int64
                    //-------------------------------------

                    case bsoncxx::type::k_int64:
                    {
                        string_array.push_back(
                            std::to_string(
                                value.get_int64().value));

                        break;
                    }


                    //-------------------------------------
                    // double
                    //-------------------------------------

                    case bsoncxx::type::k_double:
                    {
                        string_array.push_back(
                            std::to_string(
                                value.get_double().value));

                        break;
                    }


                    //-------------------------------------
                    // bool
                    //-------------------------------------

                    case bsoncxx::type::k_bool:
                    {
                        string_array.push_back(
                            value.get_bool().value
                                ? "true"
                                : "false");

                        break;
                    }


                    //-------------------------------------
                    // unsupported
                    //-------------------------------------

                    default:
                    {
                        std::cout
                            << "[MongoArrayReader] "
                            << "Unsupported BSON type in array: "
                            << bsoncxx::to_string(
                                value.type())
                            << std::endl;

                        return NodeStatus::FAILURE;
                    }
                }
            }


            //---------------------------------------------
            // output_type
            //---------------------------------------------

            const std::string type =
                output_type.value();


            //---------------------------------------------
            // array
            //---------------------------------------------

            if (type == "array")
            {
                config().blackboard->set(
                    output_port.value(),
                    string_array);


                std::cout
                    << "[MongoArrayReader] "
                    << "Stored array ["
                    << output_port.value()
                    << "] : [";

                for (size_t i = 0;
                     i < string_array.size();
                     ++i)
                {
                    std::cout
                        << string_array[i];

                    if (i + 1 < string_array.size())
                    {
                        std::cout << ", ";
                    }
                }

                std::cout
                    << "]"
                    << std::endl;


                return NodeStatus::SUCCESS;
            }


            //---------------------------------------------
            // len
            //---------------------------------------------

            if (type == "len")
            {
                int length =
                    static_cast<int>(
                        string_array.size());


                config().blackboard->set(
                    output_port.value(),
                    length);


                std::cout
                    << "[MongoArrayReader] "
                    << "Stored array length ["
                    << output_port.value()
                    << "] : "
                    << length
                    << std::endl;


                return NodeStatus::SUCCESS;
            }


            //---------------------------------------------
            // [n] / [-n]
            //---------------------------------------------

            if (type.size() >= 3 &&
                type.front() == '[' &&
                type.back() == ']')
            {
                std::string index_string =
                    type.substr(
                        1,
                        type.size() - 2);


                int index;


                //-----------------------------------------
                // Convert index
                //-----------------------------------------

                try
                {
                    size_t pos = 0;

                    index =
                        std::stoi(
                            index_string,
                            &pos);

                    if (pos != index_string.size())
                    {
                        throw std::invalid_argument(
                            "invalid index");
                    }
                }
                catch (const std::exception&)
                {
                    std::cout
                        << "[MongoArrayReader] "
                        << "Invalid output_type: "
                        << type
                        << std::endl;

                    return NodeStatus::FAILURE;
                }


                //-----------------------------------------
                // Empty array
                //-----------------------------------------

                if (string_array.empty())
                {
                    std::cout
                        << "[MongoArrayReader] "
                        << "Cannot access element ["
                        << index
                        << "] because array is empty."
                        << std::endl;

                    return NodeStatus::FAILURE;
                }


                //-----------------------------------------
                // Negative index
                //-----------------------------------------

                if (index < 0)
                {
                    index =
                        static_cast<int>(
                            string_array.size())
                        + index;
                }


                //-----------------------------------------
                // Range check
                //-----------------------------------------

                if (index < 0 ||
                    index >=
                        static_cast<int>(
                            string_array.size()))
                {
                    std::cout
                        << "[MongoArrayReader] "
                        << "Array index out of range: "
                        << type
                        << " (array size = "
                        << string_array.size()
                        << ")"
                        << std::endl;

                    return NodeStatus::FAILURE;
                }


                //-----------------------------------------
                // Store element
                //-----------------------------------------

                const std::string& value =
                    string_array[index];


                config().blackboard->set(
                    output_port.value(),
                    value);


                std::cout
                    << "[MongoArrayReader] "
                    << "Stored array element ["
                    << output_port.value()
                    << "] : "
                    << value
                    << " (index = "
                    << index
                    << ")"
                    << std::endl;


                return NodeStatus::SUCCESS;
            }


            //---------------------------------------------
            // Unknown output_type
            //---------------------------------------------

            std::cout
                << "[MongoArrayReader] "
                << "Unknown output_type: "
                << type
                << std::endl;

            std::cout
                << "[MongoArrayReader] "
                << "Supported output_type: "
                << "\"array\", \"len\", \"[n]\", \"[-n]\""
                << std::endl;

            return NodeStatus::FAILURE;
        }
        catch (const std::exception& e)
        {
            std::cout
                << "[MongoArrayReader] "
                << "Exception caught: "
                << e.what()
                << std::endl;

            return NodeStatus::FAILURE;
        }
    }


private:

    rclcpp::Node::SharedPtr node_;

    std::thread spin_thread_;

    mongocxx::pool pool_;
};


#endif
