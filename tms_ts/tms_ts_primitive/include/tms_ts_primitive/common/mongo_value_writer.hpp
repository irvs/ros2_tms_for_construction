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
#include <string>

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
        spin_thread_ = std::thread([this](){rclcpp::spin(node_);});
    }


    ~MongoValueWriter()
    {
        if (spin_thread_.joinable())
        {
            rclcpp::shutdown();
            spin_thread_.join();
        }
    }


    static PortsList providedPorts()
    {
        return
        {
            InputPort<std::string>("input_value"),
            InputPort<std::string>("input_type"),// input_type は任意, 指定されていなければ元の自動型判定を使用
            InputPort<std::string>("mongo_record_name"),
            InputPort<std::string>("mongo_param_name")
        };
    }


    NodeStatus tick() override
    {
        // ============================================================
        // Input
        // ============================================================

        auto input_str = getInput<std::string>("input_value");
        auto input_type = getInput<std::string>("input_type");
        auto record_name = getInput<std::string>("mongo_record_name");
        auto param_name = getInput<std::string>("mongo_param_name");


        // ============================================================
        // Required input check
        // ============================================================

        if (!input_str)
        {
            std::cout
                << "[MongoValueWriter] "
                << "input_value is missing: "
                << input_str.error()
                << std::endl;

            return NodeStatus::FAILURE;
        }

        if (!record_name)
        {
            std::cout
                << "[MongoValueWriter] "
                << "mongo_record_name is missing: "
                << record_name.error()
                << std::endl;

            return NodeStatus::FAILURE;
        }

        if (!param_name)
        {
            std::cout
                << "[MongoValueWriter] "
                << "mongo_param_name is missing: "
                << param_name.error()
                << std::endl;

            return NodeStatus::FAILURE;
        }


        // ============================================================
        // input_type
        //
        // input_type がない場合:
        //     type_specified = false
        //
        // input_type="" の場合:
        //     type_specified = false
        //
        // input_type="double" などの場合:
        //     type_specified = true
        // ============================================================

        bool type_specified = false;
        std::string type;

        if (input_type)
        {
            type = input_type.value();

            if (!type.empty())
            {
                type_specified = true;
            }
        }


        const std::string v =
            input_str.value();

        const std::string record =
            record_name.value();

        const std::string param =
            param_name.value();


        std::cout
            << "[MongoValueWriter] input_value is: "
            << v
            << std::endl;


        if (type_specified)
        {
            std::cout
                << "[MongoValueWriter] input_type is: "
                << type
                << std::endl;
        }
        else
        {
            std::cout
                << "[MongoValueWriter] "
                << "input_type is not specified. "
                << "Using automatic type detection."
                << std::endl;
        }


        // ============================================================
        // input_type が指定されている場合のチェック
        // ============================================================

        if (type_specified)
        {
            if (type != "int" &&
                type != "double" &&
                type != "bool" &&
                type != "string")
            {
                std::cout
                    << "[MongoValueWriter] "
                    << "Unsupported input_type: "
                    << type
                    << std::endl;

                return NodeStatus::FAILURE;
            }
        }


        // ============================================================
        // MongoDB
        // ============================================================

        auto client_entry =
            pool_.acquire();

        auto db =
            (*client_entry)["rostmsdb"];

        auto collection =
            db["parameter"];


        // ============================================================
        // Filter
        // ============================================================

        bsoncxx::builder::stream::document filter_builder;

        filter_builder
            << "record_name"
            << record;

        auto filter =
            filter_builder.view();


        // ============================================================
        // Update
        // ============================================================

        bsoncxx::builder::stream::document update_doc;

        update_doc
            << "$set"
            << open_document;


        bool type_ok = false;


        // ============================================================
        // Blackboard value
        // ============================================================

        if (v.size() >= 2 &&
            v.front() == '\\' &&
            v.back() == '\\')
        {
            std::string bb_key =
                v.substr(
                    1,
                    v.size() - 2);


            auto bb =
                config().blackboard;


            std::cout
                << "[MongoValueWriter] "
                << "search black board by: "
                << bb_key
                << std::endl;


            // ========================================================
            // input_type が指定されている場合
            // ========================================================

            if (type_specified)
            {
                // ====================================================
                // DOUBLE
                // ====================================================

                if (type == "double")
                {
                    double value;

                    if (bb->get(bb_key, value))
                    {
                        std::cout
                            << "[MongoValueWriter] "
                            << "Blackboard double: "
                            << value
                            << std::endl;

                        update_doc
                            << param
                            << bsoncxx::types::b_double{value};

                        type_ok = true;
                    }
                    else
                    {
                        // int -> double
                        int int_value;

                        if (bb->get(bb_key, int_value))
                        {
                            double value =
                                static_cast<double>(int_value);

                            std::cout
                                << "[MongoValueWriter] "
                                << "Blackboard int converted "
                                << "to double: "
                                << value
                                << std::endl;

                            update_doc
                                << param
                                << bsoncxx::types::b_double{value};

                            type_ok = true;
                        }
                    }
                }


                // ====================================================
                // INT
                // ====================================================

                else if (type == "int")
                {
                    int value;

                    if (bb->get(bb_key, value))
                    {
                        std::cout
                            << "[MongoValueWriter] "
                            << "Blackboard int: "
                            << value
                            << std::endl;

                        update_doc
                            << param
                            << bsoncxx::types::b_int32{value};

                        type_ok = true;
                    }
                }


                // ====================================================
                // BOOL
                // ====================================================

                else if (type == "bool")
                {
                    bool value;

                    if (bb->get(bb_key, value))
                    {
                        std::cout
                            << "[MongoValueWriter] "
                            << "Blackboard bool: "
                            << value
                            << std::endl;

                        update_doc
                            << param
                            << bsoncxx::types::b_bool{value};

                        type_ok = true;
                    }
                }


                // ====================================================
                // STRING
                // ====================================================

                else if (type == "string")
                {
                    std::string value;

                    if (bb->get(bb_key, value))
                    {
                        std::cout
                            << "[MongoValueWriter] "
                            << "Blackboard string: "
                            << value
                            << std::endl;

                        update_doc
                            << param
                            << bsoncxx::types::b_utf8{value};

                        type_ok = true;
                    }
                }


                // ====================================================
                // 指定型で取得できなかった
                // ====================================================

                if (!type_ok)
                {
                    std::cout
                        << "[MongoValueWriter] "
                        << "Cannot get Blackboard value '"
                        << bb_key
                        << "' as type '"
                        << type
                        << "'"
                        << std::endl;

                    return NodeStatus::FAILURE;
                }
            }


            // ========================================================
            // input_type が指定されていない場合
            //
            // 元の MongoValueWriter と同じ処理
            // ========================================================

            else
            {
                bool b;
                int i;
                double d;
                std::string s;


                // ----------------------------------------------------
                // string
                // ----------------------------------------------------

                if (bb->get(bb_key, s))
                {
                    std::cout
                        << "[MongoValueWriter] "
                        << "Blackboard string: "
                        << s
                        << std::endl;


                    if (s == "true" ||
                        s == "false")
                    {
                        update_doc
                            << param
                            << bsoncxx::types::b_bool{
                                   s == "true"
                               };
                    }
                    else
                    {
                        update_doc
                            << param
                            << bsoncxx::types::b_utf8{s};
                    }

                    type_ok = true;
                }


                // ----------------------------------------------------
                // bool
                // ----------------------------------------------------

                else if (bb->get(bb_key, b))
                {
                    update_doc
                        << param
                        << bsoncxx::types::b_bool{b};

                    type_ok = true;
                }


                // ----------------------------------------------------
                // int
                // ----------------------------------------------------

                else if (bb->get(bb_key, i))
                {
                    update_doc
                        << param
                        << bsoncxx::types::b_int32{i};

                    type_ok = true;
                }


                // ----------------------------------------------------
                // double
                // ----------------------------------------------------

                else if (bb->get(bb_key, d))
                {
                    update_doc
                        << param
                        << bsoncxx::types::b_double{d};

                    type_ok = true;
                }


                else
                {
                    std::cout
                        << "[MongoValueWriter] "
                        << "Unsupported Blackboard type"
                        << std::endl;

                    return NodeStatus::FAILURE;
                }
            }
        }


        // ============================================================
        // Literal value
        // ============================================================

        else
        {
            // ========================================================
            // input_type が指定されている場合
            // ========================================================

            if (type_specified)
            {
                // ====================================================
                // DOUBLE
                // ====================================================

                if (type == "double")
                {
                    try
                    {
                        double value =
                            std::stod(v);

                        update_doc
                            << param
                            << bsoncxx::types::b_double{value};

                        std::cout
                            << "[MongoValueWriter] "
                            << "Literal double: "
                            << value
                            << std::endl;

                        type_ok = true;
                    }
                    catch (...)
                    {
                        std::cout
                            << "[MongoValueWriter] "
                            << "Failed to convert '"
                            << v
                            << "' to double"
                            << std::endl;

                        return NodeStatus::FAILURE;
                    }
                }


                // ====================================================
                // INT
                // ====================================================

                else if (type == "int")
                {
                    try
                    {
                        int value =
                            std::stoi(v);

                        update_doc
                            << param
                            << bsoncxx::types::b_int32{value};

                        std::cout
                            << "[MongoValueWriter] "
                            << "Literal int: "
                            << value
                            << std::endl;

                        type_ok = true;
                    }
                    catch (...)
                    {
                        std::cout
                            << "[MongoValueWriter] "
                            << "Failed to convert '"
                            << v
                            << "' to int"
                            << std::endl;

                        return NodeStatus::FAILURE;
                    }
                }


                // ====================================================
                // BOOL
                // ====================================================

                else if (type == "bool")
                {
                    if (v == "true")
                    {
                        update_doc
                            << param
                            << bsoncxx::types::b_bool{true};

                        type_ok = true;
                    }
                    else if (v == "false")
                    {
                        update_doc
                            << param
                            << bsoncxx::types::b_bool{false};

                        type_ok = true;
                    }
                    else
                    {
                        std::cout
                            << "[MongoValueWriter] "
                            << "Invalid bool literal: "
                            << v
                            << std::endl;

                        return NodeStatus::FAILURE;
                    }
                }


                // ====================================================
                // STRING
                // ====================================================

                else if (type == "string")
                {
                    std::string value = v;


                    if (value.size() >= 2 &&
                        value.front() == '"' &&
                        value.back() == '"')
                    {
                        value =
                            value.substr(
                                1,
                                value.size() - 2);
                    }


                    update_doc
                        << param
                        << bsoncxx::types::b_utf8{value};

                    std::cout
                        << "[MongoValueWriter] "
                        << "Literal string: "
                        << value
                        << std::endl;

                    type_ok = true;
                }
            }


            // ========================================================
            // input_type が指定されていない場合
            //
            // 元の MongoValueWriter と同じ処理
            // ========================================================

            else
            {
                // ----------------------------------------------------
                // 強制 string
                // ----------------------------------------------------

                if (v.size() >= 2 &&
                    v.front() == '"' &&
                    v.back() == '"')
                {
                    std::string unquoted =
                        v.substr(
                            1,
                            v.size() - 2);

                    update_doc
                        << param
                        << bsoncxx::types::b_utf8{
                               unquoted
                           };

                    type_ok = true;
                }


                // ----------------------------------------------------
                // bool
                // ----------------------------------------------------

                else if (v == "true" ||
                         v == "false")
                {
                    update_doc
                        << param
                        << bsoncxx::types::b_bool{
                               v == "true"
                           };

                    type_ok = true;
                }


                // ----------------------------------------------------
                // int -> double -> string
                // ----------------------------------------------------

                else
                {
                    try
                    {
                        int i =
                            std::stoi(v);

                        update_doc
                            << param
                            << bsoncxx::types::b_int32{i};

                        type_ok = true;
                    }
                    catch (...)
                    {
                        try
                        {
                            double d =
                                std::stod(v);

                            update_doc
                                << param
                                << bsoncxx::types::b_double{d};

                            type_ok = true;
                        }
                        catch (...)
                        {
                            update_doc
                                << param
                                << bsoncxx::types::b_utf8{v};

                            type_ok = true;
                        }
                    }
                }
            }
        }


        // ============================================================
        // Type check
        // ============================================================

        if (!type_ok)
        {
            std::cout
                << "[MongoValueWriter] "
                << "Unsupported value type"
                << std::endl;

            return NodeStatus::FAILURE;
        }


        // ============================================================
        // Finish $set
        // ============================================================

        update_doc
            << close_document;


        // ============================================================
        // Update timestamp
        // ============================================================

        update_doc
            << "$currentDate"
            << open_document
            << "timestamps"
            << true
            << close_document;


        auto update =
            update_doc.view();


        // ============================================================
        // MongoDB update
        // ============================================================

        try
        {
            auto result =
                collection.update_one(
                    filter,
                    update);


            if (result &&
                result->matched_count() > 0)
            {
                std::cout
                    << "[MongoValueWriter] "
                    << "MongoDB update succeeded."
                    << std::endl;

                return NodeStatus::SUCCESS;
            }


            std::cout
                << "[MongoValueWriter] "
                << "No matching record found: "
                << record
                << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout
                << "[MongoValueWriter] "
                << "Mongo exception: "
                << e.what()
                << std::endl;
        }


        return NodeStatus::FAILURE;
    }


private:

    rclcpp::Node::SharedPtr node_;

    std::thread spin_thread_;

    mongocxx::pool pool_;
};


#endif