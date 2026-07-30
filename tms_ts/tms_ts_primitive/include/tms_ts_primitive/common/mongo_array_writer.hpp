// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.

#ifndef MONGO_ARRAY_WRITER_HPP
#define MONGO_ARRAY_WRITER_HPP

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <iostream>

#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/stream/document.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/pool.hpp>
#include <mongocxx/uri.hpp>

#include "behaviortree_cpp_v3/action_node.h"

using namespace BT;
using bsoncxx::builder::stream::open_document;
using bsoncxx::builder::stream::close_document;

class MongoArrayWriter : public SyncActionNode
{
public:

    MongoArrayWriter(const std::string& name,
                     const NodeConfiguration& config)
        : SyncActionNode(name, config),
          pool_(mongocxx::uri{})
    {
        node_ = rclcpp::Node::make_shared("mongo_array_writer");
        spin_thread_ = std::thread([this](){ rclcpp::spin(node_); });
    }

    ~MongoArrayWriter()
    {
        rclcpp::shutdown();

        if(spin_thread_.joinable())
            spin_thread_.join();
    }

    static PortsList providedPorts()
    {
        return{
            InputPort<std::string>("mongo_record_name"),
            InputPort<std::string>("mongo_param_name"),
            InputPort<std::string>("input_value"),
            InputPort<std::string>("operation")     // add(Duplicates are not included) / push (Duplicates included)/ remove
        };
    }

    NodeStatus tick() override
    {
        auto record = getInput<std::string>("mongo_record_name");
        auto param  = getInput<std::string>("mongo_param_name");
        auto value  = getInput<std::string>("input_value");
        auto op     = getInput<std::string>("operation");

        if(!record || !param || !value || !op)
            return NodeStatus::FAILURE;

        //---------------------------------------------
        // Mongo
        //---------------------------------------------
        auto client = pool_.acquire();

        auto db = (*client)["rostmsdb"];
        auto collection = db["parameter"];

        bsoncxx::builder::stream::document filter;
        filter << "record_name" << record.value();

        //---------------------------------------------
        // value
        //---------------------------------------------

        bsoncxx::types::bson_value::value bson_value("");

        std::string v = value.value();

        // Blackboard
        if(!v.empty() && v.front()=='\\' && v.back()=='\\')
        {
            std::string bb_key = v.substr(1,v.size()-2);

            auto bb = config().blackboard;

            bool b;
            int i;
            double d;
            std::string s;

            if(bb->get(bb_key,s))
            {
                if(s=="true" || s=="false")
                    bson_value = bsoncxx::types::b_bool{s=="true"};
                else
                    bson_value = bsoncxx::types::b_utf8{s};
            }
            else if(bb->get(bb_key,b))
            {
                bson_value = bsoncxx::types::b_bool{b};
            }
            else if(bb->get(bb_key,i))
            {
                bson_value = bsoncxx::types::b_int32{i};
            }
            else if(bb->get(bb_key,d))
            {
                bson_value = bsoncxx::types::b_double{d};
            }
            else
            {
                std::cout<<"Unsupported Blackboard type"<<std::endl;
                return NodeStatus::FAILURE;
            }
        }

        // Literal
        else
        {
            if(v.size()>=2 && v.front()=='"' && v.back()=='"')
            {
                bson_value = bsoncxx::types::b_utf8{
                    v.substr(1,v.size()-2)};
            }
            else if(v=="true" || v=="false")
            {
                bson_value = bsoncxx::types::b_bool{v=="true"};
            }
            else
            {
                try{
                    int i=std::stoi(v);
                    bson_value=bsoncxx::types::b_int32{i};
                }
                catch(...)
                {
                    try{
                        double d=std::stod(v);
                        bson_value=bsoncxx::types::b_double{d};
                    }
                    catch(...)
                    {
                        bson_value=bsoncxx::types::b_utf8{v};
                    }
                }
            }
        }

        //---------------------------------------------
        // update
        //---------------------------------------------
        bsoncxx::builder::stream::document update;

        if(op.value()=="add")
        {
            update << "$addToSet"
                   << open_document
                   << param.value()
                   << bson_value
                   << close_document;
        }
        else if(op.value()=="push")
        {
            update << "$push"
                   << open_document
                   << param.value()
                   << bson_value
                   << close_document;
        }
        else if(op.value()=="remove")
        {
            update << "$pull"
                   << open_document
                   << param.value()
                   << bson_value
                   << close_document;
        }
        else
        {
            std::cout<<"Unknown operation : "<<op.value()<<std::endl;
            return NodeStatus::FAILURE;
        }

        update
            << "$currentDate"
            << open_document
            << "timestamps"
            << true
            << close_document;

        try
        {
            auto result =
                collection.update_one(filter.view(),
                                      update.view());

            if(result && result->matched_count()>0)
                return NodeStatus::SUCCESS;
        }
        catch(const std::exception& e)
        {
            std::cout<<e.what()<<std::endl;
        }

        return NodeStatus::FAILURE;
    }

private:

    rclcpp::Node::SharedPtr node_;

    std::thread spin_thread_;

    mongocxx::pool pool_;
};

#endif