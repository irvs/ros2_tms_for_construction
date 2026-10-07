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

#ifndef BULLDOZER_PATH_CREATER_HPP
#define BULLDOZER_PATH_CREATER_HPP

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <stdexcept>

#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/pool.hpp>
#include <mongocxx/options/update.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

using bsoncxx::builder::stream::open_document;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::close_array;


// ============================================================
// Point
// ============================================================

struct BulldozerPoint2D
{
    double x;
    double y;
};


// ============================================================
// Quaternion
// ============================================================

struct BulldozerQuaternion
{
    double x;
    double y;
    double z;
    double w;
};


// ============================================================
// Waypoint
// ============================================================

struct BulldozerWaypoint
{
    std::array<double, 2> x;
    std::array<double, 2> y;
    std::array<double, 2> z;

    std::array<double, 2> qx;
    std::array<double, 2> qy;
    std::array<double, 2> qz;
    std::array<double, 2> qw;
};


// ============================================================
// BulldozerPathCreater
// ============================================================

class BulldozerPathCreater : public SyncActionNode
{
public:

    BulldozerPathCreater(
        const std::string& name,
        const NodeConfiguration& config)
        : SyncActionNode(name, config),
          pool_(mongocxx::uri{})
    {
    }


    // ========================================================
    // Ports
    //
    // data1:
    //   現在のブロックのrecord_name
    //
    // data2:
    //   dynamic dataのrecord_name
    // ========================================================

    static PortsList providedPorts()
    {
        return
        {
            InputPort<std::string>("data1"),
            InputPort<std::string>("data2")
        };
    }


    // ========================================================
    // tick
    // ========================================================

    NodeStatus tick() override
    {
        // ====================================================
        // Input
        // ====================================================

        auto data1_name =
            getInput<std::string>("data1");

        auto data2_name =
            getInput<std::string>("data2");


        if (!data1_name)
        {
            std::cout
                << "[BulldozerPathCreater] "
                << "data1 is missing."
                << std::endl;

            return NodeStatus::FAILURE;
        }


        if (!data2_name)
        {
            std::cout
                << "[BulldozerPathCreater] "
                << "data2 is missing."
                << std::endl;

            return NodeStatus::FAILURE;
        }


        const std::string data1_record =
            data1_name.value();

        const std::string data2_record =
            data2_name.value();


        // ====================================================
        // MongoDB
        // ====================================================

        auto client_entry =
            pool_.acquire();

        auto db =
            (*client_entry)["rostmsdb"];

        auto collection =
            db["parameter"];


        // ====================================================
        // Read data1
        // ====================================================

        auto data1_doc =
            findRecord(
                collection,
                data1_record);


        if (!data1_doc)
        {
            std::cout
                << "[BulldozerPathCreater] "
                << "Could not find data1 record: "
                << data1_record
                << std::endl;

            return NodeStatus::FAILURE;
        }


        // ====================================================
        // Read data2
        // ====================================================

        auto data2_doc =
            findRecord(
                collection,
                data2_record);


        if (!data2_doc)
        {
            std::cout
                << "[BulldozerPathCreater] "
                << "Could not find data2 record: "
                << data2_record
                << std::endl;

            return NodeStatus::FAILURE;
        }


        try
        {
            auto data1_view =
                data1_doc->view();

            auto data2_view =
                data2_doc->view();


            // =================================================
            // 1. data1.block_size
            // =================================================

            auto block_size_element =
                data1_view["block_size"];


            if (!block_size_element ||
                block_size_element.type() !=
                    bsoncxx::type::k_document)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "data1.block_size is missing."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            auto block_size =
                block_size_element
                    .get_document()
                    .view();


            const double wx =
                getNumber(
                    block_size,
                    "x");


            const double wy =
                getNumber(
                    block_size,
                    "y");


            // =================================================
            // 2. data1.vector = L
            // =================================================

            auto vector_element =
                data1_view["vector"];


            if (!vector_element ||
                vector_element.type() !=
                    bsoncxx::type::k_document)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "data1.vector is missing."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            auto vector =
                vector_element
                    .get_document()
                    .view();


            BulldozerPoint2D L;

            L.x =
                getNumber(
                    vector,
                    "x");

            L.y =
                getNumber(
                    vector,
                    "y");


            // -------------------------------------------------
            // Normalize L
            // -------------------------------------------------

            const double l_norm =
                std::sqrt(
                    L.x * L.x +
                    L.y * L.y);


            if (l_norm < 1.0e-9)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "data1.vector has zero length."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            L.x /= l_norm;
            L.y /= l_norm;


            // =================================================
            // 3. data2.w_blade
            // =================================================

            const double w_blade =
                getNumber(
                    data2_view,
                    "w_blade");


            // =================================================
            // 4. data2.offset
            // =================================================

            const double back_offset =
                getNumber(
                    data2_view,
                    "back_offset");
                
            const double blade_offset =
                getNumber(
                    data2_view,
                    "blade_offset");


            // =================================================
            // 5. LATEST_LEVELED_BLOCK
            // =================================================

            auto current_block_element =
                data2_view[
                    "LATEST_LEVELED_BLOCK"];


            if (!current_block_element ||
                current_block_element.type() !=
                    bsoncxx::type::k_utf8)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "LATEST_LEVELED_BLOCK is missing."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            const std::string current_block =
                current_block_element
                    .get_utf8()
                    .value
                    .to_string();


            // =================================================
            // data1と現在ブロックの一致確認
            // =================================================

            if (data1_record != current_block)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "data1 record ["
                    << data1_record
                    << "] does not match "
                    << "LATEST_LEVELED_BLOCK ["
                    << current_block
                    << "]."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            // =================================================
            // 6. RELEACED_BLOCK
            //
            // block_n + 1を取得
            // =================================================

            auto released_element =
                data2_view[
                    "RELEACED_BLOCK"];


            if (!released_element ||
                released_element.type() !=
                    bsoncxx::type::k_array)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "RELEACED_BLOCK is missing."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            std::vector<std::string>
                released_blocks;


            for (const auto& element :
                 released_element
                     .get_array()
                     .value)
            {
                if (element.type() ==
                    bsoncxx::type::k_utf8)
                {
                    released_blocks.push_back(
                        element
                            .get_utf8()
                            .value
                            .to_string());
                }
            }


            // =================================================
            // 現在ブロックのindex
            // =================================================

            int current_index = -1;


            for (std::size_t i = 0;
                 i < released_blocks.size();
                 ++i)
            {
                if (released_blocks[i] ==
                    current_block)
                {
                    current_index =
                        static_cast<int>(i);

                    break;
                }
            }


            if (current_index < 0)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "Current block ["
                    << current_block
                    << "] is not in "
                    << "RELEACED_BLOCK."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            // =================================================
            // block_n + 1
            // =================================================

            if (current_index + 1 >=
                static_cast<int>(
                    released_blocks.size()))
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "There is no next block after ["
                    << current_block
                    << "]."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            const std::string next_block =
                released_blocks[
                    current_index + 1];


            // =================================================
            // 7. block_n+1をMongoDBから取得
            // =================================================

            auto next_block_doc =
                findRecord(
                    collection,
                    next_block);


            if (!next_block_doc)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "Could not find next block: "
                    << next_block
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            // =================================================
            // 8. block_n center
            // =================================================

            auto current_center_element =
                data1_view[
                    "block_center"];


            if (!current_center_element ||
                current_center_element.type() !=
                    bsoncxx::type::k_document)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "data1.block_center is missing."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            auto current_center =
                current_center_element
                    .get_document()
                    .view();


            BulldozerPoint2D current_point;

            current_point.x =
                getNumber(
                    current_center,
                    "x");

            current_point.y =
                getNumber(
                    current_center,
                    "y");


            // =================================================
            // 9. block_n+1 center
            // =================================================

            auto next_view =
                next_block_doc->view();


            auto next_center_element =
                next_view[
                    "block_center"];


            if (!next_center_element ||
                next_center_element.type() !=
                    bsoncxx::type::k_document)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << next_block
                    << ".block_center is missing."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            auto next_center =
                next_center_element
                    .get_document()
                    .view();


            BulldozerPoint2D next_point;

            next_point.x =
                getNumber(
                    next_center,
                    "x");

            next_point.y =
                getNumber(
                    next_center,
                    "y");


            // =================================================
            // 10. D
            //
            // block_n -> block_n+1
            // =================================================

            BulldozerPoint2D D;

            D.x =
                next_point.x -
                current_point.x;

            D.y =
                next_point.y -
                current_point.y;


            const double d_norm =
                std::sqrt(
                    D.x * D.x +
                    D.y * D.y);


            if (d_norm < 1.0e-9)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "Distance between "
                    << current_block
                    << " and "
                    << next_block
                    << " is zero."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            // normalize D

            D.x /= d_norm;
            D.y /= d_norm;


            // =================================================
            // 11. L+
            //
            // [ 0 -1 ] L
            // [ 1  0 ]
            // =================================================

            BulldozerPoint2D L_plus;

            L_plus.x =
                -L.y;

            L_plus.y =
                 L.x;


            // =================================================
            // 12. L-
            //
            // [ 0  1 ] L
            // [-1  0 ]
            // =================================================

            BulldozerPoint2D L_minus;

            L_minus.x =
                 L.y;

            L_minus.y =
                -L.x;


            // =================================================
            // 13. S
            // =================================================

            const double dot_plus =
                D.x * L_plus.x +
                D.y * L_plus.y;


            const double dot_minus =
                D.x * L_minus.x +
                D.y * L_minus.y;


            BulldozerPoint2D S;


            if (dot_plus > dot_minus)
            {
                S = L_plus;
            }
            else
            {
                S = L_minus;
            }


            // =================================================
            // 14. orig_first
            //
            // (x,y) - S * ((wy-w_blade)/2)
            // =================================================

            const double lateral_offset =
                (wy - w_blade) / 2.0;


            BulldozerPoint2D orig_first;


            orig_first.x =
                current_point.x -
                S.x * lateral_offset;

            orig_first.y =
                current_point.y -
                S.y * lateral_offset;


            // =================================================
            // 15. orig_last
            //
            // (x,y) + S * ((wy-w_blade)/2)
            // =================================================

            BulldozerPoint2D orig_last;


            orig_last.x =
                current_point.x +
                S.x * lateral_offset;

            orig_last.y =
                current_point.y +
                S.y * lateral_offset;


            // =================================================
            // 16. waypoint
            //
            // waypoint1:
            //
            // orig_first - L * back_offset
            // orig_first + L * (wx - blade_offset)
            //
            // waypoint2:
            //
            // center - L * back_offset
            // center + L * (wx - blade_offset)
            //
            // waypoint3:
            //
            // orig_last - L * back_offset
            // orig_last + L * (wx - blade_offset)
            // =================================================

            const BulldozerPoint2D
                wp1_start =
                    subtract(
                        orig_first,
                        multiply(
                            L,
                            back_offset));


            const BulldozerPoint2D
                wp1_end =
                    add(
                        orig_first,
                        multiply(
                            L,
                            wx - blade_offset));


            const BulldozerPoint2D
                wp2_start =
                    subtract(
                        current_point,
                        multiply(
                            L,
                            back_offset));


            const BulldozerPoint2D
                wp2_end =
                    add(
                        current_point,
                        multiply(
                            L,
                            wx - blade_offset));


            const BulldozerPoint2D
                wp3_start =
                    subtract(
                        orig_last,
                        multiply(
                            L,
                            back_offset));


            const BulldozerPoint2D
                wp3_end =
                    add(
                        orig_last,
                        multiply(
                            L,
                            wx - blade_offset));


            // =================================================
            // 17. Quaternion
            //
            // waypointの進行方向をLとする
            // =================================================

            const double yaw =
                std::atan2(
                    L.y,
                    L.x);


            BulldozerQuaternion q;

            q.x = 0.0;
            q.y = 0.0;
            q.z = std::sin(
                yaw / 2.0);
            q.w = std::cos(
                yaw / 2.0);


            // =================================================
            // 18. Waypoint data
            // =================================================

            BulldozerWaypoint waypoint1;

            waypoint1.x =
            {
                wp1_start.x,
                wp1_end.x
            };

            waypoint1.y =
            {
                wp1_start.y,
                wp1_end.y
            };

            waypoint1.z =
            {
                0.0,
                0.0
            };

            waypoint1.qx =
            {
                q.x,
                q.x
            };

            waypoint1.qy =
            {
                q.y,
                q.y
            };

            waypoint1.qz =
            {
                q.z,
                q.z
            };

            waypoint1.qw =
            {
                q.w,
                q.w
            };


            BulldozerWaypoint waypoint2;

            waypoint2.x =
            {
                wp2_start.x,
                wp2_end.x
            };

            waypoint2.y =
            {
                wp2_start.y,
                wp2_end.y
            };

            waypoint2.z =
            {
                0.0,
                0.0
            };

            waypoint2.qx =
            {
                q.x,
                q.x
            };

            waypoint2.qy =
            {
                q.y,
                q.y
            };

            waypoint2.qz =
            {
                q.z,
                q.z
            };

            waypoint2.qw =
            {
                q.w,
                q.w
            };


            BulldozerWaypoint waypoint3;

            waypoint3.x =
            {
                wp3_start.x,
                wp3_end.x
            };

            waypoint3.y =
            {
                wp3_start.y,
                wp3_end.y
            };

            waypoint3.z =
            {
                0.0,
                0.0
            };

            waypoint3.qx =
            {
                q.x,
                q.x
            };

            waypoint3.qy =
            {
                q.y,
                q.y
            };

            waypoint3.qz =
            {
                q.z,
                q.z
            };

            waypoint3.qw =
            {
                q.w,
                q.w
            };


            // =================================================
            // 19. Record name
            // =================================================

            const std::string record1 =
                "bull_leveling_waypoint_1";

            const std::string record2 =
                "bull_leveling_waypoint_2";

            const std::string record3 =
                "bull_leveling_waypoint_3";


            // =================================================
            // 20. MongoDBへwaypointを書き込む
            // =================================================

            if (!writeWaypoint(
                    collection,
                    record1,
                    waypoint1))
            {
                return NodeStatus::FAILURE;
            }


            if (!writeWaypoint(
                    collection,
                    record2,
                    waypoint2))
            {
                return NodeStatus::FAILURE;
            }


            if (!writeWaypoint(
                    collection,
                    record3,
                    waypoint3))
            {
                return NodeStatus::FAILURE;
            }


            // =================================================
            // 21. data2.waypoints_record_nameを更新
            //
            // "waypoints_record_name": [
            //     "bull_leveling_waypoint_1",
            //     "bull_leveling_waypoint_2",
            //     "bull_leveling_waypoint_3"
            // ]
            // =================================================

            bsoncxx::builder::stream::document update_builder;

            update_builder
                << "$set"
                << open_document

                << "waypoints_record_name"
                << open_array

                << record1
                << record2
                << record3

                << close_array

                << close_document

                << "$currentDate"
                << open_document
                << "timestamps"
                << true
                << close_document;


            mongocxx::options::update update_options;

            update_options.upsert(false);


            // ========================================================
            // data2の検索条件
            // ========================================================

            bsoncxx::builder::stream::document filter_builder;

            filter_builder
                << "record_name"
                << data2_record;


            auto filter =
                filter_builder.view();


            auto update_result =
                collection.update_one(
                    filter,
                    update_builder.view(),
                    update_options);


            if (!update_result ||
                update_result->matched_count() == 0)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "Failed to update "
                    << "waypoints_record_name."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            if (!update_result ||
                update_result->matched_count() == 0)
            {
                std::cout
                    << "[BulldozerPathCreater] "
                    << "Failed to update "
                    << "waypoints_record_name."
                    << std::endl;

                return NodeStatus::FAILURE;
            }


            // =================================================
            // Debug
            // =================================================

            std::cout
                << std::endl
                << "========================================"
                << std::endl
                << "[BulldozerPathCreater]"
                << std::endl
                << "data1          : "
                << data1_record
                << std::endl
                << "data2          : "
                << data2_record
                << std::endl
                << "current block  : "
                << current_block
                << std::endl
                << "next block     : "
                << next_block
                << std::endl
                << "wx             : "
                << wx
                << std::endl
                << "wy             : "
                << wy
                << std::endl
                << "w_blade        : "
                << w_blade
                << std::endl
                << "back_offset    : "
                << back_offset
                << std::endl
                << "blade_offset   : "
                << blade_offset
                << std::endl
                << "L              : ("
                << L.x
                << ", "
                << L.y
                << ")"
                << std::endl
                << "D              : ("
                << D.x
                << ", "
                << D.y
                << ")"
                << std::endl
                << "L+             : ("
                << L_plus.x
                << ", "
                << L_plus.y
                << ")"
                << std::endl
                << "L-             : ("
                << L_minus.x
                << ", "
                << L_minus.y
                << ")"
                << std::endl
                << "S              : ("
                << S.x
                << ", "
                << S.y
                << ")"
                << std::endl
                << "orig_first     : ("
                << orig_first.x
                << ", "
                << orig_first.y
                << ")"
                << std::endl
                << "orig_last      : ("
                << orig_last.x
                << ", "
                << orig_last.y
                << ")"
                << std::endl
                << "Quaternion     : ("
                << q.x
                << ", "
                << q.y
                << ", "
                << q.z
                << ", "
                << q.w
                << ")"
                << std::endl
                << std::endl;


            printWaypoint(
                record1,
                waypoint1);

            printWaypoint(
                record2,
                waypoint2);

            printWaypoint(
                record3,
                waypoint3);


            std::cout
                << "========================================"
                << std::endl
                << std::endl;


            return NodeStatus::SUCCESS;
        }
        catch (const std::exception& e)
        {
            std::cout
                << "[BulldozerPathCreater] "
                << "Exception: "
                << e.what()
                << std::endl;

            return NodeStatus::FAILURE;
        }
    }


private:

    // ========================================================
    // MongoDB record search
    // ========================================================

    static mongocxx::stdx::optional<
        bsoncxx::document::value>
    findRecord(
        mongocxx::collection& collection,
        const std::string& record_name)
    {
        bsoncxx::builder::stream::document
            filter_builder;

        filter_builder
            << "record_name"
            << record_name;

        return collection.find_one(
            filter_builder.view());
    }


    // ========================================================
    // MongoDB filter
    // ========================================================

    // static bsoncxx::document::view
    // data2_filter(
    //     const std::string& record_name)
    // {
    //     // この関数はviewの寿命の問題を避けるため
    //     // 実際には使用しない
    //     //
    //     // tick()内では下記のようにfilterを作成する。
    //     //
    //     // bsoncxx::builder::stream::document filter_builder;
    //     // filter_builder << "record_name" << data2_record;

    //     static bsoncxx::document::value dummy =
    //         bsoncxx::builder::stream::document{}
    //             << "record_name"
    //             << record_name;

    //     return dummy.view();
    // }


    // ========================================================
    // BSON number -> double
    // ========================================================

    static double getNumber(
        const bsoncxx::document::view& doc,
        const std::string& key)
    {
        auto element =
            doc[key];


        if (!element)
        {
            throw std::runtime_error(
                "Missing numeric field: " + key);
        }


        switch (element.type())
        {
            case bsoncxx::type::k_double:

                return element
                    .get_double()
                    .value;


            case bsoncxx::type::k_int32:

                return static_cast<double>(
                    element
                        .get_int32()
                        .value);


            case bsoncxx::type::k_int64:

                return static_cast<double>(
                    element
                        .get_int64()
                        .value);


            default:

                throw std::runtime_error(
                    "Field is not numeric: " + key);
        }
    }


    // ========================================================
    // Add
    // ========================================================

    static BulldozerPoint2D add(
        const BulldozerPoint2D& a,
        const BulldozerPoint2D& b)
    {
        return
        {
            a.x + b.x,
            a.y + b.y
        };
    }


    // ========================================================
    // Subtract
    // ========================================================

    static BulldozerPoint2D subtract(
        const BulldozerPoint2D& a,
        const BulldozerPoint2D& b)
    {
        return
        {
            a.x - b.x,
            a.y - b.y
        };
    }


    // ========================================================
    // Multiply
    // ========================================================

    static BulldozerPoint2D multiply(
        const BulldozerPoint2D& a,
        double scalar)
    {
        return
        {
            a.x * scalar,
            a.y * scalar
        };
    }


    // ========================================================
    // Write waypoint
    // ========================================================

    static bool writeWaypoint(
        mongocxx::collection& collection,
        const std::string& record_name,
        const BulldozerWaypoint& waypoint)
    {
        bsoncxx::builder::stream::document
            update_builder;


        update_builder
            << "$set"
            << open_document

            << "model_name"
            << open_array
            << "d37pxi_24"
            << "d37pxi"
            << close_array

            << "type"
            << "static"

            << "record_name"
            << record_name

            // -----------------------------------------------
            // x
            // -----------------------------------------------

            << "x"
            << open_array
            << waypoint.x[0]
            << waypoint.x[1]
            << close_array

            // -----------------------------------------------
            // y
            // -----------------------------------------------

            << "y"
            << open_array
            << waypoint.y[0]
            << waypoint.y[1]
            << close_array

            // -----------------------------------------------
            // z
            // -----------------------------------------------

            << "z"
            << open_array
            << waypoint.z[0]
            << waypoint.z[1]
            << close_array

            // -----------------------------------------------
            // qx
            // -----------------------------------------------

            << "qx"
            << open_array
            << waypoint.qx[0]
            << waypoint.qx[1]
            << close_array

            // -----------------------------------------------
            // qy
            // -----------------------------------------------

            << "qy"
            << open_array
            << waypoint.qy[0]
            << waypoint.qy[1]
            << close_array

            // -----------------------------------------------
            // qz
            // -----------------------------------------------

            << "qz"
            << open_array
            << waypoint.qz[0]
            << waypoint.qz[1]
            << close_array

            // -----------------------------------------------
            // qw
            // -----------------------------------------------

            << "qw"
            << open_array
            << waypoint.qw[0]
            << waypoint.qw[1]
            << close_array

            << close_document;


        bsoncxx::builder::stream::document
            filter_builder;


        filter_builder
            << "record_name"
            << record_name;


        mongocxx::options::update options;

        options.upsert(true);


        auto result =
            collection.update_one(
                filter_builder.view(),
                update_builder.view(),
                options);


        if (!result)
        {
            std::cout
                << "[BulldozerPathCreater] "
                << "Failed to write waypoint: "
                << record_name
                << std::endl;

            return false;
        }


        std::cout
            << "[BulldozerPathCreater] "
            << "Waypoint written: "
            << record_name
            << std::endl;

        return true;
    }


    // ========================================================
    // Debug print
    // ========================================================

    static void printWaypoint(
        const std::string& name,
        const BulldozerWaypoint& waypoint)
    {
        std::cout
            << name
            << std::endl

            << "  x  : ["
            << waypoint.x[0]
            << ", "
            << waypoint.x[1]
            << "]"
            << std::endl

            << "  y  : ["
            << waypoint.y[0]
            << ", "
            << waypoint.y[1]
            << "]"
            << std::endl

            << "  z  : ["
            << waypoint.z[0]
            << ", "
            << waypoint.z[1]
            << "]"
            << std::endl

            << "  qx : ["
            << waypoint.qx[0]
            << ", "
            << waypoint.qx[1]
            << "]"
            << std::endl

            << "  qy : ["
            << waypoint.qy[0]
            << ", "
            << waypoint.qy[1]
            << "]"
            << std::endl

            << "  qz : ["
            << waypoint.qz[0]
            << ", "
            << waypoint.qz[1]
            << "]"
            << std::endl

            << "  qw : ["
            << waypoint.qw[0]
            << ", "
            << waypoint.qw[1]
            << "]"
            << std::endl;
    }


private:

    mongocxx::pool pool_;
};


#endif  // BULLDOZER_PATH_CREATER_HPP