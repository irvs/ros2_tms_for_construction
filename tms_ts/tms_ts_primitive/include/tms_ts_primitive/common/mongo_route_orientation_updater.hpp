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

#ifndef MONGO_ROUTE_ORIENTATION_UPDATER_HPP
#define MONGO_ROUTE_ORIENTATION_UPDATER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>

#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/types.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/options/find.hpp>
#include <mongocxx/options/update.hpp>

#include <mongocxx/exception/exception.hpp>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class MongoRouteOrientationUpdater : public SyncActionNode
{
public:
    MongoRouteOrientationUpdater(
        const std::string& name,
        const NodeConfiguration& config)
        : SyncActionNode(name, config)
    {
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("model_name"),
            InputPort<std::string>("record_name"),
            InputPort<std::string>("read_direction")
        };
    }

    NodeStatus tick() override
    {
        auto model_name = getInput<std::string>("model_name");
        auto record_name = getInput<std::string>("record_name");
        auto read_direction = getInput<std::string>("read_direction");

        if (!model_name || !record_name)
        {
            std::cerr
                << "[MongoRouteOrientationUpdater] "
                << "Missing model_name or record_name"
                << std::endl;

            return NodeStatus::FAILURE;
        }

        if (read_direction.value() == "DOWN" || read_direction.value() == "down")
            {
            read_direction = "down";
            }
            else
            {
            read_direction = "up";
            }

        try
        {
            // --------------------------------------------------
            // 1. Get current position/orientation
            //    Prefer machine_pose in MongoDB.
            //    Fall back to Blackboard if unavailable.
            // --------------------------------------------------

            double current_x = 0.0;
            double current_y = 0.0;
            double current_z = 0.0;

            double current_qx = 0.0;
            double current_qy = 0.0;
            double current_qz = 0.0;
            double current_qw = 1.0;

            bool current_pose_available = false;

            mongocxx::client client{
                mongocxx::uri{"mongodb://localhost:27017"}
            };

            auto db = client["rostmsdb"];

            auto pose_collection = db["machine_pose"];

            {
                bsoncxx::builder::basic::document filter;
                filter.append(
                    bsoncxx::builder::basic::kvp(
                        "name", model_name.value()));

                bsoncxx::builder::basic::document sort;
                sort.append(
                    bsoncxx::builder::basic::kvp("time", -1));

                mongocxx::options::find options;
                options.sort(sort.view());
                options.limit(1);

                auto cursor = pose_collection.find(
                    filter.view(), options);

                auto iter = cursor.begin();

                if (iter != cursor.end())
                {
                    auto doc = *iter;

                    auto read_double =
                        [](bsoncxx::document::view document,
                           const char* field,
                           double& output) -> bool
                    {
                        auto element = document[field];

                        if (!element)
                        {
                            return false;
                        }

                        switch (element.type())
                        {
                            case bsoncxx::type::k_double:
                                output =
                                    element.get_double().value;
                                return true;

                            case bsoncxx::type::k_int32:
                                output = static_cast<double>(
                                    element.get_int32().value);
                                return true;

                            case bsoncxx::type::k_int64:
                                output = static_cast<double>(
                                    element.get_int64().value);
                                return true;

                            default:
                                return false;
                        }
                    };

                    auto msg_element = doc["msg"];

                    if (msg_element &&
                        msg_element.type() == bsoncxx::type::k_document)
                    {
                        auto msg = msg_element.get_document().view();

                        auto pose_element = msg["pose"];

                        if (pose_element &&
                            pose_element.type() == bsoncxx::type::k_document)
                        {
                            auto pose = pose_element.get_document().view();

                            // poseからpositionとorientationを読み出す関数
                            auto read_pose_fields =
                                [&](bsoncxx::document::view pose_doc) -> bool
                            {
                                auto position_element = pose_doc["position"];
                                auto orientation_element = pose_doc["orientation"];

                                if (!position_element ||
                                    !orientation_element ||
                                    position_element.type() !=
                                        bsoncxx::type::k_document ||
                                    orientation_element.type() !=
                                        bsoncxx::type::k_document)
                                {
                                    return false;
                                }

                                auto position =
                                    position_element.get_document().view();

                                auto orientation =
                                    orientation_element.get_document().view();

                                // 位置とクォータニオンの全要素を取得
                                return
                                    read_double(position, "x", current_x) &&
                                    read_double(position, "y", current_y) &&
                                    read_double(position, "z", current_z) &&
                                    read_double(orientation, "x", current_qx) &&
                                    read_double(orientation, "y", current_qy) &&
                                    read_double(orientation, "z", current_qz) &&
                                    read_double(orientation, "w", current_qw);
                            };

                            // ------------------------------------------
                            // パターンA:
                            // msg.pose.position
                            // ------------------------------------------
                            current_pose_available = read_pose_fields(pose);

                            // ------------------------------------------
                            // パターンB:
                            // msg.pose.pose.position
                            // ------------------------------------------
                            if (!current_pose_available)
                            {
                                auto nested_pose_element = pose["pose"];

                                if (nested_pose_element &&
                                    nested_pose_element.type() ==
                                        bsoncxx::type::k_document)
                                {
                                    auto nested_pose =
                                        nested_pose_element.get_document().view();

                                    current_pose_available =
                                        read_pose_fields(nested_pose);
                                }
                            }
                        }
                    }

                    if (msg_element &&
                        msg_element.type() ==
                            bsoncxx::type::k_document)
                    {
                        auto msg =
                            msg_element.get_document().view();

                        auto pose_element = msg["pose"];

                        if (pose_element &&
                            pose_element.type() ==
                                bsoncxx::type::k_document)
                        {
                            auto pose =
                                pose_element.get_document().view();

                            auto pose_pose_element = pose["pose"];

                            if (pose_pose_element &&
                                pose_pose_element.type() ==
                                    bsoncxx::type::k_document)
                            {
                                auto pose_pose =
                                    pose_pose_element
                                        .get_document().view();

                                auto position_element =
                                    pose_pose["position"];

                                auto orientation_element =
                                    pose_pose["orientation"];

                                if (
                                    position_element &&
                                    orientation_element &&
                                    position_element.type() ==
                                        bsoncxx::type::k_document &&
                                    orientation_element.type() ==
                                        bsoncxx::type::k_document)
                                {
                                    auto position =
                                        position_element
                                            .get_document().view();

                                    auto orientation =
                                        orientation_element
                                            .get_document().view();

                                    current_pose_available =
                                        read_double(
                                            position, "x", current_x) &&
                                        read_double(
                                            position, "y", current_y) &&
                                        read_double(
                                            position, "z", current_z) &&
                                        read_double(
                                            orientation, "x", current_qx) &&
                                        read_double(
                                            orientation, "y", current_qy) &&
                                        read_double(
                                            orientation, "z", current_qz) &&
                                        read_double(
                                            orientation, "w", current_qw);
                                }
                            }
                        }
                    }
                }
            }

            // --------------------------------------------------
            // 2. Fall back to Blackboard if current pose
            //    is not available in machine_pose.
            // --------------------------------------------------

            auto blackboard = config().blackboard;

            if (!current_pose_available)
            {
                std::cout
                    << "[MongoRouteOrientationUpdater] "
                    << "Current pose unavailable in machine_pose. "
                    << "Using Blackboard values."
                    << std::endl;

                if (
                    !blackboard->get("x", current_x) ||
                    !blackboard->get("y", current_y) ||
                    !blackboard->get("z", current_z) ||
                    !blackboard->get("qx", current_qx) ||
                    !blackboard->get("qy", current_qy) ||
                    !blackboard->get("qz", current_qz) ||
                    !blackboard->get("qw", current_qw))
                {
                    std::cerr
                        << "[MongoRouteOrientationUpdater] "
                        << "Failed to read fallback pose "
                        << "from Blackboard."
                        << std::endl;

                    return NodeStatus::FAILURE;
                }
            }

            std::cout
                << "[MongoRouteOrientationUpdater] "
                << "Current pose source: "
                << (current_pose_available
                        ? "machine_pose"
                        : "Blackboard")
                << std::endl;

            std::cout
                << "  model_name = " << model_name.value()
                << "\n  position = ("
                << current_x << ", "
                << current_y << ", "
                << current_z << ")"
                << "\n  quaternion = ("
                << current_qx << ", "
                << current_qy << ", "
                << current_qz << ", "
                << current_qw << ")"
                << std::endl;

            // --------------------------------------------------
            // 3. Read route document
            // --------------------------------------------------

            auto route_collection = db["parameter"];

            bsoncxx::builder::basic::document route_filter;
            route_filter.append(
                bsoncxx::builder::basic::kvp(
                    "record_name", record_name.value()));

            auto route_cursor =
                route_collection.find(route_filter.view());

            auto route_iter = route_cursor.begin();

            if (route_iter == route_cursor.end())
            {
                std::cerr
                    << "[MongoRouteOrientationUpdater] "
                    << "Route not found: "
                    << record_name.value()
                    << std::endl;

                return NodeStatus::FAILURE;
            }

            auto route_doc = *route_iter;

            // --------------------------------------------------
            // 4. Read numeric arrays from route document
            // --------------------------------------------------

            auto read_array =
                [](bsoncxx::document::view document,
                   const char* field,
                   std::vector<double>& output) -> bool
            {
                auto element = document[field];

                if (!element ||
                    element.type() != bsoncxx::type::k_array)
                {
                    return false;
                }

                auto array = element.get_array().value;

                for (auto item : array)
                {
                    switch (item.type())
                    {
                        case bsoncxx::type::k_double:
                            output.push_back(
                                item.get_double().value);
                            break;

                        case bsoncxx::type::k_int32:
                            output.push_back(
                                static_cast<double>(
                                    item.get_int32().value));
                            break;

                        case bsoncxx::type::k_int64:
                            output.push_back(
                                static_cast<double>(
                                    item.get_int64().value));
                            break;

                        default:
                            return false;
                    }
                }

                return true;
            };

            std::vector<double> x;
            std::vector<double> y;
            std::vector<double> z;

            if (
                !read_array(route_doc, "x", x) ||
                !read_array(route_doc, "y", y) ||
                !read_array(route_doc, "z", z))
            {
                std::cerr
                    << "[MongoRouteOrientationUpdater] "
                    << "Failed to read route coordinates."
                    << std::endl;

                return NodeStatus::FAILURE;
            }

            const std::size_t point_num = x.size();

            if (point_num == 0 ||
                y.size() != point_num ||
                z.size() != point_num)
            {
                std::cerr
                    << "[MongoRouteOrientationUpdater] "
                    << "Invalid route coordinate array sizes."
                    << std::endl;

                return NodeStatus::FAILURE;
            }

            // --------------------------------------------------
            // 5. Calculate quaternion arrays
            // --------------------------------------------------

            std::vector<double> qx(point_num, 0.0);
            std::vector<double> qy(point_num, 0.0);
            std::vector<double> qz(point_num, 0.0);
            std::vector<double> qw(point_num, 1.0);

            // downの場合は逆方向に処理する
            const bool reverse = (read_direction.value() == "down");

            // 最初に処理する点のインデックス
            const std::size_t first_index = reverse ? point_num - 1 : 0;

            // First point: current orientation.
            qx[first_index] = current_qx;
            qy[first_index] = current_qy;
            qz[first_index] = current_qz;
            qw[first_index] = current_qw;

            // Remaining points: face the direction
            // from the previous point to the current point.
            for (std::size_t step = 1; step < point_num; ++step)
            {
                // 1つ前に処理した点
                const std::size_t previous_index =
                    reverse ? point_num - step : step - 1;

                // 現在処理している点
                const std::size_t current_index =
                    reverse ? point_num - step - 1 : step;
                    
                // 進行方向ベクトル
                const double dx =
                    x[current_index] - x[previous_index];

                const double dy =
                    y[current_index] - y[previous_index];

                // If two consecutive points have the same
                // XY coordinates, retain the previous
                // orientation.
                if (std::hypot(dx, dy) < 1e-9)
                {
                    qx[current_index] = qx[previous_index];
                    qy[current_index] = qy[previous_index];
                    qz[current_index] = qz[previous_index];
                    qw[current_index] = qw[previous_index];

                    continue;
                }

                const double yaw = std::atan2(dy, dx);

                qx[current_index] = 0.0;
                qy[current_index] = 0.0;
                qz[current_index] = std::sin(yaw / 2.0);
                qw[current_index] = std::cos(yaw / 2.0);
            }

            // --------------------------------------------------
            // 6. Update only quaternion arrays in MongoDB
            // --------------------------------------------------

            auto make_bson_array =
                [](const std::vector<double>& values)
            {
                bsoncxx::builder::basic::array array;

                for (double value : values)
                {
                    array.append(value);
                }

                return array;
            };

            bsoncxx::builder::basic::document update_fields;

            update_fields.append(
                bsoncxx::builder::basic::kvp(
                    "qx", make_bson_array(qx)));

            update_fields.append(
                bsoncxx::builder::basic::kvp(
                    "qy", make_bson_array(qy)));

            update_fields.append(
                bsoncxx::builder::basic::kvp(
                    "qz", make_bson_array(qz)));

            update_fields.append(
                bsoncxx::builder::basic::kvp(
                    "qw", make_bson_array(qw)));

            bsoncxx::builder::basic::document update_doc;

            update_doc.append(
                bsoncxx::builder::basic::kvp(
                    "$set", update_fields.extract()));

            auto update_result = route_collection.update_one(
                route_filter.view(),
                update_doc.view());

            if (!update_result) {
                std::cerr
                    << "[MongoRouteOrientationUpdater] "
                    << "Route update failed: no document matched the filter."
                    << std::endl;

                return NodeStatus::FAILURE;
            }

            std::cout
                << "[MongoRouteOrientationUpdater] "
                << "Route matched successfully. Modified documents: "
                << update_result->modified_count()
                << std::endl;

            // --------------------------------------------------
            // 7. Log results
            // --------------------------------------------------

            for (std::size_t i = 0; i < point_num; ++i)
            {
                std::cout
                    << "[MongoRouteOrientationUpdater] Point "
                    << i
                    << " position=("
                    << x[i] << ", "
                    << y[i] << ", "
                    << z[i] << ") quaternion=("
                    << qx[i] << ", "
                    << qy[i] << ", "
                    << qz[i] << ", "
                    << qw[i] << ")"
                    << std::endl;
            }

            std::cout
                << "[MongoRouteOrientationUpdater] "
                << "Updated route: "
                << record_name.value()
                << std::endl;

            return NodeStatus::SUCCESS;
        }
        catch (const std::exception& e)
        {
            std::cerr
                << "[MongoRouteOrientationUpdater] "
                << "Exception: " << e.what()
                << std::endl;

            return NodeStatus::FAILURE;
        }
    }
};

#endif  // MONGO_ROUTE_ORIENTATION_UPDATER_HPP