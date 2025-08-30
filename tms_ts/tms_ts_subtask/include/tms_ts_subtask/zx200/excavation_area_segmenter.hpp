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

#ifndef EXCAVATION_AREA_SEGMENTER_NODE_HPP
#define EXCAVATION_AREA_SEGMENTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <cmath>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>  
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/pool.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class ExcavationAreaSegmenter : public SyncActionNode
{
public:
    ExcavationAreaSegmenter(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config), pool_(mongocxx::uri{})
    {
        node_ = rclcpp::Node::make_shared("excavation_area_segmenter");
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~ExcavationAreaSegmenter()
    {
        // ���I�������̈�̉��

        std::cout << "OK1" << std::endl;
        if (corners_coord != nullptr) {
            delete[] corners_coord[0];
            delete[] corners_coord[1];
            delete[] corners_coord;
        }
        rclcpp::shutdown();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("mongo_record_name")};
    }

    NodeStatus tick() override
    {
        std::cout << "OK2" << std::endl;
        Optional<std::string> key = getInput<std::string>("mongo_record_name");
        if (!key)
        {
            std::cout << "[ExcavationAreaSegmenter] missing required input. Please fill key or value parameters." << std::endl;
            return NodeStatus::FAILURE;
        }

        mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
        mongocxx::database db = client["rostmsdb"];
        mongocxx::collection collection = db["parameter"];
        bsoncxx::builder::stream::document filter_builder;
        filter_builder << "record_name" << key.value();
        auto filter = filter_builder.view();
        auto doc = collection.find_one(filter);


        if (!doc)
        {
            std::cout << "[ExcavationAreaSegmenter]  Couldn't find parameter data containing " << key.value() << " as record_name in parameter collection" << std::endl;
            return NodeStatus::FAILURE;
        }

        try
        {
            auto view = doc->view();
            if (view["CORNERS"] && view["CORNERS"].type() == bsoncxx::type::k_array) {
                auto corners = view["CORNERS"].get_array().value;

                auto x_array_it = corners.begin();
                auto y_array_it = std::next(x_array_it);

                if (x_array_it != corners.end() && y_array_it != corners.end() && x_array_it->type() == bsoncxx::type::k_array && y_array_it->type() == bsoncxx::type::k_array) {

                    auto x_array = x_array_it->get_array().value;
                    auto y_array = y_array_it->get_array().value;
                    

                    if (std::distance(x_array.begin(), x_array.end()) == std::distance(y_array.begin(), y_array.end())) {
                        auto x_it = x_array.begin();
                        auto y_it = y_array.begin();

                        int corners_size = std::distance(x_array.begin(), x_array.end());
                        
                        // �������̈悪�m�ۂ���Ă���ꍇ�͉��
                        if (corners_coord != nullptr) {
                            delete[] corners_coord[0];
                            delete[] corners_coord[1];
                            delete[] corners_coord;
                        }

                        // ���I�������̈���m��
                        corners_coord = new double*[2];
                        corners_coord[0] = new double[corners_size];
                        corners_coord[1] = new double[corners_size];

                        int index = 0;
                        for (; x_it != x_array.end() && y_it != y_array.end(); ++x_it, ++y_it) {
                            corners_coord[0][index] = x_it->get_double().value;
                            corners_coord[1][index] = y_it->get_double().value;
                            std::cout << "[ExcavationAreaSegmenter]  Corner" << index << ":(" << x_it->get_double().value << ", " << y_it->get_double().value << ")" << std::endl;
                            index++;
                        }

                    } else {
                        std::cout << "[ExcavationAreaSegmenter]  The sizes of the '0' and '1' arrays in CORNERS array do not match." << std::endl;
                        return NodeStatus::FAILURE;
                    }
                } else {
                    std::cout << "[ExcavationAreaSegmenter]  '0' or '1' arrays are not valid or missing." << std::endl;
                    return NodeStatus::FAILURE;
                }
            } else {
                std::cout << "[ExcavationAreaSegmenter]  CORNERS not found or not an array." << std::endl;
                return NodeStatus::FAILURE;
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "[ExcavationAreaSegmenter]  Exception caught: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
        
        
        // ���o�����̈��ZX200�̃o�P�b�g���y�ь@��̈�Ɋ�Â��ĕ�������B����͌@��̈�͎l�p�`�A�@��̈�̏c���̓o�P�b�g���ȏ�Ƃ������O��̌������Bros2�̍��W��m��ł���B

        double main_cell_x_width = corners_coord[0][0]-corners_coord[0][1];
        double main_cell_y_width = corners_coord[1][0]-corners_coord[1][3]; 

        double bucket_width = 1.030; //�@The bucket width of ZX200 is 1.030m without the side cutter.
        
        if (main_cell_x_width < bucket_width) {
            std::cout << "[ExcavationAreaSegmenter]  The width of the excavation area is less than the bucket width of ZX200." << std::endl;
            return NodeStatus::FAILURE;
        }else{
            // x�������̕��������Z�o
            int subcell_num_x = int(main_cell_x_width / bucket_width);
            double surplus_dist_x = fmod(main_cell_x_width, bucket_width);
            // subcells_x[�Z���̃C���f�b�N�X][0:�Z���̎n�_, 1:�Z���̏I�_]�@x�������̃Z���̎n�_�ƏI�_���i�[
            double subcells_x[subcell_num_x+1][2];
            for(int index = 0; index < subcell_num_x-1; index++){
                subcells_x[index][0] = corners_coord[0][0] + index * bucket_width;
                subcells_x[index][1] = corners_coord[0][0] + (index+1) * bucket_width;
            }
            subcells_x[subcell_num_x][0] = corners_coord[0][1] - bucket_width;
            subcells_x[subcell_num_x][1] = corners_coord[0][1];
            
            // y�������̕��������Z�o
            int excavate_dist_y = 3; //�@**ZX200�̌@��̈�̏c����3m�Ɖ���B�_���ǂ�Ŏw����@���l���邱��**
            int subcell_num_y = main_cell_y_width / excavate_dist_y;
            double surplus_dist_y = fmod(main_cell_y_width,excavate_dist_y);
            // subcells_y[�Z���̃C���f�b�N�X][0:�Z���̎n�_, 1:�Z���̏I�_]�@y�������̃Z���̎n�_�ƏI�_���i�[
            double subcells_y[subcell_num_y+1][2];
            // subcells_y[�Z���̃C���f�b�N�X][0:�Z���̎n�_, 1:�Z���̏I�_]�@y�������̃Z���̎n�_�ƏI�_���i�[
            for(int index = 0; index < subcell_num_y-1; index++){
                subcells_y[index][0] = corners_coord[1][0] + index * excavate_dist_y;
                subcells_y[index][1] = corners_coord[1][0] + (index+1) * excavate_dist_y;
            }
            subcells_y[subcell_num_y][0] = corners_coord[1][3] - excavate_dist_y;
            subcells_y[subcell_num_y][1] = corners_coord[1][3];

            // **�e�T�u�Z���ɂ�����ڕW�@����W(���݂͒��S���W)���Z�o�B�_���ǂ�Ŏw����@���l���邱��**
            // target_excavate_coord[�ڕW���S���W�̌�][0:x����, 1:y����]   �ڕW�@��̈�̒��S���W�����
            int subcell_total_num = subcell_num_x * subcell_num_y;
            double target_excavate_coord[subcell_total_num][2];

            // **�ڕW�@����W�����n��Ɋi�[�B�����Ɨǂ��N���v�悪�Ȃ����������邱��**
            // ���ʂȌv�Z�������̂Ŏ��Ԃ�����Ή��P���邱��
            for(int subcell_index_y = 0; subcell_index_y < subcell_num_y; subcell_index_y++){
                for(int subcell_index_x = 0; subcell_index_x < subcell_num_x; subcell_index_x++){
                    target_excavate_coord[subcell_index_x + subcell_index_y][0] = (subcells_x[subcell_index_x][1] - subcells_x[subcell_index_x][0]) / 2.0;
                    target_excavate_coord[subcell_index_x + subcell_index_y][1] = (subcells_y[subcell_index_y][1] - subcells_y[subcell_index_y][0]) / 2.0;
                }
            }

            // �ڕW�@��ʒu�̗����mongodb�Ɋi�[
            bsoncxx::builder::stream::array array_builder;
            for (int subcell_index = 0; subcell_index < subcell_total_num; subcell_index++) {
                array_builder << bsoncxx::builder::stream::open_array
                              << target_excavate_coord[subcell_index][0]
                              << target_excavate_coord[subcell_index][1]
                              << bsoncxx::builder::stream::close_array;
            }
            bsoncxx::builder::stream::document document{};
            document << "model_name" << "zx200"
                     << "type" << "dynamic"
                     << "description" << "The calculated target excavation points for ZX200."
                     << "record_name" << "target_excavate_points"
                     << "target_excavate_points" << array_builder;

            bsoncxx::document::value doc_value = document << bsoncxx::builder::stream::finalize;
            auto result = collection.insert_one(doc_value.view());

            if (result) {
                std::cout << "Successfully inserted document with ID: "
                        << result->inserted_id().get_oid().value.to_string() << std::endl;
                return NodeStatus::SUCCESS;

            } else {
                std::cout << "Failed to insert document" << std::endl;
                return NodeStatus::FAILURE;
            }

        }
}
private:
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_;
    mongocxx::pool pool_; // Create a pool of connections.
    double** corners_coord;
};

#endif
