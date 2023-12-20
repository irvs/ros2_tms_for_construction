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

#include "tms_ts_subtask/subtask_node_base.hpp"

// BaseClassSubtasks::inst = mongocxx::instance{};

mongocxx::instance BaseClassSubtasks::inst{};

BaseClassSubtasks::BaseClassSubtasks(const std::string& node_name_) : rclcpp::Node(node_name_) {
};

// データベースから動的パラメータの値をとってくるための関数
std::map<std::string, int> BaseClassSubtasks::GetParamFromDB(std::string parts_name){
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    RCLCPP_INFO(this->get_logger(), "===========================================A");
    client["rostmsdb"];
    RCLCPP_INFO(this->get_logger(), "===========================================B");
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];
    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "parts_name" << parts_name;
    auto filter = filter_builder.view();
    auto result = collection.find_one(filter);
    if (result) {
        std::map<std::string, int> dataMap;

        for (auto&& element : result->view()) {
            std::string key = element.key().to_string();
            if (key != "_id" && key != "type" && key != "description" && key != "parameter_id" && key != "parameter_type" && key != "parts_name") {
                if (element.type() == bsoncxx::type::k_int32) {
                    int value = element.get_int32();
                    dataMap[key] = value;
                }
            }
        }

        return dataMap;
    } else {
        std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
        return std::map<std::string, int>();
    }
}