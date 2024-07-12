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

mongocxx::instance SubtaskNodeBase::inst{};   


SubtaskNodeBase::SubtaskNodeBase(const std::string& node_name_) : rclcpp::Node(node_name_) {};

// データベースから動的パラメータの値をとってくるための関数(Original)
// std::map<std::string, float> SubtaskNodeBase::GetParamFromDB(std::string model_name, std::string record_name){
//     mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
//     mongocxx::database db = client["rostmsdb"];
//     mongocxx::collection collection = db["parameter"];
//     bsoncxx::builder::stream::document filter_builder;
//     filter_builder << "model_name" << model_name << "record_name" << record_name;
//     auto filter = filter_builder.view();
//     auto result = collection.find_one(filter);
//     if (result) {
//         std::map<std::string, float> dataMap;

//         for (auto&& element : result->view()) {
//             std::string key = element.key().to_string();
//             if (key != "_id" && key != "model_name" && key != "type" && key != "record_name") {
//                 if (element.type() == bsoncxx::type::k_double) {
//                     float value = static_cast<float>(element.get_double());
//                     dataMap[key] = value;
//                 }
//             }
//         }

//         return dataMap;
//     } else {
//         std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
//         return std::map<std::string, float>();
//     }
// }
