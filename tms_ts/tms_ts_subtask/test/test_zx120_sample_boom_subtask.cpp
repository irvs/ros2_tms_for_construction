#include <mongocxx/client.hpp>
#include <mongocxx/exception/exception.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <gtest/gtest.h>

TEST(zx120_sample_sample_subtask_test, mongoclient){
    static mongocxx::instance inst{};
    // mongocxx::instance MongoDB::inst{};
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
}

TEST(zx120_sample_sample_subtask_test, mongoclient2){
    mongocxx::client client;
}

TEST(zx120_sample_sample_subtask_test, mongoclient3){
    mongocxx::uri uri_text{"mongodb://localhost:27017"};
}

TEST(zx120_sample_sample_subtask_test, mongoclient4){
    static mongocxx::instance inst{};
    // mongocxx::instance MongoDB::inst{};
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
}

TEST(zx120_sample_sample_subtask_test, mongoclient5){
    mongocxx::instance inst{};
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];

    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "parts_name" << "zx120_boom";

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

                    std::cout << key << " : " << value << std::endl;
                }
            }
        }

        // return dataMap;
    } else {
        std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
        // return std::map<std::string, int>();
    }
}