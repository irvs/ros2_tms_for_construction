#ifndef CONDITIONAL_EXPRESSION_BOOL_NODE_HPP
#define CONDITIONAL_EXPRESSION_BOOL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/pool.hpp>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "exprtk.hpp"
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <typeinfo>

using namespace BT;

class ConditionalExpressionBool : public SyncActionNode
{
public:
    ConditionalExpressionBool(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config), pool_(mongocxx::uri{})
    { 
        node_ = rclcpp::Node::make_shared("conditional_expression_bool");
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~ConditionalExpressionBool()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    static PortsList providedPorts()
    {
        return { 
            InputPort<std::string>("conditional_expression_bool")
        };
    }

    NodeStatus tick() override
    {
        Optional<std::string> expr_str = getInput<std::string>("conditional_expression_bool");

        if (!expr_str)
        {
            std::cout << "[ConditionalExpressionBool] Missing required input." << std::endl;
            return NodeStatus::FAILURE;
        }

        bool result;
        if (!evaluateCondition(expr_str.value(), result))
        {
            std::cout << "[ConditionalExpressionBool] Failed to evaluate condition." << std::endl;
            return NodeStatus::FAILURE;
        }

        std::cout << "[ConditionalExpressionBool] Evaluated condition: " << expr_str.value() << " Result: " << std::boolalpha << result << std::endl;

        return result ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::thread spin_thread_; 
    mongocxx::pool pool_; 

    bool evaluateCondition(const std::string& condition, bool& result)
    {
        typedef exprtk::symbol_table<double> symbol_table_t;
        typedef exprtk::expression<double>   expression_t;
        typedef exprtk::parser<double>       parser_t;

        symbol_table_t symbol_table;
        expression_t   expression;
        parser_t       parser;

        // Blackboardの変数をsymbol_tableに追加
        auto blackboard_ptr = config().blackboard;
        auto blackboard_keys = blackboard_ptr->getKeys();

        std::unordered_map<std::string, double> temp_variables;

        for (const auto& key_view : blackboard_keys)
        {
            try
            {
                std::string key(key_view.data(), key_view.size());
                auto value_any = blackboard_ptr->getAny(key);
                if (value_any)
                {
                    if (value_any->type() == typeid(bool))
                    {
                        bool value = value_any->cast<bool>();
                        temp_variables[key] = value ? 1.0 : 0.0;
                    }
                    else
                    {
                        continue; // Skip unsupported types
                    }
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "[ConditionalExpressionBool] Error: Unable to retrieve variable " << key_view << " from blackboard or convert it to bool. " << e.what() << std::endl;
                return false;
            }
        }

        // symbol_tableにBlackboard変数を登録
        for (auto& var : temp_variables)
        {
            std::cout << "[ConditionalExpressionBool] Variable: " << var.first << " Value: " << std::boolalpha << (var.second != 0.0) << std::endl;
            symbol_table.add_variable(var.first, var.second);
        }

        // "true" と "false" のリテラルをsymbol_tableに追加
        symbol_table.add_constant("true", 1.0);
        symbol_table.add_constant("false", 0.0);

        expression.register_symbol_table(symbol_table);

        // 条件式をコンパイルして評価
        if (!parser.compile(condition, expression))
        {
            std::cerr << "[ConditionalExpressionBool] Error: " << parser.error() << std::endl;
            for (std::size_t i = 0; i < parser.error_count(); ++i) {
                exprtk::parser_error::type error = parser.get_error(i);
                std::cerr << "[ConditionalExpressionBool] Error: " << error.diagnostic << " at position: " << error.token.position << std::endl;
            }
            return false;
        }

        result = expression.value() != 0.0;
        return true;
    }
};

#endif
