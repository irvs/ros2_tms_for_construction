#ifndef CONDITIONAL_EXPRESSION_NODE_HPP
#define CONDITIONAL_EXPRESSION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
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

class ConditionalExpression : public SyncActionNode
{
public:
    ConditionalExpression(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config), pool_(mongocxx::uri{})
    { 
        node_ = rclcpp::Node::make_shared("conditional_expression");
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~ConditionalExpression()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    static PortsList providedPorts()
    {
        return { 
            InputPort<std::string>("conditional_expression")
        };
    }

    NodeStatus tick() override
    {
        Optional<std::string> expr_str = getInput<std::string>("conditional_expression");

        if (!expr_str)
        {
            std::cout << "[ConditionalExpression] Missing required input." << std::endl;
            return NodeStatus::FAILURE;
        }

        bool result;
        if (!evaluateCondition(expr_str.value(), result))
        {
            std::cout << "[ConditionalExpression] Failed to evaluate condition." << std::endl;
            return NodeStatus::FAILURE;
        }

        std::cout << "[ConditionalExpression] Evaluated condition: " << expr_str.value() << " Result: " << std::boolalpha << result << std::endl;

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

        auto blackboard_ptr = config().blackboard;
        auto blackboard_keys = blackboard_ptr->getKeys();

        std::unordered_map<std::string, double> temp_variables;

        for (const auto& key_view : blackboard_keys)
        {
            try
            {
                std::string key(key_view.data(), key_view.size());
                auto value_any = blackboard_ptr->getAny(key);
                if(value_any)
                {
                    if (value_any->type() == typeid(int))
                    {
                        double value = static_cast<double>(value_any->cast<int>());
                        temp_variables[key] = value;
                    }
                    else if (value_any->type() == typeid(double))
                    {
                        double value = value_any->cast<double>();
                        temp_variables[key] = value;
                    }
                    else if (value_any->type() == typeid(std::string))
                    {
                        std::string value_str = value_any->cast<std::string>();
                        double value = std::stod(value_str);
                        temp_variables[key] = value;
                    }
                    else
                    {
                        continue; // Skip unsupported types
                    }
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error: Unable to retrieve variable " << key_view << " from blackboard or convert it to double. " << e.what() << std::endl;
                return false;
            }
        }

        for (auto& var : temp_variables)
        {
            std::cout << "Variable: " << var.first << " Value: " << var.second << std::endl;
            symbol_table.add_variable(var.first, var.second);
        }

        expression.register_symbol_table(symbol_table);

        if (!parser.compile(condition, expression))
        {
            std::cerr << "Error: " << parser.error() << std::endl;
            for (std::size_t i = 0; i < parser.error_count(); ++i) {
                exprtk::parser_error::type error = parser.get_error(i);
                std::cerr << "Error: " << error.diagnostic << " at position: " << error.token.position << std::endl;
            }
            return false;
        }

        result = expression.value() != 0.0;
        return true;
    }
};

#endif
