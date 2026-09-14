#ifndef LOCAL_BLACKBOARD_CALUCULATOR_NODE_H
#define LOCAL_BLACKBOARD_CALUCULATOR_NODE_H

#include "behaviortree_cpp_v3/action_node.h"

#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>

namespace BT
{
class LocalBlackboardCaluculator : public SyncActionNode
{
public:
LocalBlackboardCaluculator(
const std::string& name,
const NodeConfiguration& config)
: SyncActionNode(name, config)
{
setRegistrationID("LocalBlackboardCaluculator");
}

static PortsList providedPorts()
{
return {InputPort<std::string>("calculation_formula"), InputPort<std::string>("output_key")};
}

private:

// ------------------------------------------------------------
// 数式パーサ
//
// 対応:
// + - * /
// ()
// 小数
// 負数
//
// 例:
// 1 + 2 * 3
// (1 + 2) * 3
// -1.5 + 2
// ------------------------------------------------------------
class ExpressionParser
{
public:
explicit ExpressionParser(const std::string& expression)
: expression_(expression), pos_(0)
{
}

double parse()
{
  double result = parseExpression();

  skipSpaces();

  if (pos_ != expression_.size())
  {
    throw std::runtime_error(
        "Unexpected character in expression at position " +
        std::to_string(pos_));
  }

  return result;
}


private:

void skipSpaces()
{
  while (pos_ < expression_.size() &&
         std::isspace(static_cast<unsigned char>(expression_[pos_])))
  {
    ++pos_;
  }
}

double parseExpression()
{
  double value = parseTerm();

  while (true)
  {
    skipSpaces();

    if (pos_ >= expression_.size())
    {
      break;
    }

    char op = expression_[pos_];

    if (op != '+' && op != '-')
    {
      break;
    }

    ++pos_;

    double rhs = parseTerm();

    if (op == '+')
    {
      value += rhs;
    }
    else
    {
      value -= rhs;
    }
  }

  return value;
}

double parseTerm()
{
  double value = parseFactor();

  while (true)
  {
    skipSpaces();

    if (pos_ >= expression_.size())
    {
      break;
    }

    char op = expression_[pos_];

    if (op != '*' && op != '/')
    {
      break;
    }

    ++pos_;

    double rhs = parseFactor();

    if (op == '*')
    {
      value *= rhs;
    }
    else
    {
      if (rhs == 0.0)
      {
        throw std::runtime_error("Division by zero");
      }

      value /= rhs;
    }
  }

  return value;
}

double parseFactor()
{
  skipSpaces();

  // 括弧
  if (pos_ < expression_.size() &&
      expression_[pos_] == '(')
  {
    ++pos_;

    double value = parseExpression();

    skipSpaces();

    if (pos_ >= expression_.size() ||
        expression_[pos_] != ')')
    {
      throw std::runtime_error("Missing ')'");
    }

    ++pos_;

    return value;
  }

  // 符号
  if (pos_ < expression_.size() &&
      (expression_[pos_] == '+' || expression_[pos_] == '-'))
  {
    char sign = expression_[pos_++];
    double value = parseFactor();

    return sign == '-' ? -value : value;
  }

  // 数値
  skipSpaces();

  std::size_t start = pos_;

  while (pos_ < expression_.size() &&
         (std::isdigit(
              static_cast<unsigned char>(expression_[pos_])) ||
          expression_[pos_] == '.'))
  {
    ++pos_;
  }

  if (start == pos_)
  {
    throw std::runtime_error(
        "Expected number at position " +
        std::to_string(pos_));
  }

  try
  {
    return std::stod(expression_.substr(start, pos_ - start));
  }
  catch (const std::exception&)
  {
    throw std::runtime_error(
        "Invalid number at position " +
        std::to_string(start));
  }
}

const std::string& expression_;
std::size_t pos_;


};

// ------------------------------------------------------------
// "{xxx}" を Blackboard の値に置換する
//
// 例:
//
// Blackboard:
// data1 = 10
// data2 = 20
//
// 入力:
// "{data1} + {data2} + 0.1"
//
// 結果:
// "10 + 20 + 0.1"
// ------------------------------------------------------------
std::string replaceBlackboardValues(
const std::string& formula)
{
std::string result;

std::size_t pos = 0;

while (pos < formula.size())
{
  std::size_t open = formula.find('{', pos);

  // { がなければ残りをそのまま追加
  if (open == std::string::npos)
  {
    result += formula.substr(pos);
    break;
  }

  // { より前の部分
  result += formula.substr(pos, open - pos);

  std::size_t close = formula.find('}', open);

  if (close == std::string::npos)
  {
    throw RuntimeError(
        "[LocalBlackboardCaluculator] missing '}' in formula");
  }

  // {data1} の data1 部分
  std::string key =
      formula.substr(open + 1, close - open - 1);

  if (key.empty())
  {
    throw RuntimeError(
        "[LocalBlackboardCaluculator] empty blackboard key");
  }

  // Blackboard から double として取得
  double value = 0.0;

  if (!config().blackboard->get(key, value))
  {
    // int として保存されている場合
    int int_value = 0;

    if (config().blackboard->get(key, int_value))
    {
      value = static_cast<double>(int_value);
    }
    else
    {
      throw RuntimeError(
          "[LocalBlackboardCaluculator] blackboard key [" +
          key + "] not found or is not numeric");
    }
  }

  // 数値文字列に変換
  std::ostringstream oss;
  oss << value;

  result += oss.str();

  pos = close + 1;
}

return result;


}

virtual BT::NodeStatus tick() override
{
  std::string output_key;

  if (!getInput("output_key", output_key))
  {
    throw RuntimeError(
        "[LocalBlackboardCaluculator] missing port [output_key]");
  }

  std::string formula;

  if (!getInput("calculation_formula", formula))
  {
    throw RuntimeError(
        "[LocalBlackboardCaluculator] missing port "
        "[calculation_formula]");
  }

  std::string resolved_formula =
      replaceBlackboardValues(formula);

  double result = 0.0;

  try
  {
    ExpressionParser parser(resolved_formula);
    result = parser.parse();
  }
  catch (const std::exception& e)
  {
    throw RuntimeError(
        "[LocalBlackboardCaluculator] failed to calculate [" +
        formula + "]: " + e.what());
  }

  // answer というキーに double を保存
  config().blackboard->set(output_key, result);

  return NodeStatus::SUCCESS;
}

};

} // namespace BT

#endif // LOCAL_BLACKBOARD_CALUCULATOR_NODE_H