#ifndef WAIT_FOR_CLICK_NODE_HPP
#define WAIT_FOR_CLICK_NODE_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include <cstdlib>
#include <string>

namespace BT
{
// WaitForClick: zenity の情報ダイアログでメッセージを大きく表示し、Yes/No 2ボタンを表示。
// Yes → SUCCESS, No → FAILURE
class WaitForClick : public SyncActionNode
{
public:
  WaitForClick(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
    setRegistrationID("wait_for_click");
  }

  static PortsList providedPorts()
  {
    return {
      // button_text は中央に大きく表示するメッセージ
      InputPort<std::string>("button_info",  "Press Yes to continue", "ダイアログ本文（大きな文字）"),
      // window_title はウィンドウのタイトル
      InputPort<std::string>("window_title", "Confirm",  "ウィンドウタイトル")
    };
  }

  NodeStatus tick() override
  {
    std::string text, title;
    getInput("window_title", title);
    getInput("button_info",  text);

    // Pango マークアップで大きい文字に
    std::string markup = "<span size='x-large'>" + text + "</span>";

    // zenity コマンド組み立て
    std::string cmd =
      "zenity --info"
      " --title=\"" + title + "\""
      " --text=\""  + markup + "\""
      " --width=800"
      " --height=200"
      " --ok-label=\"Yes\""
      " --extra-button=\"No\"";

    // 実行（ブロック）
    int ret = std::system(cmd.c_str());

    // 0: Yes → SUCCESS, 1: No → FAILURE （その他も FAILURE 扱い）
    return (ret == 0)
         ? NodeStatus::SUCCESS
         : NodeStatus::FAILURE;
  }
};

} // namespace BT

#endif // WAIT_FOR_CLICK_NODE_HPP
