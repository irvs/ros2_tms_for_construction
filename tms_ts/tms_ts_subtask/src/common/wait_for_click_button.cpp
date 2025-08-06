#include "wait_for_click_node.h"

// Qt ヘッダ
#include <QApplication>
#include <QPushButton>
#include <QDialog>
#include <mutex>
#include <thread>

namespace BT
{

WaitForClick::WaitForClick(const std::string& name, const NodeConfiguration& config)
  : AsyncActionNode(name, config),
    clicked_(false)
{}

PortsList WaitForClick::providedPorts()
{
  // ポートは特になし
  return {};
}

NodeStatus WaitForClick::tick()
{
  // Qt のアプリケーションはプロセスにつき一度だけ生成
  static std::once_flag flag;
  static QApplication* app = nullptr;
  std::call_once(flag, [&](){
    int argc = 0;
    app = new QApplication(argc, nullptr);
  });

  clicked_ = false;

  // モーダルダイアログ＋ボタンを表示
  QDialog dialog;
  dialog.setWindowTitle(QString::fromStdString(name()));
  dialog.resize(200, 100);

  QPushButton button("Clik this button to continue this task.", &dialog);
  button.setGeometry(20, 30, 160, 40);

  QObject::connect(&button, &QPushButton::clicked, [&](){
    clicked_ = true;
    dialog.accept();  // ダイアログを閉じる
  });

  // この呼び出し中、別スレッドでは Qt のイベントループが回る
  dialog.exec();

  // クリックされたら SUCCESS、ダイアログを手動で閉じられたら FAILURE
  return clicked_ ? NodeStatus::SUCCESS
                  : NodeStatus::FAILURE;
}

void WaitForClick::halt()
{
  // AsyncActionNode 側の終了処理
  AsyncActionNode::halt();
}

} // namespace BT
