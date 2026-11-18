
#ifndef WAIT_FOR_CLICK_NODE_HPP
#define WAIT_FOR_CLICK_NODE_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include <gtk/gtk.h>
#include <string>

namespace BT
{

class WaitForClick : public SyncActionNode
{
public:

  WaitForClick(
      const std::string& name,
      const NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
    setRegistrationID("wait_for_click");
  }

  static PortsList providedPorts()
  {
    return {
      InputPort<std::string>(
          "button_info",
          "Press Yes to continue",
          "ダイアログ本文"),

      InputPort<std::string>(
          "window_title",
          "Confirm",
          "ウィンドウタイトル")
    };
  }

  // Blackboard変数を文字列に展開
  std::string replaceBlackboardVariables(
    const std::string& input)
  {
      std::string result = input;
      size_t start = 0;

      while ((start = result.find('{', start))
            != std::string::npos)
      {
          size_t end = result.find('}', start);

          if (end == std::string::npos)
              break;

          std::string key =
              result.substr(start + 1, end - start - 1);

          std::string value;

          auto entry = config().blackboard->getEntry(key);

          if (!entry)
          {
              g_warning("Blackboard key not found: %s",
                        key.c_str());
              return result;
          }

          try
          {
              config().blackboard->get(key, value);
          }
          catch (const std::exception& e)
          {
              g_warning("Blackboard get error: %s", e.what());
              return result;
          }

          g_print("Key: %s, Value: %s\n",
                  key.c_str(), value.c_str());

          result.replace(
              start,
              end - start + 1,
              value);

          start += value.length();
      }

      g_print("Final result: %s\n", result.c_str());

      return result;
  }

  NodeStatus tick() override
  {
    std::string text;
    std::string title;

    getInput("window_title", title);
    getInput("button_info", text);

    // Blackboard変数を展開
    text = replaceBlackboardVariables(text);
    title = replaceBlackboardVariables(title);

    // GTK初期化
    if (!gtk_init_check(nullptr, nullptr))
    {
      g_warning("GTK initialization failed");
      return NodeStatus::FAILURE;
    }

    // ダイアログ作成
    GtkWidget* dialog = gtk_dialog_new();

    gtk_window_set_title(
        GTK_WINDOW(dialog),
        title.c_str());

    gtk_window_set_default_size(
        GTK_WINDOW(dialog),
        800,
        250);

    gtk_window_set_resizable(
        GTK_WINDOW(dialog),
        FALSE);

    gtk_window_set_position(
        GTK_WINDOW(dialog),
        GTK_WIN_POS_CENTER);

    // 本文表示領域
    GtkWidget* content =
        gtk_dialog_get_content_area(
            GTK_DIALOG(dialog));

    GtkWidget* label =
        gtk_label_new(text.c_str());

    gtk_label_set_line_wrap(
        GTK_LABEL(label),
        TRUE);

    gtk_label_set_justify(
        GTK_LABEL(label),
        GTK_JUSTIFY_CENTER);

    gtk_widget_set_margin_top(label, 30);
    gtk_widget_set_margin_bottom(label, 30);
    gtk_widget_set_margin_start(label, 30);
    gtk_widget_set_margin_end(label, 30);

    gtk_box_pack_start(
        GTK_BOX(content),
        label,
        TRUE,
        TRUE,
        10);

    // Yes / No ボタン
    GtkWidget* yes_button =
        gtk_dialog_add_button(
            GTK_DIALOG(dialog),
            "Yes",
            GTK_RESPONSE_YES);

    GtkWidget* no_button =
        gtk_dialog_add_button(
            GTK_DIALOG(dialog),
            "No",
            GTK_RESPONSE_NO);

    // ボタンサイズ設定
    gtk_widget_set_size_request(
        yes_button,
        180,
        80);

    gtk_widget_set_size_request(
        no_button,
        180,
        80);

    // CSS設定
    GtkCssProvider* css_provider =
        gtk_css_provider_new();

    const gchar* css =
        "label {"
        "  font-size: 28pt;"
        "  font-weight: bold;"
        "}"
        "button {"
        "  font-size: 32pt;"
        "  font-weight: bold;"
        "  padding: 15px 35px;"
        "  min-width: 180px;"
        "  min-height: 80px;"
        "}";

    gtk_css_provider_load_from_data(
        css_provider,
        css,
        -1,
        nullptr);

    GdkScreen* screen =
        gdk_screen_get_default();

    gtk_style_context_add_provider_for_screen(
        screen,
        GTK_STYLE_PROVIDER(css_provider),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    // ダイアログ表示
    gtk_widget_show_all(dialog);

    // ボタン押下まで待機
    int response =
        gtk_dialog_run(GTK_DIALOG(dialog));

    // 後片付け
    gtk_widget_destroy(dialog);

    while (gtk_events_pending())
    {
      gtk_main_iteration();
    }

    g_object_unref(css_provider);

    // 結果判定
    if (response == GTK_RESPONSE_YES)
    {
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }
};

} // namespace BT

#endif // WAIT_FOR_CLICK_NODE_HPP
