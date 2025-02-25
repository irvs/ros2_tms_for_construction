#ifndef SUBTASK_NODE_BASE_HPP
#define SUBTASK_NODE_BASE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <map>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tms_msg_db/srv/tmsdb_get_parameter.hpp"

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

class SubtaskNodeBase : public rclcpp::Node
{
public:
  /// @brief コンストラクタ
  explicit SubtaskNodeBase(const std::string &node_name_);

  /**
   * @brief 1次元のパラメータ（キー=std::string）を同期的に取得 (戻り値を std::map で返す)
   *
   * @tparam K = std::string
   * @tparam T = double, int, std::string など
   * @param model_name
   * @param record_name
   * @return std::map<K, T>
   */
//   template <typename K, typename T>
//   std::map<K, T> CustomGetParamFromDB(
//       const std::string &model_name,
//       const std::string &record_name,
//       std::enable_if_t<std::is_same_v<K, std::string>, bool> = true)
//   {
//     auto request = std::make_shared<tms_msg_db::srv::TmsdbGetParameter::Request>();
//     request->model_name = model_name;
//     request->record_name = record_name;

//     // サービスが起動するまで待機
//     while (!client_->wait_for_service(std::chrono::seconds(3))) {
//       RCLCPP_WARN(this->get_logger(), "[%s] Waiting for service to become available...",
//                   this->get_name());
//     }

//     // サービスコール (非同期だが、サーバは別プロセスなので自前のspin_some()は不要)
//     auto future = client_->async_send_request(request);
//     std::map<K, T> dataMap;
//     RCLCPP_INFO(this->get_logger(), "BBBBBBBBBBBBBBBBBBBBBBBBB (1D param request sent)");

//     try {
//       // 待機（10秒以内にレスポンスが来るかチェック）
//       // auto status = future.wait_for(std::chrono::seconds(10)); // spinで実行される部分までブロッキングされてしまう
//       auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future); 

//       if (status == rclcpp::FutureReturnCode::SUCCESS) {
//         auto response = future.get();
//         RCLCPP_INFO(this->get_logger(), "AAAAAAAAAAAAAAAAAAAAAAA (1D param response arrived)");

//         // map に変換
//         for (size_t i = 0; i < response->keys.size(); ++i) {
//           const std::string &key_str = response->keys[i];
//           const std::string &value_str = response->values[i];

//           if constexpr (std::is_same_v<T, double>) {
//             dataMap[key_str] = std::stod(value_str);
//           } else if constexpr (std::is_same_v<T, int>) {
//             dataMap[key_str] = std::stoi(value_str);
//           } else {
//             dataMap[key_str] = value_str;
//           }
//         }
//         RCLCPP_INFO(this->get_logger(), "[%s] Received parameters from service (1D).", this->get_name());
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "[%s] Service call timeout or not ready (1D).", this->get_name());
//       }
//     }
//     catch (const std::exception &e) {
//       RCLCPP_ERROR(this->get_logger(), "[%s] Service call failed: %s", this->get_name(), e.what());
//     }

//     return dataMap;
//   }

// subtask_node_base.hpp 内の例
template <typename K, typename T, typename CallbackT,
          typename = std::enable_if_t<std::is_same_v<K, std::string>>>
void CustomGetParamFromDB(
  const std::string &model_name,
  const std::string &record_name,
  CallbackT callback)
{
  // サービスリクエストの作成
  auto request = std::make_shared<tms_msg_db::srv::TmsdbGetParameter::Request>();
  request->model_name  = model_name;
  request->record_name = record_name;

  // サービス利用可能になるまで待機（ここではwait_for_serviceは使用しますが、spin_until_future_completeなどは使いません）
  while (!client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_WARN(this->get_logger(), "[%s] サービスが利用可能になるのを待っています...", this->get_name());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 非同期リクエスト送信
  auto future = client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "サービスリクエストを送信しました");

  // 別スレッドで future の完了を待機し、結果をコールバックで返す
  std::thread([this, future, callback]() mutable {
    try {
      // future.get() はこのスレッド内でブロッキングしても問題ない
      auto response = future.get();

      // レスポンスから map 型に変換
      std::map<std::string, double> dataMap;
      for (size_t i = 0; i < response->keys.size(); ++i) {
        const std::string &key_str   = response->keys[i];
        const std::string &value_str = response->values[i];
        dataMap[key_str] = std::stod(value_str);
      }

      // コールバックを呼び出して結果を渡す
      callback(dataMap);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "サービスコールに失敗しました: %s", e.what());
      // エラー時もコールバックを呼び出す場合は、空の map やエラー用の値を渡すなどの処理を追加してください
    }
  }).detach();
}





  /**
   * @brief 2次元パラメータ（キー = std::pair<std::string, std::string>）を同期的に取得
   * @tparam K = std::pair<std::string, std::string>
   * @tparam T = double, int, std::string ...
   */
  template <typename K, typename T>
  std::map<K, T> CustomGetParamFromDB(
      const std::string &model_name,
      const std::string &record_name,
      std::enable_if_t<std::is_same_v<K, std::pair<std::string, std::string>>, bool> = true)
  {
    auto request = std::make_shared<tms_msg_db::srv::TmsdbGetParameter::Request>();
    request->model_name = model_name;
    request->record_name = record_name;

    while (!client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_WARN(this->get_logger(), "[%s] Waiting for service to become available...",
                  this->get_name());
    }

    auto future = client_->async_send_request(request);
    std::map<K, T> dataMap;

    auto start = std::chrono::steady_clock::now();
    bool ready = false;
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
      if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        ready = true;
        break;
      }
      // ※ 同じノードのspin_some()呼び出しは削除（エグゼキュータ登録の二重化を防止）
    }

    if (ready) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "[%s] Received service response (2D param).", this->get_name());

      std::string last_key;
      int index_count = 0;

      for (size_t i = 0; i < response->keys.size(); ++i) {
        const std::string &current_key = response->keys[i];
        const std::string &value_str = response->values[i];

        if (i == 0) {
          last_key = current_key;
          index_count = 0;
        } else {
          if (current_key == last_key) {
            index_count++;
          } else {
            last_key = current_key;
            index_count = 0;
          }
        }

        K key_pair{ current_key, std::to_string(index_count) };

        if constexpr (std::is_same_v<T, double>) {
          dataMap[key_pair] = std::stod(value_str);
        } else if constexpr (std::is_same_v<T, int>) {
          dataMap[key_pair] = std::stoi(value_str);
        } else {
          dataMap[key_pair] = value_str;
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "[%s] Service call timeout or not ready (2D).", this->get_name());
    }

    return dataMap;
  }

private:
  // MongoDB C++ ドライバ用インスタンス (グローバルに一回だけ初期化)
  static mongocxx::instance inst;

protected:
  // TmsdbGetParameter 用のクライアント
  rclcpp::Client<tms_msg_db::srv::TmsdbGetParameter>::SharedPtr client_;
};

#endif // SUBTASK_NODE_BASE_HPP
