#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/link_padding.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <geometric_shapes/shape_operations.h>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>

#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <atomic>
#include <cmath>

#include "tms_ts_primitive/primitive_node_base.hpp"

class ExcavatorSceneManager : public PrimitiveNodeBase
{
public:
  ExcavatorSceneManager()
  : PrimitiveNodeBase("excavator_scene_manager")
  {
    this->declare_parameter<std::string>("model_name", "zx200");
    this->declare_parameter<std::string>("root_record_name", "");
    this->declare_parameter<std::string>("planning_frame", "base_link");
    this->declare_parameter<std::string>("visualization_record_name", "");

    model_name_ = this->get_parameter("model_name").as_string();
    root_record_name_ = this->get_parameter("root_record_name").as_string();
    planning_frame_ = this->get_parameter("planning_frame").as_string();
    visualization_record_name_ = this->get_parameter("visualization_record_name").as_string();

    if (model_name_.empty() || root_record_name_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "model_name or root_record_name is empty.");
      return;
    }

    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "excavatable_markers", 10);

    apply_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "tms_rp_excavator_apply_planning_scene");

    if (!apply_client_->wait_for_service(std::chrono::seconds(20))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available: tms_rp_excavator_apply_planning_scene");
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]()
      {
        if (in_flight_.exchange(true)) {
          RCLCPP_WARN(this->get_logger(), "Previous ApplyPlanningScene still running; skip this tick.");
          return;
        }

        try {
          auto scene = buildPlanningSceneFromDb(model_name_, root_record_name_);
          applyPlanningSceneAsync(scene);
          publishVisualizationMarkers();
        } catch (const std::exception& e) {
          in_flight_ = false;
          RCLCPP_ERROR(this->get_logger(), "Update failed: %s", e.what());
        }
      });
  }

private:
  static std::optional<bsoncxx::document::value> tryParseJsonDoc(const std::string& json)
  {
    try {
      return bsoncxx::from_json(json);
    } catch (...) {
      return std::nullopt;
    }
  }

  template <class ElemT>
  static double readNumberAsDouble(const ElemT& e)
  {
    if (e.type() == bsoncxx::type::k_double) return e.get_double().value;
    if (e.type() == bsoncxx::type::k_int32)  return static_cast<double>(e.get_int32().value);
    if (e.type() == bsoncxx::type::k_int64)  return static_cast<double>(e.get_int64().value);
    throw std::runtime_error("Unsupported numeric type");
  }

  moveit_msgs::msg::PlanningScene buildPlanningSceneFromDb(
    const std::string& model_name,
    const std::string& root_record_name)
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;

    const auto root_map = this->GetParamFromDBAsJson(model_name, root_record_name);
    if (root_map.empty()) throw std::runtime_error("Root record not found");

    auto root_lp_it = root_map.find("link_padding");
    if (root_lp_it != root_map.end()) {
      auto lp_doc_opt = tryParseJsonDoc(root_lp_it->second);
      if (!lp_doc_opt) throw std::runtime_error("Failed to parse link_padding");

      auto lp_elem = lp_doc_opt->view()["link_padding"];
      if (!lp_elem || lp_elem.type() != bsoncxx::type::k_document)
        throw std::runtime_error("link_padding invalid");

      auto lp_view = lp_elem.get_document().view();
      for (auto&& kv : lp_view) {
        moveit_msgs::msg::LinkPadding p;
        p.link_name = std::string(kv.key());
        p.padding = readNumberAsDouble(kv);
        scene.link_padding.push_back(p);
      }
    }

    auto root_arr_it = root_map.find("collision_object_record_names");
    if (root_arr_it == root_map.end())
      throw std::runtime_error("collision_object_record_names missing");

    auto arr_doc_opt = tryParseJsonDoc(root_arr_it->second);
    if (!arr_doc_opt)
      throw std::runtime_error("Failed to parse collision_object_record_names");

    auto arr_elem = arr_doc_opt->view()["collision_object_record_names"];
    if (!arr_elem || arr_elem.type() != bsoncxx::type::k_array)
      throw std::runtime_error("collision_object_record_names invalid");

    for (auto&& name_elem : arr_elem.get_array().value) {
      if (name_elem.type() != bsoncxx::type::k_utf8) continue;

      const std::string child_record_name = name_elem.get_utf8().value.to_string();
      const auto child_map = this->GetParamFromDBAsJson(model_name, child_record_name);
      if (child_map.empty()) continue;

      std::string type;
      auto type_it = child_map.find("type");
      if (type_it != child_map.end()) {
        auto type_doc_opt = tryParseJsonDoc(type_it->second);
        if (type_doc_opt) {
          auto type_elem = type_doc_opt->view()["type"];
          if (type_elem && type_elem.type() == bsoncxx::type::k_utf8)
            type = type_elem.get_utf8().value.to_string();
        }
      }

      if (type == "mesh")
        addMeshFromMap(scene, child_map, child_record_name);
      else
        addPrimitiveFromMap(scene, child_map, child_record_name);
    }

    return scene;
  }

  void addPrimitiveFromMap(moveit_msgs::msg::PlanningScene& scene,
                           const std::map<std::string, std::string>& m,
                           const std::string& record_name)
  {
    auto pt_doc = bsoncxx::from_json(m.at("primitive_type"));
    auto dim_doc = bsoncxx::from_json(m.at("dimensions"));
    auto pose_doc = bsoncxx::from_json(m.at("pose"));

    auto pt_elem = pt_doc.view()["primitive_type"];
    if (!pt_elem) throw std::runtime_error("primitive_type missing");

    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = planning_frame_;
    co.id = "excavator_scene/" + record_name;

    shape_msgs::msg::SolidPrimitive prim;
    if (pt_elem.type() == bsoncxx::type::k_int32)
      prim.type = pt_elem.get_int32().value;
    else if (pt_elem.type() == bsoncxx::type::k_int64)
      prim.type = static_cast<int32_t>(pt_elem.get_int64().value);
    else
      throw std::runtime_error("primitive_type invalid");

    auto dim_elem = dim_doc.view()["dimensions"];
    if (!dim_elem || dim_elem.type() != bsoncxx::type::k_array)
      throw std::runtime_error("dimensions invalid");

    for (auto&& d : dim_elem.get_array().value)
      prim.dimensions.push_back(readNumberAsDouble(d));

    auto pose_elem = pose_doc.view()["pose"];
    if (!pose_elem || pose_elem.type() != bsoncxx::type::k_document)
      throw std::runtime_error("pose invalid");

    auto pv = pose_elem.get_document().view();
    auto pos_e = pv["position"];
    auto ori_e = pv["orientation"];
    if (!pos_e || pos_e.type() != bsoncxx::type::k_document ||
        !ori_e || ori_e.type() != bsoncxx::type::k_document)
      throw std::runtime_error("pose.position/orientation invalid");

    auto pos = pos_e.get_document().view();
    auto ori = ori_e.get_document().view();

    geometry_msgs::msg::Pose pose;
    pose.position.x = readNumberAsDouble(pos["x"]);
    pose.position.y = readNumberAsDouble(pos["y"]);
    pose.position.z = readNumberAsDouble(pos["z"]);
    pose.orientation.x = readNumberAsDouble(ori["x"]);
    pose.orientation.y = readNumberAsDouble(ori["y"]);
    pose.orientation.z = readNumberAsDouble(ori["z"]);
    pose.orientation.w = readNumberAsDouble(ori["w"]);

    co.primitives.push_back(prim);
    co.primitive_poses.push_back(pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    scene.world.collision_objects.push_back(co);
  }

  void addMeshFromMap(moveit_msgs::msg::PlanningScene& scene,
                      const std::map<std::string, std::string>& m,
                      const std::string& record_name)
  {
    auto data_doc = bsoncxx::from_json(m.at("data"));
    auto bin_elem = data_doc.view()["data"];
    if (!bin_elem || bin_elem.type() != bsoncxx::type::k_binary)
      throw std::runtime_error("data is not binary");

    auto bin = bin_elem.get_binary();

    auto x_doc  = bsoncxx::from_json(m.at("x"));
    auto y_doc  = bsoncxx::from_json(m.at("y"));
    auto z_doc  = bsoncxx::from_json(m.at("z"));
    auto qx_doc = bsoncxx::from_json(m.at("qx"));
    auto qy_doc = bsoncxx::from_json(m.at("qy"));
    auto qz_doc = bsoncxx::from_json(m.at("qz"));
    auto qw_doc = bsoncxx::from_json(m.at("qw"));

    geometry_msgs::msg::Pose pose;
    pose.position.x = readNumberAsDouble(x_doc.view()["x"]);
    pose.position.y = readNumberAsDouble(y_doc.view()["y"]);
    pose.position.z = readNumberAsDouble(z_doc.view()["z"]);
    pose.orientation.x = readNumberAsDouble(qx_doc.view()["qx"]);
    pose.orientation.y = readNumberAsDouble(qy_doc.view()["qy"]);
    pose.orientation.z = readNumberAsDouble(qz_doc.view()["qz"]);
    pose.orientation.w = readNumberAsDouble(qw_doc.view()["qw"]);

    const std::string temp_path = "/tmp/temp_dump_mesh_" + record_name + ".dae";
    {
      std::ofstream ofs(temp_path, std::ios::binary);
      ofs.write(reinterpret_cast<const char*>(bin.bytes), bin.size);
    }

    shapes::Mesh* mesh = shapes::createMeshFromResource("file://" + temp_path);
    if (!mesh) throw std::runtime_error("Mesh load failed");

    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = planning_frame_;
    co.id = "excavator_scene/" + record_name + "_mesh";
    co.meshes.push_back(mesh_msg);
    co.mesh_poses.push_back(pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    scene.world.collision_objects.push_back(co);
  }

  visualization_msgs::msg::MarkerArray buildVisualizationMarkersFromDb()
  {
    visualization_msgs::msg::MarkerArray markers;
    if (visualization_record_name_.empty()) {
      return markers;
    }

    const auto viz_record = this->GetParamFromDBAsJson(model_name_, visualization_record_name_);
    if (viz_record.empty()) {
      return markers;
    }

    auto makeMarkerHeader = [this]() {
      std_msgs::msg::Header header;
      header.frame_id = planning_frame_;
      header.stamp = this->now();
      return header;
    };

    auto makeAreaMarker = [&](const bsoncxx::document::view& area_data) {
      auto x = area_data["x"];
      auto y = area_data["y"];
      auto z = area_data["z"];
      auto size_x = area_data["size_x"];
      auto size_y = area_data["size_y"];
      auto size_z = area_data["size_z"];
      if (!x || !y || !z || !size_x || !size_y || !size_z) {
        return;
      }

      geometry_msgs::msg::Pose pose;
      pose.position.x = readNumberAsDouble(x) + readNumberAsDouble(size_x) * 0.5;
      pose.position.y = readNumberAsDouble(y) + readNumberAsDouble(size_y) * 0.5;
      pose.position.z = readNumberAsDouble(z) + readNumberAsDouble(size_z) * 0.5;
      // double theta_w = 0.0;
      // auto theta_elem = area_data["theta_w"];
      // if (theta_elem && (theta_elem.type() == bsoncxx::type::k_double || theta_elem.type() == bsoncxx::type::k_int32 || theta_elem.type() == bsoncxx::type::k_int64)) {
      //   theta_w = readNumberAsDouble(theta_elem);
      // }
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      // pose.orientation.z = std::sin(theta_w * 0.5);
      // pose.orientation.w = std::cos(theta_w * 0.5);
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;

      visualization_msgs::msg::Marker marker;
      marker.header = makeMarkerHeader();
      marker.ns = "excavation_area";
      marker.id = static_cast<int>(markers.markers.size());
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;
      marker.scale.x = readNumberAsDouble(size_x);
      marker.scale.y = readNumberAsDouble(size_y);
      marker.scale.z = readNumberAsDouble(size_z);
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.5f;
      marker.lifetime = rclcpp::Duration::from_seconds(1.2);
      marker.frame_locked = false;
      markers.markers.push_back(marker);
    };

    auto addPointsMarker = [&](const bsoncxx::array::view& points_array,
                                const std::string& marker_ns,
                                float r,
                                float g,
                                float b,
                                float a) {
      visualization_msgs::msg::Marker marker;
      marker.header = makeMarkerHeader();
      marker.ns = marker_ns;
      marker.id = static_cast<int>(markers.markers.size());
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.lifetime = rclcpp::Duration::from_seconds(1.2);
      marker.frame_locked = false;

      for (auto&& point_elem : points_array) {
        if (point_elem.type() != bsoncxx::type::k_document) {
          continue;
        }
        auto point_doc = point_elem.get_document().value;
        if (!point_doc["x"] || !point_doc["y"] || !point_doc["z"]) {
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = readNumberAsDouble(point_doc["x"]);
        point.y = readNumberAsDouble(point_doc["y"]);
        point.z = readNumberAsDouble(point_doc["z"]);
        marker.points.push_back(point);
      }

      if (!marker.points.empty()) {
        markers.markers.push_back(marker);
      }
    };

    auto waypoints_it = viz_record.find("waypoints");
    if (waypoints_it != viz_record.end()) {
      auto doc_opt = tryParseJsonDoc(waypoints_it->second);
      if (doc_opt) {
        auto waypoints_elem = doc_opt->view()["waypoints"];
        if (waypoints_elem && waypoints_elem.type() == bsoncxx::type::k_array) {
          for (auto&& waypoint : waypoints_elem.get_array().value) {
            if (waypoint.type() != bsoncxx::type::k_document) {
              continue;
            }
            auto waypoint_doc = waypoint.get_document().value;
            auto type_elem = waypoint_doc["type"];
            if (!type_elem || type_elem.type() != bsoncxx::type::k_utf8) {
              continue;
            }
            if (type_elem.get_utf8().value.to_string() != "area") {
              continue;
            }
            auto data_elem = waypoint_doc["data"];
            if (!data_elem || data_elem.type() != bsoncxx::type::k_document) {
              continue;
            }
            makeAreaMarker(data_elem.get_document().value);
            break;
          }
        }
      }
    }

    auto points_it = viz_record.find("excavatable_points");
    if (points_it != viz_record.end()) {
      auto doc_opt = tryParseJsonDoc(points_it->second);
      if (doc_opt) {
        auto points_elem = doc_opt->view()["excavatable_points"];
        if (points_elem && points_elem.type() == bsoncxx::type::k_array) {
          addPointsMarker(points_elem.get_array().value, "reachable_points", 1.0f, 0.0f, 0.0f, 1.0f);
        }
      }
    }

    // Comment out the following block to disable ik_pass_points visualization.
    // auto ik_points_it = viz_record.find("ik_pass_points");
    // if (ik_points_it != viz_record.end()) {
    //   auto doc_opt = tryParseJsonDoc(ik_points_it->second);
    //   if (doc_opt) {
    //     auto points_elem = doc_opt->view()["ik_pass_points"];
    //     if (points_elem && points_elem.type() == bsoncxx::type::k_array) {
    //       addPointsMarker(points_elem.get_array().value, "ik_pass_points", 0.0f, 0.0f, 1.0f, 1.0f);
    //     }
    //   }
    // }

    return markers;
  }

  void publishVisualizationMarkers()
  {
    if (visualization_record_name_.empty()) {
      return;
    }

    auto markers = buildVisualizationMarkersFromDb();
    if (!markers.markers.empty()) {
      visualization_pub_->publish(markers);
    }
  }

  void applyPlanningSceneAsync(const moveit_msgs::msg::PlanningScene& scene)
  {
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene = scene;

    apply_client_->async_send_request(
      req,
      [this](rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedFuture future)
      {
        try {
          const bool ok = future.get()->success;
          RCLCPP_INFO(this->get_logger(), "Scene reapplied: %s", ok ? "success" : "failure");
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "ApplyPlanningScene callback exception: %s", e.what());
        }
        in_flight_ = false;
      });
  }

private:
  std::string model_name_;
  std::string root_record_name_;
  std::string planning_frame_;
  std::string visualization_record_name_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic_bool in_flight_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavatorSceneManager>());
  rclcpp::shutdown();
  return 0;
}