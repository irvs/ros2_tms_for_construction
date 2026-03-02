#include <rclcpp/rclcpp.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/link_padding.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <geometric_shapes/shape_operations.h>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>

#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <atomic>

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

    model_name_ = this->get_parameter("model_name").as_string();
    root_record_name_ = this->get_parameter("root_record_name").as_string();
    planning_frame_ = this->get_parameter("planning_frame").as_string();

    if (model_name_.empty() || root_record_name_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "model_name or root_record_name is empty.");
      return;
    }

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