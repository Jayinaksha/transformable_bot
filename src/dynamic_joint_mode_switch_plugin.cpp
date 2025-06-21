#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

class DynamicJointModeSwitchPlugin : public gz::sim::System,
                                    public gz::sim::ISystemConfigure,
                                    public gz::sim::ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  
  std::string arm_name_;
  std::string service_name_;
  
  // Link names
  std::string base_link_name_;
  std::string hub9rods_link_name_; 
  std::string motor_link_name_;
  
  // Link entities
  gz::sim::Entity base_link_entity_;
  gz::sim::Entity hub9rods_link_entity_;
  gz::sim::Entity motor_link_entity_;
  gz::sim::Entity model_entity_;
  
  // Dynamic joint entities
  gz::sim::Entity active_joint_entity_;
  
  bool drone_mode_ = false;
  bool links_found_ = false;
  bool mode_change_requested_ = false;
  bool new_mode_state_ = true;
  bool link_search_completed_ = false;
  
  static int instance_counter_;
  int my_instance_id_;
  int search_attempts_ = 0;
  const int MAX_SEARCH_ATTEMPTS = 50;
  
  // Retry timing
  std::chrono::steady_clock::time_point last_search_time_;
  const std::chrono::milliseconds SEARCH_INTERVAL{100};

public:
  DynamicJointModeSwitchPlugin() {
    my_instance_id_ = ++instance_counter_;
    last_search_time_ = std::chrono::steady_clock::now();
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &/*eventMgr*/) override
  {
    model_entity_ = entity;
    
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    std::string node_name = "dynamic_joint_mode_" + std::to_string(my_instance_id_);
    node_ = rclcpp::Node::make_shared(node_name);
    
    // Read SDF parameters
    if (sdf && sdf->HasElement("arm_name")) {
      arm_name_ = sdf->Get<std::string>("arm_name");
      RCLCPP_INFO(node_->get_logger(), "✅ Read arm_name from SDF: '%s'", arm_name_.c_str());
    } else {
      arm_name_ = "arm_" + std::to_string(my_instance_id_);
      RCLCPP_WARN(node_->get_logger(), "⚠️ No arm_name in SDF, defaulting to: '%s'", arm_name_.c_str());
    }
    
    if (sdf && sdf->HasElement("service_name")) {
      service_name_ = sdf->Get<std::string>("service_name");
    } else {
      service_name_ = "/robot/" + arm_name_ + "/mode_switch";
    }
    
    // Construct link names
    base_link_name_ = "base_link";
    hub9rods_link_name_ = "ring_attach_" + arm_name_ + "_hub_9rods";
    motor_link_name_ = arm_name_ + "_motor";
    
    // Create ROS service
    service_ = node_->create_service<std_srvs::srv::SetBool>(
      service_name_,
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        if (!links_found_) {
          response->success = false;
          response->message = arm_name_ + " not ready - links still loading";
          RCLCPP_WARN(node_->get_logger(), "⚠️ %s: Mode switch requested but links not ready yet", arm_name_.c_str());
          return;
        }
        
        mode_change_requested_ = true;
        new_mode_state_ = request->data;
        response->success = true;
        
        if (request->data) {
          response->message = arm_name_ + " DRONE mode - direct ECM joint: base_link <-> hub_9rods";
          RCLCPP_INFO(node_->get_logger(), "🚁 %s: DRONE MODE requested", arm_name_.c_str());
        } else {
          response->message = arm_name_ + " ROVER mode - direct ECM joint: hub_9rods <-> motor";
          RCLCPP_INFO(node_->get_logger(), "🚗 %s: ROVER MODE requested", arm_name_.c_str());
        }
        
        auto mode_msg = std_msgs::msg::String();
        mode_msg.data = request->data ? "drone" : "rover";
        mode_publisher_->publish(mode_msg);
      });
    
    mode_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/robot/" + arm_name_ + "/current_mode", 10);
    
    RCLCPP_INFO(node_->get_logger(), 
                "🔧 Direct ECM Joint Mode Switch Plugin loaded for %s", 
                arm_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), 
                "   Looking for links: %s, %s, %s", 
                base_link_name_.c_str(), hub9rods_link_name_.c_str(), motor_link_name_.c_str());
  }

  void PreUpdate(const gz::sim::UpdateInfo &/*info*/,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    }
    
    // Search for links
    if (!links_found_ && !link_search_completed_) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_search_time_ >= SEARCH_INTERVAL) {
        findLinks(ecm);
        last_search_time_ = now;
      }
    }
    
    // Handle mode change requests
    if (mode_change_requested_ && links_found_) {
      if (new_mode_state_) {
        createDroneJointDirect(ecm);
      } else {
        createRoverJointDirect(ecm);
      }
      drone_mode_ = new_mode_state_;
      mode_change_requested_ = false;
    }
  }

private:
  void findLinks(gz::sim::EntityComponentManager &ecm)
  {
    search_attempts_++;
    
    if (search_attempts_ > MAX_SEARCH_ATTEMPTS) {
      link_search_completed_ = true;
      RCLCPP_ERROR(node_->get_logger(), 
                   "❌ %s: Link search FAILED after %d attempts", 
                   arm_name_.c_str(), MAX_SEARCH_ATTEMPTS);
      printAllLinksForDebugging(ecm);
      return;
    }
    
    auto links = ecm.EntitiesByComponents(gz::sim::components::Link());
    
    base_link_entity_ = gz::sim::kNullEntity;
    hub9rods_link_entity_ = gz::sim::kNullEntity; 
    motor_link_entity_ = gz::sim::kNullEntity;
    
    bool found_base = false;
    bool found_hub9rods = false;
    bool found_motor = false;
    
    for (auto link : links) {
      auto nameComp = ecm.Component<gz::sim::components::Name>(link);
      if (!nameComp) continue;
      
      std::string link_name = nameComp->Data();
      
      if (link_name == base_link_name_) {
        base_link_entity_ = link;
        found_base = true;
      }
      else if (link_name == hub9rods_link_name_) {
        hub9rods_link_entity_ = link;
        found_hub9rods = true;
      }
      else if (link_name == motor_link_name_) {
        motor_link_entity_ = link;
        found_motor = true;
      }
    }
    
    int found_count = (found_base ? 1 : 0) + (found_hub9rods ? 1 : 0) + (found_motor ? 1 : 0);
    
    if (found_count == 3) {
      links_found_ = true;
      link_search_completed_ = true;
      
      auto baseNameComp = ecm.Component<gz::sim::components::Name>(base_link_entity_);
      auto hubNameComp = ecm.Component<gz::sim::components::Name>(hub9rods_link_entity_);
      auto motorNameComp = ecm.Component<gz::sim::components::Name>(motor_link_entity_);
      
      RCLCPP_INFO(node_->get_logger(), 
                  "🎯 %s: All 3 links found! Ready for direct ECM joint creation.", 
                  arm_name_.c_str());
      RCLCPP_INFO(node_->get_logger(), "   Actual link names found:");
      RCLCPP_INFO(node_->get_logger(), "   ✅ Base: '%s'", baseNameComp ? baseNameComp->Data().c_str() : "NULL");
      RCLCPP_INFO(node_->get_logger(), "   ✅ Hub: '%s'", hubNameComp ? hubNameComp->Data().c_str() : "NULL");
      RCLCPP_INFO(node_->get_logger(), "   ✅ Motor: '%s'", motorNameComp ? motorNameComp->Data().c_str() : "NULL");
    } else if (search_attempts_ % 10 == 1) {
      RCLCPP_WARN(node_->get_logger(), 
                  "⚠️ %s: Found %d/3 links (attempt %d/%d)", 
                  arm_name_.c_str(), found_count, search_attempts_, MAX_SEARCH_ATTEMPTS);
    }
  }
  
  void printAllLinksForDebugging(gz::sim::EntityComponentManager &ecm)
  {
    RCLCPP_ERROR(node_->get_logger(), "=== 🔍 ALL ROBOT LINKS FOR %s ===", arm_name_.c_str());
    auto links = ecm.EntitiesByComponents(gz::sim::components::Link());
    
    std::vector<std::string> all_links;
    for (auto link : links) {
      auto nameComp = ecm.Component<gz::sim::components::Name>(link);
      if (nameComp) {
        all_links.push_back(nameComp->Data());
      }
    }
    
    std::sort(all_links.begin(), all_links.end());
    
    RCLCPP_ERROR(node_->get_logger(), "📋 ALL LINKS FOUND (%zu total):", all_links.size());
    for (const auto& link : all_links) {
      if (link.find(arm_name_) != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "  🎯 %s (ARM RELATED)", link.c_str());
      } else if (link.find("ring_attach") != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "  🔗 %s (RING SYSTEM)", link.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "  - %s", link.c_str());
      }
    }
    RCLCPP_ERROR(node_->get_logger(), "=== 🔍 END LINK DEBUG ===");
  }
  
  void createDroneJointDirect(gz::sim::EntityComponentManager &ecm)
  {
    RCLCPP_INFO(node_->get_logger(), "🔧 %s: Creating DRONE joint using direct ECM method...", arm_name_.c_str());
    
    destroyActiveJoint(ecm);
    
    // Get actual link names
    auto baseNameComp = ecm.Component<gz::sim::components::Name>(base_link_entity_);
    auto hubNameComp = ecm.Component<gz::sim::components::Name>(hub9rods_link_entity_);
    
    if (!baseNameComp || !hubNameComp) {
      RCLCPP_ERROR(node_->get_logger(), "❌ %s: Cannot get link names for DRONE joint", arm_name_.c_str());
      return;
    }
    
    // CREATE JOINT DIRECTLY USING ECM - NO SdfEntityCreator!
    active_joint_entity_ = ecm.CreateEntity();
    
    // Add joint components directly
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::Name(arm_name_ + "_drone_mode_joint"));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::Joint());
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::JointType(sdf::JointType::FIXED));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::ParentEntity(model_entity_));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::ParentLinkName(baseNameComp->Data()));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::ChildLinkName(hubNameComp->Data()));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::Pose(gz::math::Pose3d::Zero));
    
    RCLCPP_INFO(node_->get_logger(), 
                "🔗 %s: Created DRONE joint using direct ECM: %s <-> %s", 
                arm_name_.c_str(), baseNameComp->Data().c_str(), hubNameComp->Data().c_str());
  }
  
  void createRoverJointDirect(gz::sim::EntityComponentManager &ecm)
  {
    RCLCPP_INFO(node_->get_logger(), "🔧 %s: Creating ROVER joint using direct ECM method...", arm_name_.c_str());
    
    destroyActiveJoint(ecm);
    
    // Get actual link names
    auto hubNameComp = ecm.Component<gz::sim::components::Name>(hub9rods_link_entity_);
    auto motorNameComp = ecm.Component<gz::sim::components::Name>(motor_link_entity_);
    
    if (!hubNameComp || !motorNameComp) {
      RCLCPP_ERROR(node_->get_logger(), "❌ %s: Cannot get link names for ROVER joint", arm_name_.c_str());
      return;
    }
    
    // CREATE JOINT DIRECTLY USING ECM - NO SdfEntityCreator!
    active_joint_entity_ = ecm.CreateEntity();
    
    // Add joint components directly
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::Name(arm_name_ + "_rover_mode_joint"));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::Joint());
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::JointType(sdf::JointType::FIXED));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::ParentEntity(model_entity_));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::ParentLinkName(hubNameComp->Data()));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::ChildLinkName(motorNameComp->Data()));
    ecm.CreateComponent(active_joint_entity_, 
                       gz::sim::components::Pose(gz::math::Pose3d::Zero));
    
    RCLCPP_INFO(node_->get_logger(), 
                "🔗 %s: Created ROVER joint using direct ECM: %s <-> %s", 
                arm_name_.c_str(), hubNameComp->Data().c_str(), motorNameComp->Data().c_str());
  }
  
  void destroyActiveJoint(gz::sim::EntityComponentManager &ecm)
  {
    if (active_joint_entity_ != gz::sim::kNullEntity) {
      RCLCPP_INFO(node_->get_logger(), "🗑️ %s: Destroying previous joint...", arm_name_.c_str());
      ecm.RequestRemoveEntity(active_joint_entity_);
      active_joint_entity_ = gz::sim::kNullEntity;
    }
  }
};

int DynamicJointModeSwitchPlugin::instance_counter_ = 0;

// Register the plugin
GZ_ADD_PLUGIN(DynamicJointModeSwitchPlugin,
              gz::sim::System,
              DynamicJointModeSwitchPlugin::ISystemConfigure,
              DynamicJointModeSwitchPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DynamicJointModeSwitchPlugin, "dynamic_joint_mode_switch_plugin")
GZ_ADD_PLUGIN_ALIAS(DynamicJointModeSwitchPlugin, "mode_switch_plugin")
GZ_ADD_PLUGIN_ALIAS(DynamicJointModeSwitchPlugin, "front_right_mode_switch")
GZ_ADD_PLUGIN_ALIAS(DynamicJointModeSwitchPlugin, "back_right_mode_switch")
GZ_ADD_PLUGIN_ALIAS(DynamicJointModeSwitchPlugin, "back_left_mode_switch")
GZ_ADD_PLUGIN_ALIAS(DynamicJointModeSwitchPlugin, "front_left_mode_switch")