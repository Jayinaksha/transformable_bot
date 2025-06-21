#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sdf/Element.hh>
#include <chrono>

class MotorPlugin : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
  
  std::string arm_name_;
  std::string joint_name_;
  std::string velocity_topic_;
  
  gz::sim::Entity joint_entity_;
  bool joint_found_ = false;
  bool components_initialized_ = false;
  
  // Motor parameters
  double target_velocity_ = 0.0;
  double max_velocity_ = 1000.0; // Max RPM converted to rad/s (about 10,472 rad/s)
  
  static int instance_counter_;
  int my_instance_id_;
  int search_attempts_ = 0;
  const int MAX_SEARCH_ATTEMPTS = 100;
  
  // Timing
  std::chrono::steady_clock::time_point last_search_time_;
  const std::chrono::milliseconds SEARCH_INTERVAL{100};

public:
  MotorPlugin() {
    my_instance_id_ = ++instance_counter_;
    last_search_time_ = std::chrono::steady_clock::now();
  }

  void Configure(const gz::sim::Entity &/*entity*/,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &/*ecm*/,
                 gz::sim::EventManager &/*eventMgr*/) override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    // Read arm name from SDF
    if (sdf && sdf->HasElement("arm_name")) {
      arm_name_ = sdf->Get<std::string>("arm_name");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("motor_temp"), "✅ Read arm_name from SDF: '" << arm_name_ << "'");
    } else {
      arm_name_ = "arm_" + std::to_string(my_instance_id_);
      RCLCPP_WARN_STREAM(rclcpp::get_logger("motor_temp"), "⚠️ No arm_name in SDF, defaulting to: '" << arm_name_ << "'");
    }
    
    std::string node_name = "motor_" + arm_name_;
    node_ = rclcpp::Node::make_shared(node_name);
    
    // Construct joint and topic names
    joint_name_ = arm_name_ + "_motor_hub_to_motor";
    velocity_topic_ = "/robot/" + arm_name_ + "/motor/velocity";
    
    // Read SDF parameters with defaults
    if (sdf && sdf->HasElement("joint_name")) {
      joint_name_ = sdf->Get<std::string>("joint_name");
    }
    if (sdf && sdf->HasElement("velocity_topic")) {
      velocity_topic_ = sdf->Get<std::string>("velocity_topic");
    }
    if (sdf && sdf->HasElement("max_velocity")) {
      max_velocity_ = sdf->Get<double>("max_velocity");
    }
    
    // Create ROS subscriber
    velocity_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      velocity_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        // Apply velocity limits for safety
        double requested_velocity = msg->data;
        target_velocity_ = std::max(-max_velocity_, std::min(max_velocity_, requested_velocity));
        
        if (requested_velocity != target_velocity_) {
          RCLCPP_WARN(node_->get_logger(), 
                     "🛡️ %s Motor: Velocity limited from %.1f to %.1f rad/s", 
                     arm_name_.c_str(), requested_velocity, target_velocity_);
        }
        
        double rpm = target_velocity_ * 60.0 / (2.0 * M_PI);
        RCLCPP_INFO(node_->get_logger(), 
                   "🚁 %s Motor velocity: %.1f rad/s (%.0f RPM)", 
                   arm_name_.c_str(), target_velocity_, rpm);
      });
    
    RCLCPP_INFO(node_->get_logger(), 
                "🔧 Motor Plugin loaded for %s", arm_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), 
                "   Joint: %s", joint_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), 
                "   Velocity Topic: %s", velocity_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), 
                "   Max Velocity: %.1f rad/s (%.0f RPM)", 
                max_velocity_, max_velocity_ * 60.0 / (2.0 * M_PI));
  }

  void PreUpdate(const gz::sim::UpdateInfo &/*info*/,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    }
    
    // Search for joint if not found yet
    if (!joint_found_) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_search_time_ >= SEARCH_INTERVAL) {
        findJoint(ecm);
        last_search_time_ = now;
      }
      return; // Don't proceed until joint is found
    }
    
    // Initialize components if joint found but not initialized
    if (joint_found_ && !components_initialized_) {
      initializeJointComponents(ecm);
      return; // Wait one cycle for components to be ready
    }
    
    // Control the motor only if everything is ready
    if (joint_found_ && components_initialized_) {
      controlMotor(ecm);
    }
  }

private:
  void findJoint(gz::sim::EntityComponentManager &ecm)
  {
    search_attempts_++;
    
    if (search_attempts_ > MAX_SEARCH_ATTEMPTS) {
      RCLCPP_ERROR(node_->get_logger(), 
                   "❌ %s: Motor joint '%s' search FAILED after %d attempts", 
                   arm_name_.c_str(), joint_name_.c_str(), MAX_SEARCH_ATTEMPTS);
      printAllJointsForDebugging(ecm);
      return;
    }
    
    auto joints = ecm.EntitiesByComponents(gz::sim::components::Joint());
    
    for (auto joint : joints) {
      auto nameComp = ecm.Component<gz::sim::components::Name>(joint);
      if (nameComp && nameComp->Data() == joint_name_) {
        joint_entity_ = joint;
        joint_found_ = true;
        
        RCLCPP_INFO(node_->get_logger(), 
                   "🎯 %s: Found motor joint '%s'", 
                   arm_name_.c_str(), joint_name_.c_str());
        
        return;
      }
    }
    
    // Debug output every 20 attempts
    if (search_attempts_ % 20 == 1) {
      RCLCPP_WARN(node_->get_logger(), 
                  "⚠️ %s: Searching for motor joint '%s' (attempt %d/%d)", 
                  arm_name_.c_str(), joint_name_.c_str(), search_attempts_, MAX_SEARCH_ATTEMPTS);
    }
  }
  
  void initializeJointComponents(gz::sim::EntityComponentManager &ecm)
  {
    RCLCPP_INFO(node_->get_logger(), 
               "🔧 %s: Initializing joint components for motor...", 
               arm_name_.c_str());
    
    // Create JointVelocityCmd component if it doesn't exist
    auto velCmdComp = ecm.Component<gz::sim::components::JointVelocityCmd>(joint_entity_);
    if (!velCmdComp) {
      // Create with proper size for 1 DOF joint
      std::vector<double> initialVelocity = {0.0};
      ecm.CreateComponent(joint_entity_, gz::sim::components::JointVelocityCmd(initialVelocity));
      RCLCPP_INFO(node_->get_logger(), "   ✅ Created JointVelocityCmd component with 1 DOF");
    } else {
      // Check if existing component has wrong size and fix it
      if (velCmdComp->Data().size() != 1) {
        RCLCPP_WARN(node_->get_logger(), 
                   "   🔧 Fixing JointVelocityCmd size from %zu to 1", 
                   velCmdComp->Data().size());
        std::vector<double> correctedVelocity = {0.0};
        ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_entity_, correctedVelocity);
      }
    }
    
    components_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), 
               "✅ %s: Motor joint components initialized successfully", 
               arm_name_.c_str());
  }
  
  void printAllJointsForDebugging(gz::sim::EntityComponentManager &ecm)
  {
    RCLCPP_ERROR(node_->get_logger(), "=== 🔍 ALL JOINTS FOR %s MOTOR DEBUG ===", arm_name_.c_str());
    auto joints = ecm.EntitiesByComponents(gz::sim::components::Joint());
    
    std::vector<std::string> all_joints;
    for (auto joint : joints) {
      auto nameComp = ecm.Component<gz::sim::components::Name>(joint);
      if (nameComp) {
        all_joints.push_back(nameComp->Data());
      }
    }
    
    std::sort(all_joints.begin(), all_joints.end());
    
    RCLCPP_ERROR(node_->get_logger(), "📋 ALL JOINTS FOUND (%zu total):", all_joints.size());
    for (const auto& joint : all_joints) {
      if (joint.find(arm_name_) != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "  🎯 %s (ARM RELATED)", joint.c_str());
      } else if (joint.find("motor") != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "  🚁 %s (MOTOR)", joint.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "  - %s", joint.c_str());
      }
    }
    RCLCPP_ERROR(node_->get_logger(), "   Expected joint name: '%s'", joint_name_.c_str());
    RCLCPP_ERROR(node_->get_logger(), "=== 🔍 END JOINT DEBUG ===");
  }
  
  void controlMotor(gz::sim::EntityComponentManager &ecm)
  {
    // Get velocity command component
    auto velCmdComp = ecm.Component<gz::sim::components::JointVelocityCmd>(joint_entity_);
    
    if (!velCmdComp) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "⚠️ %s: Missing JointVelocityCmd component", arm_name_.c_str());
      return;
    }
    
    // Verify velocity command component has correct size
    if (velCmdComp->Data().size() != 1) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "⚠️ %s: JointVelocityCmd has wrong size (%zu), expected 1", 
                           arm_name_.c_str(), velCmdComp->Data().size());
      return;
    }
    
    // Set velocity command - MUCH SIMPLER than servo control!
    std::vector<double> velocity_cmd = {target_velocity_};
    ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint_entity_, velocity_cmd);
    
    // Debug output (every 100 cycles to avoid spam)
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0 && std::abs(target_velocity_) > 0.1) {
      double rpm = target_velocity_ * 60.0 / (2.0 * M_PI);
      RCLCPP_DEBUG(node_->get_logger(), 
                  "🚁 %s Motor: velocity=%.1f rad/s (%.0f RPM)", 
                  arm_name_.c_str(), target_velocity_, rpm);
    }
  }
};

int MotorPlugin::instance_counter_ = 0;

// Register the plugin
GZ_ADD_PLUGIN(MotorPlugin,
              gz::sim::System,
              MotorPlugin::ISystemConfigure,
              MotorPlugin::ISystemPreUpdate)

// Plugin aliases
GZ_ADD_PLUGIN_ALIAS(MotorPlugin, "motor_plugin")
GZ_ADD_PLUGIN_ALIAS(MotorPlugin, "front_right_motor")
GZ_ADD_PLUGIN_ALIAS(MotorPlugin, "front_left_motor")
GZ_ADD_PLUGIN_ALIAS(MotorPlugin, "back_right_motor") 
GZ_ADD_PLUGIN_ALIAS(MotorPlugin, "back_left_motor")