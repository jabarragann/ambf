#ifndef _ambf_ral_h
#define _ambf_ral_h

// ros includes
// #include <ambf_server/RosComBase.h>

#if ROS1

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <ambf_msgs/ObjectState.h>
#include <ambf_msgs/ObjectCmd.h>
#include <ambf_msgs/LightState.h>
#include <ambf_msgs/LightCmd.h>
#include <ambf_msgs/VehicleState.h>
#include <ambf_msgs/VehicleCmd.h>
#include <ambf_msgs/ActuatorState.h>
#include <ambf_msgs/ActuatorCmd.h>
#include <ambf_msgs/CameraState.h>
#include <ambf_msgs/CameraCmd.h>
#include <ambf_msgs/RigidBodyState.h>
#include <ambf_msgs/RigidBodyCmd.h>
#include <ambf_msgs/SensorState.h>
#include <ambf_msgs/SensorCmd.h>
#include <ambf_msgs/WorldState.h>
#include <ambf_msgs/WorldCmd.h>

#define AMBF_RAL_MSG(package, message) package::message

#elif ROS2

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <ambf_msgs/msg/object_state.hpp>
#include <ambf_msgs/msg/object_cmd.hpp>
#include <ambf_msgs/msg/light_state.hpp>
#include <ambf_msgs/msg/light_cmd.hpp>
#include <ambf_msgs/msg/vehicle_state.hpp>
#include <ambf_msgs/msg/vehicle_cmd.hpp>
#include <ambf_msgs/msg/actuator_state.hpp>
#include <ambf_msgs/msg/actuator_cmd.hpp>
#include <ambf_msgs/msg/camera_state.hpp>
#include <ambf_msgs/msg/camera_cmd.hpp>
#include <ambf_msgs/msg/rigid_body_state.hpp>
#include <ambf_msgs/msg/rigid_body_cmd.hpp>
#include <ambf_msgs/msg/sensor_state.hpp>
#include <ambf_msgs/msg/sensor_cmd.hpp>
#include <ambf_msgs/msg/world_state.hpp>
#include <ambf_msgs/msg/world_cmd.hpp>

#define AMBF_RAL_MSG(package, message) package::msg::message

#endif

 class ambf_ral
    {
    public:
        #if ROS1
        typedef std::shared_ptr<ros::NodeHandle> node_ptr_t;
        #elif ROS2
        typedef std::shared_ptr<rclcpp::Node> node_ptr_t;
        #endif
        ambf_ral(int & argc, char * argv[], const std::string & node_name, bool anonymous_name = true);
        ambf_ral(const std::string & node_name, bool anonymous_name = true);
        ~ambf_ral();

        inline node_ptr_t node(void) {
            return m_node;
        }

        typedef std::vector<std::string> stripped_arguments_t;
        inline const stripped_arguments_t & stripped_arguments(void) const {
            return m_stripped_arguments;
        }

    protected:
        void init(int & argc,  char * argv[], const std::string & node_name, bool anonymous_name);
        std::string m_node_name;
        node_ptr_t m_node;
        stripped_arguments_t m_stripped_arguments;
    };

#endif // _ambf_ral_h