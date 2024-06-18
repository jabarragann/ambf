#ifndef _ambf_ral_h
#define _ambf_ral_h

// this file is based on cisst-ros/cisst_ros_bridge cisst_ral.h. we
// should probably try to merge this and distribute in a single
// package (Anton)

#if ROS1

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>

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

#define AMBF_RAL_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define AMBF_RAL_INFO(...)  ROS_INFO(__VA_ARGS__)
#define AMBF_RAL_WARN(...)  ROS_WARN(__VA_ARGS__)
#define AMBF_RAL_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define AMBF_RAL_FATAL(...) ROS_FATAL(__VA_ARGS__)

#define AMBF_RAL_MSG(package, message) package::message
#define AMBF_RAL_MSG_PTR(package, message) package::message##Ptr
// #define AMBF_RAL_MSG_CONST_PTR(package, message) package::message##ConstPtr

namespace ambf_ral {
    typedef std::shared_ptr<ros::NodeHandle> node_ptr_t;
    typedef ros::Rate rate_t;
    typedef std::shared_ptr<rate_t> rate_ptr_t;
    typedef ros::Time time_t;
    typedef ros::Duration duration_t;

    inline std::string node_namespace(node_ptr_t node) {
        return node->getNamespace();
    }

    inline bool time_is_zero(const ros::Time & time) {
        return time.isZero();
    }

    inline double age_in_seconds(const ros::Time & time,
                                 node_ptr_t) {
        return (ros::Time::now() - time).toSec();
    }

    inline ros::Time now(node_ptr_t) {
        return ros::Time::now();
    }

    inline ros::Duration duration_from_seconds(const double & duration) {
        return ros::Duration(duration);
    }

    inline void spin(node_ptr_t) {
        ros::spin();
    }

    inline void shutdown(void) {
        ros::shutdown();
    }

    template <typename _ros_t>
    void create_publisher(std::shared_ptr<ros::Publisher> & publisher,
                          node_ptr_t node,
                          const std::string & topic,
                          const size_t queue_size,
                          const bool latched) {
        publisher = std::make_shared<ros::Publisher>(node->advertise<_ros_t>(topic, queue_size, latched));
        if (!publisher) {
            std::cerr << "Failed to create publisher for " << topic << std::endl;
        }
    }

    template <typename _ros_t, typename _object_cb_t>
    void create_subscriber(std::shared_ptr<ros::Subscriber> & subscriber,
                           node_ptr_t node,
                           const std::string & topic,
                           const size_t queue_size,
                           void (_object_cb_t::*cb)(const _ros_t &),
                           _object_cb_t * instance
                           ) {
        subscriber = std::make_shared<ros::Subscriber>(node->subscribe(topic, queue_size, cb, instance));
        if (!subscriber) {
            std::cerr << "Failed to create subscriber for " << topic << std::endl;
        }
    }

    template <typename _pub_t>
    inline size_t nb_subscribers(_pub_t publisher) {
        return publisher->getNumSubscribers();
    }

    template <typename _pub_t>
    inline void publisher_shutdown(_pub_t publisher) {
        publisher->shutdown();
    }

    template <typename _pub_t>
    inline void subscriber_shutdown(_pub_t subscriber) {
        subscriber->shutdown();
    }
}

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

#define AMBF_RAL_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_WARN(...)  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), __VA_ARGS__)

#define AMBF_RAL_MSG(package, message) package::msg::message
#define AMBF_RAL_MSG_PTR(package, message) package::msg::message::Ptr
// #define AMBF_RAL_MSG_CONST_PTR(package, message) package::msg::message::ConstPtr

namespace ambf_ral {
    typedef std::shared_ptr<rclcpp::Node> node_ptr_t;
    typedef rclcpp::Rate rate_t;
    typedef std::shared_ptr<rate_t> rate_ptr_t;
    typedef rclcpp::Time time_t;
    typedef rclcpp::Duration duration_t;

    inline std::string node_namespace(node_ptr_t node) {
        return node->get_namespace();
    }

    inline bool time_is_zero(const rclcpp::Time & time) {
        return ((time.seconds() == 0)
                && (time.nanoseconds() == 0));
    }

    inline double age_in_seconds(const rclcpp::Time & time,
                                 node_ptr_t node) {
        return (node->get_clock()->now() - time).seconds();
    }

    inline rclcpp::Time now(node_ptr_t node) {
        return node->get_clock()->now();
    }

    inline rclcpp::Duration duration_from_seconds(const double & duration) {
        return rclcpp::Duration::from_seconds(duration);
    }

    inline void spin(node_ptr_t node) {
        rclcpp::spin(node);
    }

    inline void shutdown(void) {
        rclcpp::shutdown();
    }

    template <typename _ros_t>
    void create_publisher(typename rclcpp::Publisher<_ros_t>::SharedPtr & publisher,
                          node_ptr_t node,
                          const std::string & topic,
                          const size_t queue_size,
                          const bool latched) {
        rclcpp::QoS qos(queue_size);
        if (latched) {
            qos.transient_local();
        }
        publisher = node->create_publisher<_ros_t>(topic, qos);
        if (!publisher) {
          std::cerr << "Failed to create publisher for " << topic << std::endl;
        }
    }

    template <typename _ros_t, typename _object_cb_t>
    void create_subscriber(typename rclcpp::Subscription<_ros_t>::SharedPtr & subscriber,
                           node_ptr_t node,
                           const std::string & topic,
                           const size_t queue_size,
                           void (_object_cb_t::*cb)(const _ros_t &),
                           _object_cb_t * instance
                           ) {
        subscriber = node->create_subscription<_ros_t>(topic,
                                                       queue_size,
                                                       std::bind(cb,
                                                                 instance,
                                                                 std::placeholders::_1));
        if (!subscriber) {
            std::cerr << "Failed to create subscriber for " << topic << std::endl;
        }
    }

    template <typename _pub_t>
    inline size_t nb_subscribers(_pub_t publisher) {
        return publisher->get_subscription_count();
    }

    template <typename _pub_t>
    inline void publisher_shutdown(_pub_t) {
        // publisher->shutdown();
    }

    template <typename _pub_t>
    inline void subscriber_shutdown(_pub_t) {
        // subscriber->shutdown();
    }
}

#endif // ROS2


namespace ambf_ral {

    inline void clean_namespace(std::string & _ros_namespace) {
#if ROS1
        _ros_namespace = ros::names::clean(_ros_namespace);
#endif
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ' ', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '-', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '.', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '(', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ')', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '[', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ']', '_');
    }

    class ral
    {
    public:
        ral(int & argc, char * argv[], const std::string & node_name, bool anonymous_name = true);
        ral(const std::string & node_name, bool anonymous_name = true);
        ~ral();

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
}

#endif // _ambf_ral_h
