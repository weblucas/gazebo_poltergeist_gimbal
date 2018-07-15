#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/Joy.h>

namespace gazebo
{

//// Default values
static const std::string kDefaultParentFrameId = "gimbal_support";
static const std::string kDefaultChildFrameId = "camera_mount";
static const std::string kDefaulGimbaltLinkName = "firefly/gimbal_support_link";
static const std::string kDefaultCameraLinkName = "firefly/gimbal_yaw";
static const  uint kNumActuators = 3;
static const  float kDEG_2_RAD = M_PI / 180.0;

class GazeboPoltergeistGimbalPlugin : public ModelPlugin
{
public:    
    GazeboPoltergeistGimbalPlugin():ModelPlugin(){
        //        <robotNamespace>${namespace}</robotNamespace>
        //
        //                    <gimbal_angular_velocity>0.3</gimbal_angular_velocity>
        //                    <ref_base_link>${parent_link}</ref_base_link>
        //                    <camera_link>camera_mount_link</camera_link>
    }
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_gimbal_plugin",
                      ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        node_handle_.reset(new ros::NodeHandle("gazebo_gimbal_plugin"));

        // Store the pointer to the model
        model_ = _parent;

        std::string ref_base_link_str = _sdf->Get<std::string>("ref_base_link");
        gimbal_support_link_ = model_->GetChildLink(ref_base_link_str);

        std::string cam_link_str = _sdf->Get<std::string>("camera_link");
        camera_mount_link_ = model_->GetChildLink(cam_link_str);

        //rpy order roll pitch yaw

        joints_.push_back(model_->GetJoint(_sdf->Get<std::string>("roll_joint_name")));
        base_points_.push_back(_sdf->Get<double>("roll_zero"));
        directions_.push_back(_sdf->Get<double>("roll_direction"));

        joints_.push_back(model_->GetJoint(_sdf->Get<std::string>("pitch_joint_name")));
        base_points_.push_back(_sdf->Get<double>("pitch_zero"));
        directions_.push_back(_sdf->Get<double>("pitch_direction"));

        joints_.push_back(model_->GetJoint(_sdf->Get<std::string>("yaw_joint_name")));
        base_points_.push_back(_sdf->Get<double>("yaw_zero"));
        directions_.push_back(_sdf->Get<double>("yaw_direction"));

        goal_points_.assign(joints_.size(),0);
        actuator_status_.assign(joints_.size(),STATIC);

        velocity_ = 0.3;
        if (_sdf->HasElement("gimbal_angular_velocity"))
            velocity_ = _sdf->Get<double>("gimbal_angular_velocity");


        input_sub_ = node_handle_->subscribe<sensor_msgs::Joy>(
                    "/command/gimbal_actuators", 10, &GazeboPoltergeistGimbalPlugin::onInputCallback ,this);


        static bool first_time = true;
        if(first_time)
        {
            for(int i=0;i<kNumActuators;i++)
            {
                joints_[i]->SetPosition(0,base_points_[i]*kDEG_2_RAD);
                actuator_status_[i] = STATIC;
            }

            first_time = false;
        }

        ROS_WARN("end Hello World! gazebo_poltergeist_gimbal model v9: ");

        //       cam_joint_ = this->model_->GetJoint("my_joint");


        //      base_point_ = gazebo::math::Vector3(cam_joint_->GetAngle(0).Degree(),cam_joint_->GetAngle(1).Degree(),cam_joint_->GetAngle(2).Degree());


//         Listen to the update event. This event is broadcast every
//         simulation iteration.
            this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                        std::bind(&GazeboPoltergeistGimbalPlugin::OnUpdate, this));

        ROS_WARN("end Hello World! gazebo_poltergeist_gimbal model v9");
    }

    void onInputCallback(const sensor_msgs::Joy::ConstPtr& msg) {
        if(msg->axes.size() != 3 )
        {
            ROS_ERROR_STREAM( "Gimbal Request need to have 3 axes, Roll: Pitch: Yaw, even that they are not used");

        }
        ROS_INFO_STREAM( "Gimbal Request Roll:"<< msg->axes[0] << " Pitch:"<< msg->axes[1] << " Yaw:"<< msg->axes[2] << " ");
        bool input_valid = true;
        if(msg->axes[0] > 180 || msg->axes[0] < -179)
        {
            ROS_WARN_STREAM("Gimbal Request ignored - invalid ROLL [-179,180]: " << msg->axes[0]);
            input_valid = false;
        }

        if(msg->axes[1] > 180 || msg->axes[1] < -179)
        {
            ROS_WARN_STREAM("Gimbal Request ignored - invalid PITCH [-179,180]: " << msg->axes[1]);
            input_valid = false;
        }

        if(msg->axes[2] > 180 || msg->axes[2] < -179)
        {
            ROS_WARN_STREAM("Gimbal Request ignored - invalid YAW [-179,180]: " << msg->axes[2]);
            input_valid = false;
        }

        if(input_valid)
        {
            for(int i=0;i<kNumActuators;i++)
            {
                goal_points_[i] = msg->axes[i];
                actuator_status_[i] = MOVING;
            }
        }
    }

    // Called by the world update start event
    void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));


        for(int i=0;i<kNumActuators;i++){
            if(actuator_status_[i] == STATIC)
            {

                setSpeed(i,0.0);

            }
            else if(actuator_status_[i] == MOVING)
            {

                double curr_position = GetJointPositionOffSeted(i); //todo check function normalize from the angle class

                double angle_diff =  curr_position - goal_points_[i];

                if(std::abs(angle_diff) < 1.0)
                {
                    actuator_status_[i] = STATIC;
                    setSpeed(i,0.0);
                    //todo correct the position to the ideal value
                }else
                {
                    if(angle_diff > 0)
                        setSpeed(i,-velocity_);
                    else
                        setSpeed(i,velocity_);
                }
                ROS_INFO_STREAM("axis: "<< i <<" curr " << joints_[i]->GetAngle(0).Degree() << " diff:" << angle_diff << " curr:" << curr_position << " goal:" << goal_points_[i]  ); //camera_mount_link_->GetRelativePose() <<

            }
        }
    }

    double setSpeed(size_t joint_index,double vel)
    {
#if GAZEBO_MAJOR_VERSION > 4
        joints_[joint_index]->SetVelocity(0,vel);
#else
        joints_[joint_index]->SetAttribute("fmax", 0, 100.0);
        joints_[joint_index]->SetAttribute("vel", 0, vel);
#endif
    }

    double GetJointPosition(size_t joint_index)
    {
        return ConvertAngle180(std::fmod(joints_[joint_index]->GetAngle(0).Degree()+72000000,360.0));
    }

    double GetJointPositionOffSeted(size_t joint_index)
    {
        return ConvertAngle180(std::fmod(GetJointPosition(joint_index) -  base_points_[joint_index]+360,360.0));
    }

    double ConvertAngle180(double angle) //[-180,180]
    {
        return std::fmod(angle+180.0,360.0)-180.0;// 0->0 , 10->10, 350->-10
    }

    double ConvertAngle360(double angle)//[0,360]
    {
        return std::fmod(angle+360.0,360.0);// 0 -> 0 , 10 -> 10, -10 -> 350
    }



    double AngleClamp( float angle )
    {
        if (angle > 180)
        {
            angle = 180;
        }
        if (angle < -180)
        {
            angle = -180;
        }
        return angle;
    }

    // Pointer to the model
private:



    ///STATE

    enum ActuatorStatus
    {
        STATIC,
        MOVING
    };

    double velocity_;


    ///GAZEBO
    ///

    physics::ModelPtr model_;

    std::unique_ptr<ros::NodeHandle> node_handle_;

    physics::LinkPtr gimbal_support_link_;
    physics::LinkPtr camera_mount_link_;


    std::vector<double> base_points_;
    std::vector<double> goal_points_;
    std::vector<double> directions_;
    std::vector<physics::JointPtr> joints_;
    std::vector<ActuatorStatus> actuator_status_;
    double rotational_velocity_;

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;

    ///ROS

    /// \brief A node use for ROS transport
private:

    std::unique_ptr<ros::NodeHandle> ros_node_;

    /// \brief A ROS subscriber
    ros::Subscriber input_sub_;

    ros::Publisher status_pub_;
    ros::Publisher transform_gimbal_camera_pub_;
    ros::Publisher gimbal_camera_tf_pub_;





};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboPoltergeistGimbalPlugin)
}


