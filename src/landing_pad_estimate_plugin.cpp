#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <ignition/math.hh>

#include <thread>

using namespace gazebo;

const std::string landing_pad_global_position_topic_name = "/landing_pad/global_pose";

class LandingPadEstimatePlugin : public ModelPlugin
{
	public:
 		LandingPadEstimatePlugin() {}
		virtual ~LandingPadEstimatePlugin() {}

	protected:
		// Called when the plugin is loaded into the simulator
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			std::cerr << "LandingPadEstimatePlugin is attached to model [" << _model->GetName() << "]" << std::endl;

			this->model = _model;
			this->link = _model->GetLink("landing_pad_estimate_link");
			std::cerr << "Using link " << this->link->GetName() << std::endl;
			
			if( ! ros::isInitialized() )
			{
				int argc = 0;
				char **argv = NULL;

				ros::init(argc, argv, "gazebo_landing_pad_estimate_plugin_client");
			}

			this->ros_node.reset(new ros::NodeHandle("gazebo_landing_pad_estimate_plugin_client"));

			ros::SubscribeOptions so_landing_pad_global_position = ros::SubscribeOptions::create<geometry_msgs::Pose>(
					landing_pad_global_position_topic_name,
					1,
					boost::bind(&LandingPadEstimatePlugin::set_landing_pad_estimate_position, this, _1),
					ros::VoidPtr(),
					& this->ros_queue
					);
			this->landing_pad_global_position_subscriber = this->ros_node->subscribe(so_landing_pad_global_position);
			this->ros_queue_thread = std::thread(std::bind(&LandingPadEstimatePlugin::queue_thread, this));
		}

	private:
		physics::ModelPtr model;
		physics::LinkPtr link;
		rendering::VisualPtr visual;
		std::unique_ptr<ros::NodeHandle> ros_node;
		ros::CallbackQueue ros_queue;
		std::thread ros_queue_thread;
		ros::Subscriber landing_pad_global_position_subscriber;

		void set_landing_pad_estimate_position(const geometry_msgs::PoseConstPtr & _msg)
		{

			std::cerr << "inside landing_pad_estimate callback" << std::endl;

			ignition::math::Vector3<double>    position = ignition::math::Vector3<double>( _msg->position.x, _msg->position.y, _msg->position.z );
			//										(w, x, y, z) NOT (x, y, z, w)
//			ignition::math::Quaternion<double> rotation = ignition::math::Quaternion<double>(1, 0, 0, 0);
			ignition::math::Quaternion<double> rotation = ignition::math::Quaternion<double>(_msg->orientation.w, _msg->orientation.x, _msg->orientation.y, _msg->orientation.z);
			ignition::math::Pose3<double>          pose = ignition::math::Pose3<double>( position, rotation );

			try
			{
//				std::cerr << "( " << pose.Pos().X() << ", " << pose.Pos().Y() << ", " << pose.Pos().Z() << ")" << std::endl;
				this->model->SetWorldPose(pose);
			}
			catch( ... )
			{
				std::cerr << "Caught an exception." << std::endl;
			}
		}

		void queue_thread()
		{
			static const double timeout = 0.05;
			while(this->ros_node->ok())
			{
				this->ros_queue.callAvailable(ros::WallDuration(timeout));
			}
		}
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LandingPadEstimatePlugin)
