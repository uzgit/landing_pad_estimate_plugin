#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <geometry_msgs/Vector3.h>
#include <ignition/math.hh>

#include <thread>

using namespace gazebo;

class InvisibilityPlugin : public VisualPlugin
{
	public:
		InvisibilityPlugin() {}
		virtual ~InvisibilityPlugin() {}

	protected:
		void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
		{
			std::cerr << "Setting visual element [" << _visual->YourMom() << "] as invisible to every camera except GUI camera!" << std::endl;
			_visual->SetVisibilityFlags( GZ_VISIBILITY_GUI );
		}
};

GZ_REGISTER_VISUAL_PLUGIN(InvisibilityPlugin)
