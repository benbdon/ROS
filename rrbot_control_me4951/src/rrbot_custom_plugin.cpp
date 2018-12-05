#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <thread>
#include <memory>

#include "rrbot_control_me495/RRBotConfig.h"

namespace gazebo
{
	class RRBotPosControl : public ModelPlugin
	{

	private:
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
		// Pointer to the model:
		physics::ModelPtr model;
		// ROS node handle:
		std::unique_ptr<ros::NodeHandle> n_;
		// Subscriber:
		ros::Subscriber sub;
		// dict with names and desired angles:
		std::map<std::string, double> refAngles;
		// Queue for processing messages
		ros::CallbackQueue subQueue;
		// thread for queue
		std::thread subQueueThread;


	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
			{
				// Store the pointer to the model
				this->model = _parent;

				// Connect callback to simulation update event
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&RRBotPosControl::OnUpdate, this, _1));

				if (!ros::isInitialized())
				{
					int argc = 0;
					char **argv = NULL;
					ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
				}

				// Create our ROS node. This acts in a similar manner to the Gazebo node
				this->n_.reset(new ros::NodeHandle("rrbot_joint_position_control"));
				// subscriber:
				ros::SubscribeOptions so =
					ros::SubscribeOptions::create<rrbot_control_me495::RRBotConfig>(
						"rrbot_ref_joint_config",
						1,
						boost::bind(&RRBotPosControl::jointConfigCB, this, _1),
						ros::VoidPtr(), &this->subQueue);

				sub = this->n_->subscribe(so);

				// fill out dict of joint names:
				refAngles["joint1"] = this->model->GetJoint("joint1")->GetAngle(0).Radian();
				refAngles["joint2"] = this->model->GetJoint("joint2")->GetAngle(0).Radian();

				// now create a thread for running spin operation:
				this->subQueueThread = std::thread(std::bind(&RRBotPosControl::QueueThread, this));

				return;
			}


	public:
		// callback for received message:
		void jointConfigCB(const rrbot_control_me495::RRBotConfigConstPtr &msg)
			{
				this->refAngles["joint1"] = msg->j1;
				this->refAngles["joint2"] = msg->j2;
				return;
			}


		// Called by the world update start event
		void OnUpdate(const common::UpdateInfo & /*_info*/)
			{
				this->model->GetJoint("joint1")->SetPosition(0, refAngles["joint1"]);
				this->model->GetJoint("joint2")->SetPosition(0, refAngles["joint2"]);
				this->model->GetJoint("joint1")->SetVelocity(0, 0);
				this->model->GetJoint("joint2")->SetVelocity(0, 0);
				this->model->GetJoint("joint1")->SetForce(0, 0);
				this->model->GetJoint("joint2")->SetForce(0, 0);
				return;
			}

	private:
		// Our own simple version of a spin function:
		void QueueThread()
			{
				static const double timeout = 0.01;
				while (this->n_->ok())
				{
					this->subQueue.callAvailable(ros::WallDuration(timeout));
				}
			}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(RRBotPosControl)
}
