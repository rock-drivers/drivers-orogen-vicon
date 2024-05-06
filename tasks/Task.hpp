#ifndef VICON_TASK_TASK_HPP
#define VICON_TASK_TASK_HPP

#include <vicon/TaskBase.hpp>
#include <boost/shared_ptr.hpp>

#include <DataStreamClient.h>
#include <ViconUncertainty.hpp>

namespace vicon
{
	class Task : public TaskBase
	{
	friend class TaskBase;

	protected:
		boost::shared_ptr<vicon::ViconUncertainty <Eigen::Matrix4d> > uncertainty;
		ViconDataStreamSDK::CPP::Client dataStreamClient;

		Eigen::Affine3d start_pose_inverse;
		bool start_pose_initialized = false;

	public:
		Task(std::string const& name = "vicon::Task");

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
	};
}

#endif

