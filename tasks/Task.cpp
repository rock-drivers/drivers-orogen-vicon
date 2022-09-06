#include "Task.hpp"

#include <base-logging/Logging.hpp>

using namespace vicon;

int axesMap ( ViconDataStreamSDK::CPP::Direction::Enum axis )
{
	switch ( axis )
	{
	case ViconDataStreamSDK::CPP::Direction::Up: return 3;
	case ViconDataStreamSDK::CPP::Direction::Down: return -3;
	case ViconDataStreamSDK::CPP::Direction::Left: return 2;
	case ViconDataStreamSDK::CPP::Direction::Right: return -2;
	case ViconDataStreamSDK::CPP::Direction::Forward: return 1;
	case ViconDataStreamSDK::CPP::Direction::Backward: return -1;
	default: return 0;
	}
}

ViconDataStreamSDK::CPP::Direction::Enum axesMap ( int axis )
{
	int axis_val = axis < 0 ? -axis : axis;
	switch ( axis_val )
	{
	case 1:
		if ( axis > 0 ) return  ViconDataStreamSDK::CPP::Direction::Forward;
		else return ViconDataStreamSDK::CPP::Direction::Backward; 
	case 2:
		if ( axis > 0 ) return  ViconDataStreamSDK::CPP::Direction::Left;
		else return ViconDataStreamSDK::CPP::Direction::Right; 
	case 3:
		if ( axis > 0 ) return  ViconDataStreamSDK::CPP::Direction::Up;
		else return ViconDataStreamSDK::CPP::Direction::Down;
	default:
		return ViconDataStreamSDK::CPP::Direction::Forward;
	}
}

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

::base::samples::RigidBodyState Task::getZeroOrigin()
{
    base::samples::RigidBodyState lrbs;
    lrbs.initUnknown();
    return lrbs;
}

bool Task::configureHook()
{
    if (!_origin.value().hasValidPosition() || !_origin.value().hasValidOrientation() ) 
        _origin.set(getZeroOrigin());

    if (!_body_reference.value().hasValidPosition() || !_body_reference.value().hasValidOrientation() ) 
        _body_reference.set(getZeroOrigin());

    uncertainty.reset(new vicon::ViconUncertainty<Eigen::Matrix4d>(_uncertainty_samples.value()));

    return true;
}

bool Task::startHook()
{
	switch(dataStreamClient.Connect( _host.value() ).Result)
	{
	case ViconDataStreamSDK::CPP::Result::InvalidHostName:
		LOG_ERROR_S << "Host name '" << _host.value() << "' is invalid." << std::endl;
		break;
	case ViconDataStreamSDK::CPP::Result::ClientAlreadyConnected:
		LOG_ERROR_S << "Client is already connected." << std::endl;
		break;
	case ViconDataStreamSDK::CPP::Result::ClientConnectionFailed:
		LOG_ERROR_S << "Client connection failed." << std::endl;
		break;
	case ViconDataStreamSDK::CPP::Result::Success:
		LOG_INFO_S << "Successfully connected to '" << _host.value() << "'." << std::endl;
		break;
	default:
		LOG_ERROR_S << "Connect() returned unhandled code." << std::endl;
	}

	if( !dataStreamClient.IsConnected().Connected )
		return false;

	dataStreamClient.EnableSegmentData();
	dataStreamClient.EnableUnlabeledMarkerData();

	//dataStreamClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
	dataStreamClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
	// MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

	// Set the global up axis
	dataStreamClient.SetAxisMapping(
		axesMap(_xdir.value()),
		axesMap(_ydir.value()),
		axesMap(_zdir.value()));

	return true;
}

bool Task::getFrame( const base::Time& timeout )
{
	base::Time start = base::Time::now();
	ViconDataStreamSDK::CPP::Result::Enum result;
	while( ((result = dataStreamClient.GetFrame().Result) == ViconDataStreamSDK::CPP::Result::NoFrame) && (start+timeout > base::Time::now()) )
	{
		const unsigned long wait_ms = 10;
		usleep( wait_ms );
	}

	if( result == ViconDataStreamSDK::CPP::Result::Success )
	{
		LOG_DEBUG_S << "Got Frame!";
		return true;
	}
	else
	{
		if ( timeout.toSeconds() > 0 )
		{
			LOG_ERROR_S << "No Frame received!";
		}
		return false;
	}
}

void Task::updateHook()
{
	ViconDataStreamSDK::CPP::Result::Enum result = dataStreamClient.GetFrame().Result;
	if(result == ViconDataStreamSDK::CPP::Result::Success)
	{
		// origin is the origin2world transform for the neutral position/orientation
		Eigen::Affine3d C_world2origin( Eigen::Affine3d(_origin.value()).inverse() );
		Eigen::Affine3d C_segment2body( Eigen::Affine3d(_body_reference.value()) );

		base::samples::RigidBodyState rbs;
		rbs.time = base::Time::now();
		rbs.sourceFrame = _source_frame.get();
		rbs.targetFrame = _target_frame.get();

		// Get the segment name
		std::string SegmentName = dataStreamClient.GetSubjectRootSegmentName(_subject.value()).SegmentName;

		ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation trans = 
			dataStreamClient.GetSegmentGlobalTranslation( _subject.value(), _segment.value() );

		ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion quat = 
			dataStreamClient.GetSegmentGlobalRotationQuaternion( _subject.value(), _segment.value() );

		Eigen::Vector3d trans_m( trans.Translation[0], trans.Translation[1], trans.Translation[2] );

		// vicon data is in mm, but we prefer standard si units...
		Eigen::Affine3d segment_transform = Eigen::Translation3d( trans_m * 1e-3 ) * 
		Eigen::Quaterniond( quat.Rotation[3], quat.Rotation[0], quat.Rotation[1], quat.Rotation[2] );

		bool inFrame = !trans.Occluded;

		std::vector<base::Vector3d> markers;
		unsigned int numMarkers = dataStreamClient.GetUnlabeledMarkerCount().MarkerCount;
		for( unsigned int idx = 0 ; idx < numMarkers ; ++idx )
		{
			ViconDataStreamSDK::CPP::Output_GetUnlabeledMarkerGlobalTranslation globalTranslation =
			dataStreamClient.GetUnlabeledMarkerGlobalTranslation( idx );

			Eigen::Vector3d marker_pos( 
				globalTranslation.Translation[ 0 ],
				globalTranslation.Translation[ 1 ],
				globalTranslation.Translation[ 2 ] );

			markers.push_back( marker_pos * 1e-3 ); 
		}
		_unlabeled_markers.write( markers );

		if(trans.Result == ViconDataStreamSDK::CPP::Result::InvalidSubjectName)
		{
			LOG_ERROR_S << "subject " << _subject.value() << " not found!" << RTT::endlog();
			return;
		}else
		if(trans.Result == ViconDataStreamSDK::CPP::Result::InvalidSegmentName)
		{
			LOG_ERROR_S << "segment " << _segment.value() << " not found!" << RTT::endlog();
			return;
		}

		if (inFrame || !_invalidate_occluded.get())
		{
			/** Fill the Rbs transformation **/
			rbs.setTransform( C_world2origin * segment_transform * C_segment2body );

			/** Set uncertainty in the rbs **/
			if (_uncertainty_samples.value() > 0)
			{
				/** Push sample to the uncertainty **/
				uncertainty->push(rbs.getTransform().matrix());

				/** On line uncertainty **/
				Eigen::Matrix4d transform_uncertainty = uncertainty->getVariance();

				rbs.cov_position = transform_uncertainty.block<3,1>(0,3).asDiagonal();
				rbs.cov_orientation = transform_uncertainty.block<3,3>(0,0);
			}
		}else
		{
			rbs.invalidate();
		}

		if (inFrame || !_drop_occluded.get())
		{
			_pose_samples.write( rbs );
		}
	}
}

void Task::stopHook()
{
	LOG_INFO_S << "disconnecting";
	dataStreamClient.Disconnect();
}
