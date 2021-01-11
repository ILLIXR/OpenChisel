#include <functional>

#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

using namespace ILLIXR;

class open_chisel : public plugin {
public:
	open_chisel(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
	{
		_m_begin = std::chrono::system_clock::now();
	}


	virtual void start() override {
		plugin::start();
		// TODO: Change this plugin to use pose predict instead of slow pose
		sb->schedule<imu_cam_type>(id, "imu_cam", [&](const imu_cam_type *datum) {
			this->feed_imu_cam(datum);
		});
		sb->schedule<pose_type>(id, "slow_pose", [&](const pose_type *datum) {
			this->feed_pose(datum);
		});
	}

	void feed_pose(const pose_type *datum) {

	}

	void feed_imu_cam(const imu_cam_type *datum) {
		// Ensures that reconstruction doesnt start before valid IMU readings come in
		if (datum == NULL) {
			assert(previous_timestamp == 0);
			return;
		}

		// This ensures that every data point is coming in chronological order If youre failing this assert, 
		// make sure that your data folder matches the name in offline_imu_cam/plugin.cc
		double timestamp_in_seconds = (double(datum->dataset_time) / NANO_SEC);
		assert(timestamp_in_seconds > previous_timestamp);
		previous_timestamp = timestamp_in_seconds;

		// Ensure that reconstruction doesn't start until valid SLAM poses arrive
		if (datum == NULL) {
			return;
		}

		// TODO: Change this to get poses from pose predict
		// Convert depth_img to OpenChisel DepthImage
		// Pass converted depth and rgb image and SLAM pose to OpenChisel
		// May not want to use color

		// This may be unnecessary. Could use ChiselServer.h and modify it to work with ILLIXR.
		// Key Functions/Lines:
		// https://github.com/personalrobotics/OpenChisel/blob/30a1a7af58cdb773e8e24f86c255f35f1c163237/chisel_ros/src/ChiselServer.cpp#L420
		// https://github.com/personalrobotics/OpenChisel/blob/30a1a7af58cdb773e8e24f86c255f35f1c163237/chisel_ros/src/ChiselNode.cpp#L137
		// https://github.com/personalrobotics/OpenChisel/blob/30a1a7af58cdb773e8e24f86c255f35f1c163237/chisel_ros/include/chisel_ros/ChiselServer.h#L88

		// ROS nodeHandle is what we need to replace with ILLIXR stuff inside of ChiselServer

		// chisel::ChiselPtr chiselMap
		
		// chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator,  lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel, lastColorImage, colorCamera.lastPose, colorCamera.cameraModel);
		// chiselMap->UpdateMeshes();


	}

	virtual ~open_chisel() override {}

private:
	const std::shared_ptr<switchboard> sb;

	time_type _m_begin;

	double previous_timestamp = 0.0;
};

PLUGIN_MAIN(open_chisel)
