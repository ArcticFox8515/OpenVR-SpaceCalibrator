#pragma once

#define EIGEN_MPL2_ONLY

#include "IPCServer.h"
#include "../Protocol.h"
#include "IsometryTransform.h"

#include <Eigen/Dense>

#include <openvr_driver.h>


class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() { }

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);
	void HandleApplyRandomOffset();
	void HandleSetAlignmentSpeedParams(const protocol::AlignmentSpeedParams params) {
		alignmentSpeedParams = params;
	}

private:
	IPCServer server;
	protocol::DriverPoseShmem shmem;

	enum DeltaSize {
		TINY,
		SMALL,
		LARGE
	};

	struct DeviceTransform
	{
		bool enabled = false;
		bool quash = false;
		IsoTransform transform, targetTransform;
		double scale;
		LARGE_INTEGER lastPoll;
		DeltaSize currentRate = DeltaSize::TINY;
	};

	DeviceTransform transforms[vr::k_unMaxTrackedDeviceCount];
	Eigen::Vector3d debugTransform;
	Eigen::Quaterniond debugRotation;

	DeltaSize currentDeltaSpeed[vr::k_unMaxTrackedDeviceCount];

	protocol::AlignmentSpeedParams alignmentSpeedParams;

	DeltaSize GetTransformDeltaSize(
		DeltaSize prior_delta,
		const IsoTransform& deviceWorldPose,
		const IsoTransform& src,
		const IsoTransform& target
	) const;

	double GetTransformRate(DeltaSize delta) const;

	void BlendTransform(DeviceTransform& device, const IsoTransform& deviceWorldPose) const;
	void ApplyTransform(DeviceTransform& device, vr::DriverPose_t& devicePose) const;
};