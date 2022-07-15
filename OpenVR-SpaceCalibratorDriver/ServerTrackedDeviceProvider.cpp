#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof DeviceTransform);

	InjectHooks(this, pDriverContext);
	server.Run();
	shmem.Create(OPENVR_SPACECALIBRATOR_SHMEM_NAME);

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	shmem.Close();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

namespace {
	vr::HmdQuaternion_t convert(const Eigen::Quaterniond& q) {
		vr::HmdQuaternion_t result;
		result.w = q.w();
		result.x = q.x();
		result.y = q.y();
		result.z = q.z();
		return result;
	}

	vr::HmdVector3_t convert(const Eigen::Vector3d& v) {
		vr::HmdVector3_t result;
		result.v[0] = v.x();
		result.v[1] = v.y();
		result.v[2] = v.z();
		return result;
	}

	Eigen::Quaterniond convert(const vr::HmdQuaternion_t& q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
	}

	Eigen::Vector3d convert(const vr::HmdVector3d_t& v) {
		return Eigen::Vector3d(v.v[0], v.v[1], v.v[2]);
	}

	Eigen::Vector3d convert(const double* arr) {
		return Eigen::Vector3d(arr[0], arr[1], arr[2]);
	}
}

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
	vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
	vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	auto &tf = transforms[newTransform.openVRID];
	tf.enabled = newTransform.enabled;

	if (newTransform.updateTranslation) {
		tf.targetTranslation = convert(newTransform.translation);
		if (!newTransform.lerp) {
			tf.translation = tf.targetTranslation;
		}
	}

	if (newTransform.updateRotation) {
		tf.targetRotation = convert(newTransform.rotation);

		if (!newTransform.lerp) {
			tf.rotation = tf.targetRotation;
		}
	}

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;

	tf.quash = newTransform.quash;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose)
{
	shmem.SetPose(openVRID, pose);

	auto& tf = transforms[openVRID];

	if (tf.quash) {
		pose.poseIsValid = false;
		pose.result = vr::TrackingResult_Running_OutOfRange;
		pose.vecPosition[0] = -pose.vecWorldFromDriverTranslation[0];
		pose.vecPosition[1] = -pose.vecWorldFromDriverTranslation[1];
		pose.vecPosition[2] = -pose.vecWorldFromDriverTranslation[2];
	} else if (tf.enabled)
	{
		LARGE_INTEGER timestamp, freq;
		QueryPerformanceCounter(&timestamp);
		QueryPerformanceFrequency(&freq);

		double lerp = (timestamp.QuadPart - tf.lastPoll.QuadPart) / (double)freq.QuadPart;
		lerp *= 2;
		if (lerp > 1.0)
			lerp = 1.0;
		if (lerp < 0 || isnan(lerp))
			lerp = 0;
		// Cancel out any translation induced by this rotation slerp. This helps avoid your
		// controllers/trackers flying away when far from the origin and a rotation correction
		// is performed.
		auto priorPos = tf.rotation * convert(pose.vecPosition);
		tf.rotation = tf.rotation.slerp(lerp, tf.targetRotation);
		auto newPos = tf.rotation * convert(pose.vecPosition);
		tf.translation -= newPos - priorPos;

		tf.translation = tf.translation * (1 - lerp) + tf.targetTranslation * lerp;
		tf.lastPoll = timestamp;
		
		pose.qWorldFromDriverRotation = convert(tf.rotation * convert(pose.qWorldFromDriverRotation));

		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		auto newTranslation = tf.translation + tf.rotation * convert(pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = newTranslation.x();
		pose.vecWorldFromDriverTranslation[1] = newTranslation.y();
		pose.vecWorldFromDriverTranslation[2] = newTranslation.z();
	}

	return true;
}

