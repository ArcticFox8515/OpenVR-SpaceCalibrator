#pragma once

#include <windows.h>
#include <cstdint>
#include <atomic>
#include <stdexcept>
#include <functional>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define OPENVR_SPACECALIBRATOR_PIPE_NAME "\\\\.\\pipe\\OpenVRSpaceCalibratorDriver"
#define OPENVR_SPACECALIBRATOR_SHMEM_NAME "OpenVRSpaceCalibratorPoseMemoryV1"

#ifdef _OPENVR_API 

namespace vr {
	// We can't include openvr_driver.h as it will result in multiple definition of some structures.
	// However, we need to share driver-specific structures with the client application, so duplicate them
	// here.

	struct DriverPose_t
	{
		/* Time offset of this pose, in seconds from the actual time of the pose,
		 * relative to the time of the PoseUpdated() call made by the driver.
		 */
		double poseTimeOffset;

		/* Generally, the pose maintained by a driver
		 * is in an inertial coordinate system different
		 * from the world system of x+ right, y+ up, z+ back.
		 * Also, the driver is not usually tracking the "head" position,
		 * but instead an internal IMU or another reference point in the HMD.
		 * The following two transforms transform positions and orientations
		 * to app world space from driver world space,
		 * and to HMD head space from driver local body space.
		 *
		 * We maintain the driver pose state in its internal coordinate system,
		 * so we can do the pose prediction math without having to
		 * use angular acceleration.  A driver's angular acceleration is generally not measured,
		 * and is instead calculated from successive samples of angular velocity.
		 * This leads to a noisy angular acceleration values, which are also
		 * lagged due to the filtering required to reduce noise to an acceptable level.
		 */
		vr::HmdQuaternion_t qWorldFromDriverRotation;
		double vecWorldFromDriverTranslation[3];

		vr::HmdQuaternion_t qDriverFromHeadRotation;
		double vecDriverFromHeadTranslation[3];

		/* State of driver pose, in meters and radians. */
		/* Position of the driver tracking reference in driver world space
		* +[0] (x) is right
		* +[1] (y) is up
		* -[2] (z) is forward
		*/
		double vecPosition[3];

		/* Velocity of the pose in meters/second */
		double vecVelocity[3];

		/* Acceleration of the pose in meters/second */
		double vecAcceleration[3];

		/* Orientation of the tracker, represented as a quaternion */
		vr::HmdQuaternion_t qRotation;

		/* Angular velocity of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second. */
		double vecAngularVelocity[3];

		/* Angular acceleration of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second^2. */
		double vecAngularAcceleration[3];

		ETrackingResult result;

		bool poseIsValid;
		bool willDriftInYaw;
		bool shouldApplyHeadModel;
		bool deviceIsConnected;
	};

}

#endif

namespace protocol
{
	const uint32_t Version = 4;

	enum RequestType
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetAlignmentSpeedParams,
		RequestDebugOffset
	};

	enum ResponseType
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	struct AlignmentSpeedParams
	{
		/**
		 * The threshold at which we adjust the alignment speed based on the position offset
		 * between current and target calibrations. Generally, we increase the speed if we go
		 * above small/large, and decrease it only once it's under tiny.
		 * 
		 * These values are expressed as distance squared
		 */
		double thr_trans_tiny, thr_trans_small, thr_trans_large;

		/**
		 * Similar thresholds for rotation offsets, in radians
		 */
		double thr_rot_tiny, thr_rot_small, thr_rot_large;
		
		/**
		 * The speed of alignment, expressed as a lerp/slerp factor. 1 will blend most of the way in <1 second.
		 * (We actually do a lerp(s * delta_t) where s is the speed factor here)
		 */
		double align_speed_tiny, align_speed_small, align_speed_large;
	};

	struct SetDeviceTransform
	{
		uint32_t openVRID;
		bool enabled;
		bool updateTranslation;
		bool updateRotation;
		bool updateScale;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
		bool lerp;
		bool quash;

		SetDeviceTransform(uint32_t id, bool enabled) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false), translation(translation), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false), rotation(rotation), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true), scale(scale), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false), translation(translation), rotation(rotation), lerp(false), quash(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true), translation(translation), rotation(rotation), scale(scale), lerp(false), quash(false) { }
	};

	struct Request
	{
		RequestType type;

		union {
			SetDeviceTransform setDeviceTransform;
			AlignmentSpeedParams setAlignmentSpeedParams;
		};

		Request() : type(RequestInvalid) { }
		Request(RequestType type) : type(type) { }
		Request(AlignmentSpeedParams params) : type(RequestType::RequestSetAlignmentSpeedParams), setAlignmentSpeedParams(params) {}
	};

	struct Response
	{
		ResponseType type;

		union {
			Protocol protocol;
		};

		Response() : type(ResponseInvalid) { }
		Response(ResponseType type) : type(type) { }
	};

	class DriverPoseShmem {
	public:
		struct AugmentedPose {
			LARGE_INTEGER sample_time;
			int deviceId;
			vr::DriverPose_t pose;
		};
	private:
		static const uint32_t SYNC_ACTIVE_POSE_B = 0x80000000;
		static const uint32_t BUFFERED_SAMPLES = 64 * 1024;

		struct ShmemData {
			std::atomic<uint64_t> index;
			AugmentedPose poses[BUFFERED_SAMPLES];
		};
		
	private:
		HANDLE hMapFile;
		ShmemData* pData;
		uint64_t cursor;

		AugmentedPose lastPose[vr::k_unMaxTrackedDeviceCount];

		std::string LastErrorString(DWORD lastError)
		{
			LPSTR buffer = nullptr;
			size_t size = FormatMessageA(
				FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
			);

			std::string message(buffer, size);
			LocalFree(buffer);
			return message;
		}

	public:
		operator bool() const {
			return pData != nullptr;
		}

		bool operator!() const {
			return pData == nullptr;
		}

		DriverPoseShmem() {
			hMapFile = INVALID_HANDLE_VALUE;
			pData = nullptr;
			cursor = 0;
		}

		~DriverPoseShmem() {
			Close();
		}

		void Close() {
			if (pData) UnmapViewOfFile(pData);
			if (hMapFile) CloseHandle(hMapFile);
		}

		bool Create(LPCSTR segment_name) {
			Close();

			hMapFile = CreateFileMappingA(
				INVALID_HANDLE_VALUE,
				NULL,
				PAGE_READWRITE,
				0,
				sizeof(ShmemData),
				segment_name
			);

			if (!hMapFile) return false;

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile,
				FILE_MAP_ALL_ACCESS,
				0,
				0,
				sizeof(ShmemData)
			));

			return !!pData;
		}


		void Open(LPCSTR segment_name) {
			Close();

			hMapFile = OpenFileMappingA(
				FILE_MAP_ALL_ACCESS,
				FALSE,
				segment_name
			);

			if (!hMapFile) {
				throw std::runtime_error("Failed to open pose data shared memory segment: " + LastErrorString(GetLastError()));
			}

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile,
				FILE_MAP_ALL_ACCESS,
				0,
				0,
				sizeof(ShmemData)
			));

			if (!pData) {
				throw std::runtime_error("Failed to map pose data shared memory segment: " + LastErrorString(GetLastError()));
			}

			char tmp[256];
			snprintf(tmp, sizeof tmp, "Opened shmem segment: %p\n", pData);
			OutputDebugStringA(tmp);
		}

		void ReadNewPoses(std::function<void(AugmentedPose const&)> cb) {
			if (!pData) throw std::runtime_error("Not open");
			
			uint64_t cur_index = pData->index.load(std::memory_order_acquire);
			if (cur_index < cursor || cur_index - cursor > BUFFERED_SAMPLES / 2) {
				if (cur_index < BUFFERED_SAMPLES / 2)
					cursor = cur_index;
				else
					cursor = cur_index - BUFFERED_SAMPLES / 2;
			}

			while (cursor < cur_index) {
				cb(pData->poses[cursor % BUFFERED_SAMPLES]);
				cursor++;
			}

			std::atomic_thread_fence(std::memory_order_release);
		}

		bool GetPose(int index, vr::DriverPose_t& pose, LARGE_INTEGER *pSampleTime = NULL) {
			ReadNewPoses([this](AugmentedPose const& pose) {
				if (pose.pose.poseIsValid) {
					this->lastPose[pose.deviceId] = pose;
				}
			});

			if (index >= 0 && index < vr::k_unMaxTrackedDeviceCount) {
				pose = lastPose[index].pose;
				if (pSampleTime) *pSampleTime = lastPose[index].sample_time;
				return true;
			}
		}

		void SetPose(int index, const vr::DriverPose_t& pose) {
			if (index >= vr::k_unMaxTrackedDeviceCount) return;
			if (pData == nullptr) return;

			AugmentedPose augPose;
			augPose.deviceId = index;
			augPose.pose = pose;
			QueryPerformanceCounter(&augPose.sample_time);

			uint64_t cur_index = pData->index.load(std::memory_order_relaxed) + 1;
			pData->poses[cur_index % BUFFERED_SAMPLES] = augPose;
			pData->index.store(cur_index, std::memory_order_release);
		}
	};
}