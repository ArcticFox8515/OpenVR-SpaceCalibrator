#pragma once

#include <Eigen/Dense>
#include <openvr.h>
#include <vector>
#include <deque>
#include <iostream>

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(const Eigen::AffineCompact3d& transform) {
		rot = transform.rotation();
		trans = transform.translation();
	}
	
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i, j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(vr::HmdQuaternion_t rot, const double *trans) {
		this->rot = Eigen::Matrix3d(Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z));
		this->trans = Eigen::Vector3d(trans[0], trans[1], trans[2]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x, y, z)) { }

	Eigen::Matrix4d ToAffine() const {
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				matrix(i, j) = rot(i, j);
			}
			matrix(i, 3) = trans(i);
		}

		return matrix;
	}
};

struct Sample
{
	Pose ref, target;
	bool valid;
	Sample() : valid(false) { }
	Sample(Pose ref, Pose target) : valid(true), ref(ref), target(target) { }
};

class CalibrationCalc {
public:
	static const double AxisVarianceThreshold;

	bool enableStaticRecalibration;
	bool lockRelativePosition = false;
	
	const Eigen::AffineCompact3d Transformation() const 
	{
		return m_estimatedTransformation;
	}

	const Eigen::Vector3d EulerRotation() const {
		auto rot = m_estimatedTransformation.rotation();
		return rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	}

	bool isValid() const {
		return m_isValid;
	}
	
	const Eigen::AffineCompact3d RelativeTransformation() const 
	{
		return m_refToTargetPose;
	}

	bool isRelativeTransformationCalibrated() const
	{
		return m_relativePosCalibrated;
	}

	void setRelativeTransformation(const Eigen::AffineCompact3d transform, bool calibrated)
	{
		m_refToTargetPose = transform;
		m_relativePosCalibrated = calibrated;
	}

	void PushSample(const Sample& sample);
	void Clear();
	void ClearSamples();

	bool ComputeOneshot();
	bool ComputeIncremental(bool &lerp, double threshold);

	// Fast continuous path: when the relative transform is locked and already
	// calibrated, derive the playspace calibration from a single (the latest)
	// sample instead of collecting and averaging a full batch. Returns false if
	// preconditions aren't met (no samples / relative pose not yet established),
	// in which case the caller should fall back to the regular logic.
	bool ComputeLockedFromLatest();

	size_t SampleCount() const {
		return m_samples.size();
	}

	void ShiftSample() {
		if (!m_samples.empty()) m_samples.pop_front();
	}

	CalibrationCalc() : m_isValid(false), m_calcCycle(0), enableStaticRecalibration(true) {}

	// Debug fields
	Eigen::Vector3d m_posOffset;
	double m_axisVariance = 0.0;
	long m_calcCycle;

private:
	bool m_isValid;
	Eigen::AffineCompact3d m_estimatedTransformation;
	bool m_relativePosCalibrated = false;

	/*
	 * This affine transform estimates the pose of the target within the reference device's local pose space.
	 * That is to say, it's given by transforming the target world pose by the inverse reference pose.
	 */
	Eigen::AffineCompact3d m_refToTargetPose = Eigen::AffineCompact3d::Identity();

	std::deque<Sample> m_samples;

	Eigen::Vector3d CalibrateRotation() const;
	Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d &rotation) const;

	Eigen::AffineCompact3d ComputeCalibration() const;

	double RetargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const;
	Eigen::Vector3d ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;

	Eigen::Vector4d ComputeAxisVariance(const Eigen::AffineCompact3d& calibration) const;

	bool ValidateCalibration(const Eigen::AffineCompact3d& calibration, double *errorOut = nullptr, Eigen::Vector3d* posOffsetV = nullptr);
	void ComputeInstantOffset();

	Eigen::AffineCompact3d EstimateRefToTargetPose(const Eigen::AffineCompact3d& calibration) const;
	bool CalibrateByRelPose(Eigen::AffineCompact3d &out) const;

	// Returns the index into m_samples of the newest sample whose HMD-relative
	// target position is trustworthy, or -1 if none is. Used by
	// ComputeLockedFromLatest to roll back to the last good pose when the latest
	// sample is corrupted by tracking instability (spikes / drift slides).
	//
	// Rule (operating on rel_k = target_k.trans - ref_k.trans, which cancels
	// common-mode playspace translation):
	//  - A one-tick jump |rel_k - rel_{k-1}| > kSpikeJump starts a "spike window".
	//  - The following up to kSpikeHold samples are bad unless rel returns within
	//    kReturnDist of the pre-spike value (tracker recovered -> samples good).
	//  - If the displacement persists beyond kSpikeHold samples, it is accepted as
	//    a new steady state (playspace reset / settled drift) and samples are good
	//    again, anchored to the new baseline.
	int LastConfidentSampleIndex() const;
};