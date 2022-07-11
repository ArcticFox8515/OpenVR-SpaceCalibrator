#pragma once

#include <Eigen/Dense>
#include <openvr.h>
#include <vector>
#include <iostream>

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i, j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
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

	void PushSample(const Sample& sample);
	void Clear();

	bool ComputeOneshot();
	bool ComputeIncremental();

	size_t SampleCount() const {
		return m_samples.size();
	}

	CalibrationCalc() : m_isValid(false) {}
private:
	bool m_isValid;
	Eigen::AffineCompact3d m_estimatedTransformation;

	std::vector<Sample> m_samples;

	Eigen::Vector3d CalibrateRotation() const;
	Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d &rotation) const;

	Eigen::AffineCompact3d ComputeCalibration() const;

	double RetargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos, const Eigen::AffineCompact3d& calibration) const;
	Eigen::Vector3d ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const;

	Eigen::Vector3d ComputeIndependence(const Eigen::AffineCompact3d& calibration) const;

	bool ValidateCalibration(const Eigen::AffineCompact3d& calibration);

};