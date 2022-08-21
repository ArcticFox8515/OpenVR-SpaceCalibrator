#pragma once 
#define EIGEN_MPL2_ONLY

#include <Eigen/Dense>

/**
 * Contains an isometric transformation, represented as the pair of a rotation quaternion and translation vector.
 * The translation is applied to the left of the quaternion.
 */
struct IsoTransform {
	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;

	IsoTransform() : rotation(Eigen::Quaterniond::Identity()), translation(Eigen::Vector3d::Zero()) {}
	IsoTransform(const Eigen::Quaterniond &rot) : rotation(rot), translation(Eigen::Vector3d::Zero()) {}
	IsoTransform(const Eigen::Vector3d &trans) : rotation(Eigen::Quaterniond::Identity()) {}
	IsoTransform(const Eigen::Quaterniond& rot, const Eigen::Vector3d& trans) : rotation(rot), translation(trans) {}
	
	void pretranslate(const Eigen::Vector3d& t) {
		translation += t;
	}

	/**
	 * Interpolates between this transform and target. The position of localPoint after transformation will smoothly
	 * lerp between (this * localPoint) and (target * localPoint), despite rotation occurring around it.
	 */
	IsoTransform interpolateAround(double lerp, const IsoTransform& target, const Eigen::Vector3d& localPoint) const;
};

inline IsoTransform operator*(const IsoTransform& a, const IsoTransform& b) {
	// tA * rA * tB * rB = tA * (trans(rA * tB)) * rA * rB
	auto rot = a.rotation * b.rotation;
	Eigen::Vector3d trans = a.translation + Eigen::Isometry3d(a.rotation) * b.translation;

	return IsoTransform(rot, trans);
}

inline Eigen::Vector3d operator*(const IsoTransform& a, const Eigen::Vector3d& p) {
	return a.translation + Eigen::Isometry3d(a.rotation) * p;
}

inline IsoTransform IsoTransform::interpolateAround(double lerp, const IsoTransform& target, const Eigen::Vector3d& localPoint) const {
	auto initialPos = (*this) * localPoint;
	Eigen::Vector3d finalPos = initialPos * (1 - lerp) + (target * localPoint) * lerp;

	auto newRotation = rotation.slerp(lerp, target.rotation);
	Eigen::Vector3d newTranslation = finalPos - Eigen::Isometry3d(newRotation) * localPoint;

	return IsoTransform(newRotation, newTranslation);
}