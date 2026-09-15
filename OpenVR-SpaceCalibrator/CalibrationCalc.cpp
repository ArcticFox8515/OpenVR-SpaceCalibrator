#include "CalibrationCalc.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "..\Protocol.h"
#include <algorithm>

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
	return {
		(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
		(lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
		(lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
		(lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
	};
}

namespace {

	inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
		vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
		vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
		auto rotatedVectorQuat = quat * vectorQuat * conjugate;
		return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
	}

	inline Eigen::Matrix3d quaternionRotateMatrix(const vr::HmdQuaternion_t& quat) {
		return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
	}

	struct DSample
	{
		bool valid;
		Eigen::Vector3d ref, target;
	};

	bool StartsWith(const std::string& str, const std::string& prefix)
	{
		if (str.length() < prefix.length())
			return false;

		return str.compare(0, prefix.length(), prefix) == 0;
	}

	bool EndsWith(const std::string& str, const std::string& suffix)
	{
		if (str.length() < suffix.length())
			return false;

		return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
	}

	Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot)
	{
		return Eigen::Vector3d(rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0), rot(1, 0) - rot(0, 1));
	}

	double AngleFromRotationMatrix3(Eigen::Matrix3d rot)
	{
		return acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0);
	}

	vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
	{
		auto euler = eulerdeg * EIGEN_PI / 180.0;

		Eigen::Quaterniond rotQuat =
			Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

		vr::HmdQuaternion_t vrRotQuat;
		vrRotQuat.x = rotQuat.coeffs()[0];
		vrRotQuat.y = rotQuat.coeffs()[1];
		vrRotQuat.z = rotQuat.coeffs()[2];
		vrRotQuat.w = rotQuat.coeffs()[3];
		return vrRotQuat;
	}

	vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
	{
		auto trans = transcm * 0.01;
		vr::HmdVector3d_t vrTrans;
		vrTrans.v[0] = trans[0];
		vrTrans.v[1] = trans[1];
		vrTrans.v[2] = trans[2];
		return vrTrans;
	}

	DSample DeltaRotationSamples(Sample s1, Sample s2)
	{
		// Difference in rotation between samples.
		auto dref = s1.ref.rot * s2.ref.rot.transpose();
		auto dtarget = s1.target.rot * s2.target.rot.transpose();

		// When stuck together, the two tracked objects rotate as a pair,
		// therefore their axes of rotation must be equal between any given pair of samples.
		DSample ds;
		ds.ref = AxisFromRotationMatrix3(dref);
		ds.target = AxisFromRotationMatrix3(dtarget);

		// Reject samples that were too close to each other.
		auto refA = AngleFromRotationMatrix3(dref);
		auto targetA = AngleFromRotationMatrix3(dtarget);
		ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

		ds.ref.normalize();
		ds.target.normalize();
		return ds;
	}

	// Least-squares projection of a rotation matrix onto the yaw-only subgroup.
	// For a pure Ry(t): m(0,2) = sin t, m(2,0) = -sin t, m(0,0) = m(2,2) = cos t,
	// so this is exact for yaw-only input and the closest yaw otherwise.
	double YawFromRotation(const Eigen::Matrix3d& m)
	{
		return std::atan2(m(0, 2) - m(2, 0), m(0, 0) + m(2, 2));
	}

	// Matches the convention used by CalibrateRotation/VRRotationQuat, which build
	// AngleAxisd(yaw, UnitY()) from euler(0, yaw, 0).
	Eigen::Matrix3d YawRotation(double yaw)
	{
		return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()));
	}

	double WrapAngle(double a)
	{
		while (a > EIGEN_PI) a -= 2.0 * EIGEN_PI;
		while (a < -EIGEN_PI) a += 2.0 * EIGEN_PI;
		return a;
	}

	double Median(std::vector<double>& values)
	{
		const size_t n = values.size();
		std::nth_element(values.begin(), values.begin() + n / 2, values.end());
		double hi = values[n / 2];
		if (n % 2 == 1)
			return hi;

		// Even count: average the two central order statistics. nth_element already
		// partitioned, so the lower one is the max of the left partition.
		double lo = *std::max_element(values.begin(), values.begin() + n / 2);
		return 0.5 * (lo + hi);
	}
}

const double CalibrationCalc::AxisVarianceThreshold = 0.001;

// Confidence-classification constants for LastConfidentSampleIndex (locked mode).
// Values derived from labeled drift telemetry: normal one-tick HMD-relative jump
// p99 ~= 17mm, drift spikes are hundreds of mm; clean gap ~20-500mm.
static constexpr double kSpikeJump = 0.08;    // one-tick jump that starts a spike window (m)
static constexpr double kReturnDist = 0.04;   // how close rel must return to pre-spike to recover (m)
static constexpr int kSpikeHold = 5;          // samples (~0.1s) tolerated off-position before accepting new baseline
static constexpr double kRotSpikeJump = 10.0; // one-tick target rotation jump that starts a spike window (degrees)
static constexpr double kRotReturnDist = 5.0; // how close rotation must return to pre-spike anchor to recover (degrees)

// Locked-mode estimator constants (ComputeLockedFromLatest).
//
// Error budget rationale: the locked transform pins the paired target device
// exactly, so residual rotation error shows up at every other target device
// scaled by its distance from that device (~1.3m head-to-foot). 1 degree of
// rotation error is therefore ~2.3cm at the feet, while translation error is
// common-mode and does not get levered. Hence: filter rotation hard, pass
// translation straight through.
static constexpr int kLockedMedianSamples = 5;    // ~0.25s at the 20Hz tick rate
static constexpr double kYawTauSlow = 2.5;        // yaw filter time constant, normal band (s)
static constexpr double kYawTauFast = 0.20;       // yaw filter time constant, fast band (s)
static constexpr double kYawFastEnterDeg = 2.0;   // yaw error that arms fast tracking (degrees)
static constexpr double kYawFastExitDeg = 0.5;    // yaw error at which fast tracking disarms (degrees)
static constexpr int kYawFastHold = 5;            // ticks the error must persist before fast tracking engages

void CalibrationCalc::PushSample(const Sample& sample) {
	m_samples.push_back(sample);
}

void CalibrationCalc::Clear() {
	m_estimatedTransformation.setIdentity();
	m_isValid = false;
	m_samples.clear();
	m_axisVariance = 0.0;
	m_refToTargetPose = Eigen::AffineCompact3d::Identity();
	m_relativePosCalibrated = false;
	m_lockedYawValid = false;
	m_lockedYawFast = false;
	m_lockedYawFastTicks = 0;
	m_lockedYawFiltered = 0.0;
}

void CalibrationCalc::ClearSamples()
{
	m_samples.clear();
}

Eigen::Vector3d CalibrationCalc::CalibrateRotation() const {
	std::vector<DSample> deltas;

	for (size_t i = 0; i < m_samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(m_samples[i], m_samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	//char buf[256];
	//snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", m_samples.size(), deltas.size());
	//CalCtx.Log(buf);

	// Kabsch algorithm

    // Initialize 2D points and centroids
    Eigen::MatrixXd refPoints(deltas.size(), 2), targetPoints(deltas.size(), 2);
    Eigen::Vector2d refCentroid(0, 0), targetCentroid(0, 0);

    // Fill matrices and calculate centroids
    for (size_t i = 0; i < deltas.size(); i++) {
        refPoints.row(i) << deltas[i].ref[0], deltas[i].ref[2];  // Take only the x and z components
        refCentroid += refPoints.row(i);

        targetPoints.row(i) << deltas[i].target[0], deltas[i].target[2];  // Take only the x and z components
        targetCentroid += targetPoints.row(i);
    }

    refCentroid /= (double)deltas.size();
    targetCentroid /= (double)deltas.size();

    // Center the points
    for (size_t i = 0; i < deltas.size(); i++) {
        refPoints.row(i) -= refCentroid;
        targetPoints.row(i) -= targetCentroid;
    }

    // Calculate cross-covariance matrix
    auto crossCV = refPoints.transpose() * targetPoints;

    // Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Calculate 2D rotation matrix
    Eigen::Matrix2d i = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d rot = svd.matrixV() * i * svd.matrixU().transpose();

    // Calculate yaw angle in radians
    double yaw = std::atan2(rot(1, 0), rot(0, 0));

    // Convert to degrees
	Eigen::Vector3d euler(0.0, yaw * 180.0 / EIGEN_PI, 0.0);

	//snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	//CalCtx.Log(buf);
	return euler;
}

Eigen::Vector3d CalibrationCalc::CalibrateTranslation(const Eigen::Matrix3d &rotation) const
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < m_samples.size(); i++)
	{
		Sample s_i = m_samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = rotation * s_i.target.trans;

		for (size_t j = 0; j < i; j++)
		{
			Sample s_j = m_samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = rotation * s_j.target.trans;
			
			auto QAi = s_i.ref.rot.transpose();
			auto QAj = s_j.ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CA, dQA));

			auto QBi = s_i.target.rot.transpose();
			auto QBj = s_j.target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CB, dQB));
		}
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		for (int axis = 0; axis < 3; axis++)
		{
			constants(i * 3 + axis) = deltas[i].first(axis);
			coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
		}
	}

	Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	auto transcm = trans * 100.0;

	//char buf[256];
	//snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	//CalCtx.Log(buf);
	return trans;
}


namespace {
	Pose ApplyTransform(const Pose& originalPose, const Eigen::AffineCompact3d& transform) {
		Pose pose(originalPose);
		pose.rot = transform.rotation() * pose.rot;
		pose.trans = transform * pose.trans;
		return pose;
	}

	 Pose ApplyTransform(const Pose & originalPose, const Eigen::Vector3d & vrTrans, const Eigen::Matrix3d & rotMat) {
		Pose pose(originalPose);
		pose.rot = rotMat * pose.rot;
		pose.trans = vrTrans + (rotMat * pose.trans);
		return pose;
	}
}

Eigen::AffineCompact3d CalibrationCalc::ComputeCalibration() const {
	Eigen::Vector3d rotation = CalibrateRotation();
	Eigen::Matrix3d rotationMat = quaternionRotateMatrix(VRRotationQuat(rotation));
	Eigen::Vector3d translation = CalibrateTranslation(rotationMat);
	
	Eigen::AffineCompact3d rot(rotationMat);
	Eigen::Translation3d trans(translation);

	return trans * rot;
}



double CalibrationCalc::RetargetingErrorRMS(
	const Eigen::Vector3d& hmdToTargetPos,
	const Eigen::AffineCompact3d& calibration
) const {
	double errorAccum = 0;
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, calibration);

		const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * hmdToTargetPos + sample.ref.trans;

		// Compute error term
		double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
		errorAccum += error;
		sampleCount++;
	}

	return sqrt(errorAccum / sampleCount);
}

Eigen::Vector3d CalibrationCalc::ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const {
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, calibration);

		// Now move the transform from world to HMD space
		const auto hmdOriginPos = updatedPose.trans - sample.ref.trans;
		const auto hmdSpace = sample.ref.rot.inverse() * hmdOriginPos;

		accum += hmdSpace;
		sampleCount++;
	}

	accum /= sampleCount;

	return accum;
}

Eigen::Vector4d CalibrationCalc::ComputeAxisVariance(
	const Eigen::AffineCompact3d& calibration
) const {
	// We want to determine if the user rotated in enough axis to find a unique solution.
	// It's sufficient to rotate in two axis - this is because once we constrain the mapping
	// of those two orthogonal basis vectors, the third is determined by the cross product of
	// those two basis vectors. So, the question we then have to answer is - after accounting for
	// translational movement of the HMD itself, are we too close to having only moved on a plane?

	// To determine this, we perform primary component analysis on the rotation quaternions themselves.
	// Since an angle axis quaternion is defined as the sum of Qidentity*cos(angle/2) + Qaxis*sin(angle/2),
	// we expect that rotations around a single axis will have two primary components: One corresponding
	// to the identity component, and one to the axis component. Thus, we check the variance (eigenvalue) of
	// the third primary component to see if we've moved in two axis.
	std::ostringstream dbgStream;

	std::vector<Eigen::Vector4d> points;

	Eigen::Vector4d mean = Eigen::Vector4d::Zero();

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		auto q = Eigen::Quaterniond(sample.target.rot);
		auto point = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
		mean += point;

		points.push_back(point);
	}
	mean /= (double) points.size();

	// Compute covariance matrix
	Eigen::Matrix4d covMatrix = Eigen::Matrix4d::Zero();

	for (auto& point : points) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				covMatrix(i, j) += (point(i) - mean(i)) * (point(j) - mean(j));
			}
		}
	}
	covMatrix /= (double) points.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
	solver.compute(covMatrix);

	return solver.eigenvalues();
}

bool CalibrationCalc::ValidateCalibration(const Eigen::AffineCompact3d &calibration, double *error, Eigen::Vector3d *posOffsetV) {
	bool ok = true;

	const auto posOffset = ComputeRefToTargetOffset(calibration);

	if (posOffsetV) *posOffsetV = posOffset;

	// char buf[256];
	//snprintf(buf, sizeof buf, "HMD to target offset: (%.2f, %.2f, %.2f)\n", posOffset(0), posOffset(1), posOffset(2));
	//CalCtx.Log(buf);

	double rmsError = RetargetingErrorRMS(posOffset, calibration);
	//snprintf(buf, sizeof buf, "Position error (RMS): %.3f\n", rmsError);
	//CalCtx.Log(buf);
	if (rmsError > 0.1) ok = false;

	if (error) *error = rmsError;

	return ok;
}


// Given:
//   R - the reference pose (in reference world space)
//   T - the target pose (in target world space)
//   C - the true calibration (target world -> reference world)
// We assume that there is some "static target pose" S s.t.:
// R * S = C * T (we'll call this the static target pose)
// To compute S:
// S = R^-1 * C * T
// To compute C:
// R * S * T^-1 = C

namespace {
	class PoseAverager {
	private:
		Eigen::Matrix<double, 4, Eigen::Dynamic> quatAvg;
		Eigen::Vector3d accum = Eigen::Vector3d::Zero();
		int i = 0;
	public:
		PoseAverager(size_t n_samples) {
			quatAvg.resize(4, n_samples);
		}

		template<typename P>
		void Push(const P &pose) {
			const Eigen::Quaterniond rot(pose.rotation());
			quatAvg.col(i++) = Eigen::Vector4d(rot.w(), rot.x(), rot.y(), rot.z());
			accum += pose.translation();
		}

		Eigen::AffineCompact3d Average() {
			// https://stackoverflow.com/a/27410865/36723
			auto quatT = quatAvg.transpose();
			Eigen::Matrix4d quatMul = quatAvg * quatT;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
			solver.compute(quatMul);

			Eigen::Vector4d quatAvgV = solver.eigenvectors().col(3).real().normalized();
			Eigen::Quaterniond avgQ(quatAvgV(0), quatAvgV(1), quatAvgV(2), quatAvgV(3));
			avgQ.normalize();

			Eigen::AffineCompact3d pose(avgQ);
			pose.pretranslate(accum * (1.0 / i));

			return pose;
		}

		template<typename XS, typename F>
		static Eigen::AffineCompact3d AverageFor(const XS& samples, const F& poseProvider) {
			int sampleCount = 0;

			for (auto& sample : samples) {
				if (!sample.valid) continue;

				sampleCount++;
			}

			PoseAverager accum(sampleCount);

			for (auto& sample : samples) {
				if (!sample.valid) continue;
				auto pose = poseProvider(sample);
				accum.Push(pose);
			}

			return accum.Average();
		}
	};
}

// S = R^-1 * C * T
Eigen::AffineCompact3d CalibrationCalc::EstimateRefToTargetPose(const Eigen::AffineCompact3d &calibration) const {
	auto avg = PoseAverager::AverageFor(m_samples, [&](const auto& sample) {
		return Eigen::Affine3d(sample.ref.ToAffine().inverse() * calibration * sample.target.ToAffine());
	});

#if 0
	Eigen::Vector3d eulerAvgQ = avg.rotation().eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	Eigen::Vector3d trans = Eigen::Vector3d(avg.translation());

	std::ostringstream oss;
	oss << "==========================================================================================\n";
	oss << "Avg rot: " << eulerAvgQ.x() << ", " << eulerAvgQ.y() << ", " << eulerAvgQ.z() << "\n";
	oss << "Avg trans:\n" << trans.x() << ", " << trans.y() << ", " << trans.z() << "\n";
	OutputDebugStringA(oss.str().c_str());
#endif
	return avg;
}

// S = R^-1 * C * T
// R * S * T^-1 = C

// R * (R^-1 * C * T) * T^-1 = C

/*
 * This calibration routine attempts to use the estimated refToTargetPose to derive the
 * playspace calibration based on the relative position of reference and target device.
 * This computation can be performed even when the devices are not moving.
 */
bool CalibrationCalc::CalibrateByRelPose(Eigen::AffineCompact3d &out) const {
	// R * S * T^-1 = C
	out = PoseAverager::AverageFor(m_samples, [&](const auto& sample) {
		return Eigen::AffineCompact3d(sample.ref.ToAffine() * m_refToTargetPose * sample.target.ToAffine().inverse());
	});

	return true;
}

int CalibrationCalc::LastConfidentSampleIndex(std::vector<int> *goodIndices) const {
	if (goodIndices) goodIndices->clear();

	const int n = (int)m_samples.size();
	if (n == 0) return -1;
	if (n == 1) {
		if (!m_samples[0].valid) return -1;
		if (goodIndices) goodIndices->push_back(0);
		return 0;
	}

	// Walk the window oldest -> newest, tracking spike windows on the HMD-relative
	// target position rel_k = target_k.trans - ref_k.trans. Subtracting ref removes
	// common-mode playspace translation; using consecutive differences cancels any
	// constant bias from an imprecise saved relative transform.
	//
	// A sample is also bad if target.rot jumps by more than kRotSpikeJump degrees
	// relative to the last trusted sample's rotation. This catches transient bad
	// rotation values that leave translation untouched.
	//
	// Both criteria feed into the same state machine:
	//   - "anchor" is the last trusted rel value (the pre-spike baseline).
	//   - When a one-tick jump exceeds kSpikeJump or kRotSpikeJump we enter a spike
	//     window: samples are bad until both rel and rotation return within their
	//     respective return thresholds (recovery) or the spike has persisted
	//     kSpikeHold samples (accept new baseline).
	int lastGood = -1;
	Eigen::Vector3d anchor;
	bool haveAnchor = false;
	int spikeAge = 0; // 0 = not in a spike window
	int spikeStartGood = -1; // lastGood at the moment the spike began

	for (int k = 0; k < n; k++) {
		if (!m_samples[k].valid) {
			// Missing sample: doesn't advance lastGood, doesn't reset spike state.
			continue;
		}
		const Eigen::Vector3d rel = m_samples[k].target.trans - m_samples[k].ref.trans;

		if (!haveAnchor) {
			anchor = rel;
			haveAnchor = true;
			lastGood = k;
			if (goodIndices) goodIndices->push_back(k);
			continue;
		}

		// Rotation jump relative to the last trusted sample.
		const Eigen::Quaterniond qCur(m_samples[k].target.rot);
		const Eigen::Quaterniond qAnchor(m_samples[lastGood].target.rot);
		double dot = std::abs(qCur.coeffs().dot(qAnchor.coeffs()));
		if (dot > 1.0) dot = 1.0;
		const double rotJumpDeg = 2.0 * std::acos(dot) * 180.0 / EIGEN_PI;

		if (spikeAge == 0) {
			double jump = (rel - anchor).norm();
			if (jump > kSpikeJump || rotJumpDeg > kRotSpikeJump) {
				// Spike begins; this sample is provisionally bad.
				spikeAge = 1;
				spikeStartGood = lastGood;
			} else {
				// Stable: trust it and slide the anchor forward.
				anchor = rel;
				lastGood = k;
				if (goodIndices) goodIndices->push_back(k);
			}
		} else {
			if ((rel - anchor).norm() < kReturnDist && rotJumpDeg < kRotReturnDist) {
				// Recovered: both translation and rotation are back near the anchor.
				spikeAge = 0;
				anchor = rel;
				lastGood = k;
				if (goodIndices) goodIndices->push_back(k);
			} else if (spikeAge >= kSpikeHold) {
				// Persisted past the hold window: accept as a new steady state
				// (playspace reset / settled position). Re-anchor here.
				spikeAge = 0;
				anchor = rel;
				lastGood = k;
				if (goodIndices) goodIndices->push_back(k);
			} else {
				// Still inside an unresolved spike: keep rolling back to the last
				// good sample from before the spike started.
				spikeAge++;
				lastGood = spikeStartGood;
			}
		}
	}

	return lastGood;
}

bool CalibrationCalc::ComputeLockedFromLatest(double dt) {
	// Preconditions: the relative transform is locked and we have a fresh sample.
	// We gate on lockRelativePosition alone (matching the original locked branch
	// in ComputeIncremental) - the lock is the user asserting the stored relative
	// transform is good, regardless of the m_relativePosCalibrated flag (which is
	// reset to false by EndContinuousCalibration and is often false on load).
	if (!lockRelativePosition)
		return false;
	if (m_samples.empty())
		return false;

	// Roll back to the samples we are confident in. When recent samples are
	// corrupted by a tracking spike or an in-progress drift slide, the state
	// machine excludes them and we fall back to earlier good ones.
	static thread_local std::vector<int> good;
	good.clear();
	const int idx = LastConfidentSampleIndex(&good);
	if (idx < 0 || good.empty()) return false;

	// Use the newest few confident samples so a single noisy pose cannot move the
	// playspace. At the 20Hz tick rate this is ~0.25s of history, which is short
	// enough that the correction is still effectively instant.
	const size_t count = std::min<size_t>(good.size(), kLockedMedianSamples);
	const size_t first = good.size() - count;

	// Per-sample yaw of the full estimate C = R * S * T^-1, projected onto
	// yaw-only. Both tracking systems are gravity-aligned via their IMUs, so the
	// true C has no pitch or roll - anything we measure there is pure error, and
	// letting it through tilts the entire target playspace.
	const Eigen::Matrix3d sRot = m_refToTargetPose.rotation();

	std::vector<double> yaws;
	yaws.reserve(count);

	const double newestYaw = YawFromRotation(
		Eigen::Matrix3d(m_samples[good.back()].ref.rot * sRot * m_samples[good.back()].target.rot.transpose()));

	for (size_t i = first; i < good.size(); i++) {
		const Sample &s = m_samples[good[i]];
		const double yaw = YawFromRotation(Eigen::Matrix3d(s.ref.rot * sRot * s.target.rot.transpose()));
		// Median in the wrapped-difference domain relative to the newest sample so
		// the wrap point cannot split the set.
		yaws.push_back(WrapAngle(yaw - newestYaw));
	}

	const double yawRaw = WrapAngle(newestYaw + Median(yaws));

	// Yaw low-pass. Real yaw drift between two gravity-aligned systems is slow
	// (SLAM heading error accumulating), so heavy damping costs nothing. A large
	// sustained error means an actual re-localisation event, which we do want to
	// chase quickly - hence the banded time constant with hysteresis.
	if (!m_lockedYawValid) {
		m_lockedYawFiltered = yawRaw;
		m_lockedYawValid = true;
		m_lockedYawFast = false;
		m_lockedYawFastTicks = 0;
	}
	else {
		const double err = WrapAngle(yawRaw - m_lockedYawFiltered);
		const double errDeg = std::abs(err) * 180.0 / EIGEN_PI;

		if (m_lockedYawFast) {
			if (errDeg < kYawFastExitDeg) {
				m_lockedYawFast = false;
				m_lockedYawFastTicks = 0;
			}
		}
		else if (errDeg > kYawFastEnterDeg) {
			if (++m_lockedYawFastTicks >= kYawFastHold)
				m_lockedYawFast = true;
		}
		else {
			m_lockedYawFastTicks = 0;
		}

		const double tau = m_lockedYawFast ? kYawTauFast : kYawTauSlow;
		// Parenthesised to dodge the Windows min/max macros pulled in via Calibration.h.
		const double clampedDt = (std::max)(0.001, (std::min)(dt, 0.5));
		const double alpha = 1.0 - std::exp(-clampedDt / tau);
		m_lockedYawFiltered = WrapAngle(m_lockedYawFiltered + alpha * err);
	}

	const Eigen::Matrix3d rotC = YawRotation(m_lockedYawFiltered);

	// Translation, instant. Derived from the final rotation rather than taken from
	// R * S * T^-1, so the paired target device still lands exactly on R * S no
	// matter how far the filtered yaw lags the raw measurement:
	//   C * T = R * S  =>  t_C = (R * S).trans - rotC * T.trans
	std::vector<double> tx, ty, tz;
	tx.reserve(count); ty.reserve(count); tz.reserve(count);

	for (size_t i = first; i < good.size(); i++) {
		const Sample &s = m_samples[good[i]];
		const Eigen::Vector3d refTargetWorld = s.ref.trans + s.ref.rot * m_refToTargetPose.translation();
		const Eigen::Vector3d t = refTargetWorld - rotC * s.target.trans;
		tx.push_back(t.x()); ty.push_back(t.y()); tz.push_back(t.z());
	}

	const Eigen::Vector3d transC(Median(tx), Median(ty), Median(tz));

	Eigen::AffineCompact3d calibration;
	calibration.linear() = rotC;
	calibration.translation() = transC;

	m_estimatedTransformation = calibration;
	m_isValid = true;
	return true;
}

bool CalibrationCalc::ComputeOneshot() {
	auto calibration = ComputeCalibration();

	bool valid = ValidateCalibration(calibration);

	if (valid) {
		m_estimatedTransformation = calibration;
		m_isValid = true;
		return true;
	}
	else {
		CalCtx.Log("Not updating: Low-quality calibration result\n");
		return false;
	}
}

void CalibrationCalc::ComputeInstantOffset() {
	const auto &latestSample = m_samples.back();

	// Apply transformation
	const auto updatedPose = ApplyTransform(latestSample.target, m_estimatedTransformation);

	// Now move the transform from world to HMD space
	const auto hmdOriginPos = updatedPose.trans - latestSample.ref.trans;
	const auto hmdSpace = latestSample.ref.rot.inverse() * hmdOriginPos;
	
	Metrics::posOffset_lastSample.Push(hmdSpace * 1000);
}

bool CalibrationCalc::ComputeIncremental(bool &lerp, double threshold) {


	Metrics::RecordTimestamp();

	if (lockRelativePosition) {
		Eigen::AffineCompact3d byRelPose;
		double relPoseError = INFINITY;
		Eigen::Vector3d relPosOffset;
		CalibrateByRelPose(byRelPose);
		ValidateCalibration(byRelPose, &relPoseError, &relPosOffset);
		Metrics::posOffset_byRelPose.Push(relPosOffset * 1000);
		Metrics::error_byRelPose.Push(relPoseError * 1000);

		m_isValid = true;
		m_estimatedTransformation = byRelPose;
		return true;
	}

	double priorCalibrationError = INFINITY;
	Eigen::Vector3d priorPosOffset;
    if (m_isValid) {
        ValidateCalibration(m_estimatedTransformation, &priorCalibrationError, &priorPosOffset);
		
        Metrics::posOffset_currentCal.Push(priorPosOffset * 1000);
        Metrics::error_currentCal.Push(priorCalibrationError * 1000);

        if (priorCalibrationError < 0.005) {
            return false;
        }
    }
	double newError = INFINITY;
	bool newCalibrationValid = false;
    Eigen::AffineCompact3d byRelPose;
    Eigen::AffineCompact3d calibration;
	bool usingRelPose = false;
    double relPoseError = INFINITY;

    if (enableStaticRecalibration && CalibrateByRelPose(byRelPose)) {
		Eigen::Vector3d relPosOffset;
		ValidateCalibration(byRelPose, &relPoseError, &relPosOffset);
        Metrics::posOffset_byRelPose.Push(relPosOffset * 1000);
        Metrics::error_byRelPose.Push(relPoseError * 1000);
		
		if (relPoseError < 0.010 || m_relativePosCalibrated && relPoseError < 0.025) {
			newCalibrationValid = true;
			usingRelPose = true;
			newError = relPoseError;
			calibration = byRelPose;
			if (relPoseError * threshold >= priorCalibrationError) {
				return false;
			}
		}
	}

	double newVariance = 0;
    if (!newCalibrationValid) {
        calibration = ComputeCalibration();

        newVariance = ComputeAxisVariance(calibration)(1);
		Metrics::axisIndependence.Push(newVariance);

        if (newVariance < AxisVarianceThreshold && newVariance < m_axisVariance) {
            newCalibrationValid = false;
        } else {
            newCalibrationValid = ValidateCalibration(calibration, &newError, &m_posOffset);
            Metrics::posOffset_rawComputed.Push(m_posOffset * 1000);
        }

        if (m_isValid) {
            if (priorCalibrationError < newError * threshold) {
                // If we have a more noisy calibration than before, avoid updating.
                newCalibrationValid = false;
            }
        }

        Metrics::error_rawComputed.Push(newError * 1000);
		
		ComputeInstantOffset();
    }


	

#if 0
		char tmp[256];
		snprintf(tmp, sizeof tmp, "Prior calibration error: %.3f (valid: %s) sct %d; new error %.3f; new better? %s\n",
			priorCalibrationError, m_isValid ? "yes" : "no", stableCt, newError, !oldCalibrationBetter ? "yes" : "no");
		CalCtx.Log(tmp);
#endif
		
	
	// Now, can we use the relative pose to perform a rapid correction?
    if (!newCalibrationValid) {
		
        double existingPoseErrorUsingRelPosition = RetargetingErrorRMS(m_refToTargetPose.translation(), m_estimatedTransformation);
        Metrics::error_currentCalRelPose.Push(existingPoseErrorUsingRelPosition * 1000);
		if (relPoseError * threshold < existingPoseErrorUsingRelPosition || newCalibrationValid && relPoseError < newError) {
		newCalibrationValid = true;
        usingRelPose = true;
        newError = relPoseError;
        calibration = byRelPose;
		}
    }

	if (newCalibrationValid) {
		lerp = m_isValid;
		m_relativePosCalibrated = m_relativePosCalibrated || newError < 0.005;
		if (!m_isValid) {
			CalCtx.Log("Applying initial transformation...");
		}
		else if (m_relativePosCalibrated) {
			CalCtx.Log("Applying updated transformation...");
		} else {
			CalCtx.Log("Applying temporary transformation...");
		}
		
		m_isValid = true;
		m_estimatedTransformation = calibration;
		m_axisVariance = newVariance;

		if (!usingRelPose) {
			m_refToTargetPose = EstimateRefToTargetPose(m_estimatedTransformation);
		}

		Metrics::calibrationApplied.Push(!usingRelPose);

		return true;
	}
	else {
		return false;
	}
}


