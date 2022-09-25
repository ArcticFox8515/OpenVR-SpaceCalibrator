#include "CalibrationCalc.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "..\Protocol.h"

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
}

const double CalibrationCalc::AxisVarianceThreshold = 0.001;
void CalibrationCalc::PushSample(const Sample& sample) {
	m_samples.push_back(sample);
}

void CalibrationCalc::Clear() {
	m_estimatedTransformation.setIdentity();
	m_isValid = false;
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

	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
	Eigen::Vector3d refCentroid(0, 0, 0), targetCentroid(0, 0, 0);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) = deltas[i].ref;
		refCentroid += deltas[i].ref;

		targetPoints.row(i) = deltas[i].target;
		targetCentroid += deltas[i].target;
	}

	refCentroid /= (double)deltas.size();
	targetCentroid /= (double)deltas.size();

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
	{
		i(2, 2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

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
	if (!m_refToTargetPoseValid) return false;

	out = PoseAverager::AverageFor(m_samples, [&](const auto& sample) {
		return Eigen::AffineCompact3d(sample.ref.ToAffine() * m_refToTargetPose * sample.target.ToAffine().inverse());
	});

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

	auto calibration = ComputeCalibration();

	bool usingRelPose = false;
	bool valid = true;
	auto variance = ComputeAxisVariance(calibration);
	//std::ostringstream oss;
	//oss << "Axis variance: " << variance(0) << " " << variance(1) << " " << variance(2) << " " << variance(3) << "\n";
	//CalCtx.Log(oss.str());
	//oss.clear();
	m_axisVariance = variance(1);

	if (m_axisVariance < AxisVarianceThreshold) {
		//CalCtx.Log("Calibration points are nearly coplanar. Try moving around more?\n");
		valid = false;
	}
	Metrics::axisIndependence.Push(m_axisVariance);

	double newError = INFINITY, priorCalibrationError = INFINITY;
	if (valid) {
		valid = ValidateCalibration(calibration, &newError, &m_posOffset);
		m_newCalRMS = newError;
		Metrics::posOffset_rawComputed.Push(m_posOffset * 1000);
	}

	Metrics::error_rawComputed.Push(newError * 1000);
	
	// Use stricter thresholds for continuous calibration to limit jitter
	valid = valid && newError < 0.005;

	Eigen::Vector3d priorPosOffset;
	ValidateCalibration(m_estimatedTransformation, &priorCalibrationError, &priorPosOffset);
	m_oldCalRMS = priorCalibrationError;
	Metrics::posOffset_currentCal.Push(priorPosOffset * 1000);
	Metrics::error_currentCal.Push(priorCalibrationError * 1000);

	ComputeInstantOffset();
	
	bool ok = valid;
	
	bool oldCalibrationBetter = !valid || (m_isValid && priorCalibrationError < newError * threshold);

#if 0
		char tmp[256];
		snprintf(tmp, sizeof tmp, "Prior calibration error: %.3f (valid: %s) sct %d; new error %.3f; new better? %s\n",
			priorCalibrationError, m_isValid ? "yes" : "no", stableCt, newError, !oldCalibrationBetter ? "yes" : "no");
		CalCtx.Log(tmp);
#endif
		
	// If we have a more noisy calibration than before, avoid updating.
	if (oldCalibrationBetter) ok = false;
	
	// Now, can we use the relative pose to perform a rapid correction?
	Eigen::AffineCompact3d byRelPose;
	bool relPoseAvailable = CalibrateByRelPose(byRelPose);
	double relPoseError = INFINITY, existingPoseErrorUsingRelPosition = 0;
	Eigen::Vector3d relPosOffset;
	bool relPoseValid;
	if (relPoseAvailable && enableStaticRecalibration) {
		relPoseValid = ValidateCalibration(byRelPose, &relPoseError, &relPosOffset);
		Metrics::posOffset_byRelPose.Push(relPosOffset * 1000);
		Metrics::error_byRelPose.Push(relPoseError * 1000);
		existingPoseErrorUsingRelPosition = RetargetingErrorRMS(m_refToTargetPose.translation(), m_estimatedTransformation);
		Metrics::error_currentCalRelPose.Push(existingPoseErrorUsingRelPosition * 1000);
	}
	else {
		relPoseValid = false;
	}
	if (!ok && relPoseValid && relPoseError * threshold < existingPoseErrorUsingRelPosition) {
		usingRelPose = true;
		newError = relPoseError;
		calibration = byRelPose;
		ok = true;
	}

	if (ok) {
		lerp = m_isValid;
		if (!m_isValid) {
			CalCtx.Log("Applying initial transformation...");
		}
		else {
			CalCtx.Log("Applying updated transformation...");
		}
		
		m_isValid = true;
		m_estimatedTransformation = calibration;

		if (!usingRelPose) {
			m_refToTargetPose = EstimateRefToTargetPose(m_estimatedTransformation);
			m_refToTargetPoseValid = true;
		}

		Metrics::calibrationApplied.Push(!usingRelPose);

		return true;
	}
	else {
		return false;
	}
}


