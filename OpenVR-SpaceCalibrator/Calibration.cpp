#include "stdafx.h"
#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"

#include <string>
#include <vector>
#include <iostream>

#include <Eigen/Dense>

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
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

inline Eigen::Matrix3d quaternionRotateMatrix(const vr::HmdQuaternion_t& quat) {
	return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
}


IPCClient Driver;
static protocol::DriverPoseShmem shmem;
CalibrationContext CalCtx;

void InitCalibrator()
{
	Driver.Connect();
	shmem.Open(OPENVR_SPACECALIBRATOR_SHMEM_NAME);
}

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i,j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x,y,z)) { }

	Eigen::Matrix4d ToAffine() const {
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				matrix(i,j) = rot(i, j);
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

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string &str, const std::string &prefix)
{
	if (str.length() < prefix.length())
		return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
		return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot)
{
	return Eigen::Vector3d(rot(2,1) - rot(1,2), rot(0,2) - rot(2,0), rot(1,0) - rot(0,1));
}

double AngleFromRotationMatrix3(Eigen::Matrix3d rot)
{
	return acos((rot(0,0) + rot(1,1) + rot(2,2) - 1.0) / 2.0);
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

Eigen::Vector3d CalibrateRotation(const std::vector<Sample> &samples)
{
	std::vector<DSample> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	char buf[256];
	snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
	CalCtx.Log(buf);

	// Kabsch algorithm

	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
	Eigen::Vector3d refCentroid(0,0,0), targetCentroid(0,0,0);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) = deltas[i].ref;
		refCentroid += deltas[i].ref;

		targetPoints.row(i) = deltas[i].target;
		targetCentroid += deltas[i].target;
	}

	refCentroid /= (double) deltas.size();
	targetCentroid /= (double) deltas.size();

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
		i(2,2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

	snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	CalCtx.Log(buf);
	return euler;
}

Eigen::Vector3d CalibrateTranslation(const std::vector<Sample> &samples)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto QAi = samples[i].ref.rot.transpose();
			auto QAj = samples[j].ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (samples[j].ref.trans - samples[j].target.trans) - QAi * (samples[i].ref.trans - samples[i].target.trans);
			deltas.push_back(std::make_pair(CA, dQA));

			auto QBi = samples[i].target.rot.transpose();
			auto QBj = samples[j].target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (samples[j].ref.trans - samples[j].target.trans) - QBi * (samples[i].ref.trans - samples[i].target.trans);
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

	char buf[256];
	snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	CalCtx.Log(buf);
	return transcm;
}

static vr::TrackedDevicePose_t ConvertPose(const vr::DriverPose_t& pose) {
	vr::TrackedDevicePose_t outPose;

	outPose.bDeviceIsConnected = true;
	outPose.bPoseIsValid = pose.poseIsValid;
	outPose.eTrackingResult = pose.result;

	Eigen::Quaterniond rot = Eigen::Quaterniond(pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
	Eigen::Vector3d pos = Eigen::Vector3d(pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);

	Eigen::AffineCompact3d transform = Eigen::Translation3d(pos) * rot;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			outPose.mDeviceToAbsoluteTracking.m[i][j] = transform(i, j);
		}
		outPose.vAngularVelocity.v[i] = pose.vecAngularVelocity[i];
		outPose.vVelocity.v[i] = pose.vecVelocity[i];
	}

	return outPose;
}

Sample CollectSample(const CalibrationContext &ctx)
{
	vr::TrackedDevicePose_t reference, target;
	reference.bPoseIsValid = false;
	target.bPoseIsValid = false;

	reference = ctx.devicePoses[ctx.referenceID];
	target = ctx.devicePoses[ctx.targetID];

	bool ok = true;
	if (!reference.bPoseIsValid)
	{
		CalCtx.Log("Reference device is not tracking\n"); ok = false;
	}
	if (!target.bPoseIsValid)
	{
		CalCtx.Log("Target device is not tracking\n"); ok = false;
	}
	if (!ok)
	{
		CalCtx.Log("Aborting calibration!\n");
		CalCtx.state = CalibrationState::None;
		return Sample();
	}

	return Sample(
		Pose(reference.mDeviceToAbsoluteTracking),
		Pose(target.mDeviceToAbsoluteTracking)
	);
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

void ResetAndDisableOffsets(uint32_t id)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	Driver.SendBlocking(req);
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		/*if (deviceClass == vr::TrackedDeviceClass_HMD) // for debugging unexpected universe switches
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			auto universeId = vr::VRSystem()->GetUint64TrackedDeviceProperty(id, vr::Prop_CurrentUniverseId_Uint64, &err);
			printf("uid %d err %d\n", universeId, err);
			ResetAndDisableOffsets(id);
			continue;
		}*/

		if (!ctx.enabled)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

		if (err != vr::TrackedProp_Success)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		std::string trackingSystem(buffer);

		if (id == vr::k_unTrackedDeviceIndex_Hmd)
		{
			//auto p = ctx.devicePoses[id].mDeviceToAbsoluteTracking.m;
			//printf("HMD %d: %f %f %f\n", id, p[0][3], p[1][3], p[2][3]);

			if (trackingSystem != ctx.referenceTrackingSystem)
			{
				// Currently using an HMD with a different tracking system than the calibration.
				ctx.enabled = false;
			}

			ResetAndDisableOffsets(id);
			continue;
		}

		if (trackingSystem != ctx.targetTrackingSystem)
		{
			ResetAndDisableOffsets(id);
			continue;
		}

		protocol::Request req(protocol::RequestSetDeviceTransform);
		req.setDeviceTransform = {
			id,
			true,
			VRTranslationVec(ctx.calibratedTranslation),
			VRRotationQuat(ctx.calibratedRotation),
			ctx.calibratedScale
		};
		Driver.SendBlocking(req);
	}

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

Pose ApplyTransform(const Pose& originalPose, const Eigen::Vector3d& vrTrans, const Eigen::Matrix3d& rotMat) {
	Pose pose(originalPose);
	pose.rot = rotMat * pose.rot;
	pose.trans = vrTrans + (rotMat * pose.trans);
	return pose;
}

double RetargetingErrorRMS(
	const std::vector<Sample>& samples,
	const Eigen::Vector4d &hmdToTargetPos,
	const vr::HmdVector3d_t& vrTrans,
	const vr::HmdQuaternion_t& vrRotQuat
) {
	const auto rotMat = quaternionRotateMatrix(vrRotQuat);
	const auto trans = Eigen::Vector3d(vrTrans.v);

	double errorAccum = 0;
	int sampleCount = 0;

	for (auto& sample : samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, trans, rotMat);
		//const Eigen::Vector4d targetToWorld = Eigen::Vector4d(updatedPose.trans(0), updatedPose.trans(1), updatedPose.trans(2), 1);

		// Now compute it based on the HMD pose offset
		//const Eigen::Vector4d hmdAffine = sample.ref.ToAffine() * hmdToTargetPos;
		const auto fixedPose = Eigen::Vector3d(hmdToTargetPos(0), hmdToTargetPos(1), hmdToTargetPos(2));
		const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * fixedPose + sample.ref.trans;

		// Compute error term
		double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
		errorAccum += error;
		sampleCount++;
	}

	printf("=================================================\n");

	return sqrt(errorAccum / sampleCount);
}

Eigen::Vector4d DeriveRefToTargetOffset(
	const std::vector<Sample>& samples,
	const vr::HmdVector3d_t& vrTrans,
	const vr::HmdQuaternion_t& vrRotQuat
) {
	const auto rotMat = quaternionRotateMatrix(vrRotQuat);
	const auto trans = Eigen::Vector3d(vrTrans.v);

	Eigen::Vector3d accum = Eigen::Vector3d::Zero();
	int sampleCount = 0;

	for (auto& sample : samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, trans, rotMat);

		// Now move the transform from world to HMD space
		const auto hmdOriginPos = updatedPose.trans - sample.ref.trans;
		const auto hmdSpace = sample.ref.rot.inverse() * hmdOriginPos;
		/*
		const auto trans4 = Eigen::Vector4d(updatedPose.trans(0), updatedPose.trans(1), updatedPose.trans(2), 1);
		const auto hmdSpace = sample.ref.ToAffine().inverse() * trans4;*/

		//char tmpBuf[256];
		//snprintf(tmpBuf, sizeof tmpBuf, "hmdSpace: [%.2f %.2f %.2f]\n", hmdSpace(0), hmdSpace(1), hmdSpace(2));
		//OutputDebugStringA(tmpBuf);

		accum += hmdSpace;
		sampleCount++;
	}

	accum /= sampleCount;
	//accum(3) = 1; // Ensure we're precisely in affine form

	return Eigen::Vector4d(accum(0), accum(1), accum(2), 1);
}

bool ComputeIndependence(
	CalibrationContext& CalCtx,
	const std::vector<Sample>& samples,
	const vr::HmdVector3d_t& vrTrans,
	const vr::HmdQuaternion_t& vrRotQuat
) {
	// We want to determine if the user rotated in enough axis to find a unique solution.
	// It's sufficient to rotate in two axis - this is because once we constrain the mapping
	// of those two orthogonal basis vectors, the third is determined by the cross product of
	// those two basis vectors. So, the question we then have to answer is - after accounting for
	// translational movement of the HMD itself, are we too close to having only moved on a plane?

	// To determine this, we perform primary component analysis on the tracked device offset relative
	// to ref position. This means we first have to translate both to world space, then subtract ref
	// position.
	std::ostringstream dbgStream;

	std::vector<Eigen::Vector3d> relOffsetPoints;
	const auto rotMat = quaternionRotateMatrix(vrRotQuat);
	const auto trans = Eigen::Vector3d(vrTrans.v);

	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	double meanDist = 0;

	for (auto &sample : samples) {
		if (!sample.valid) continue;

		auto point = (rotMat * sample.target.trans + trans) - sample.ref.trans;
		mean += point;
		meanDist += point.norm();

		//dbgStream << "Indep: " << point << "\n";

		relOffsetPoints.push_back(point);
	}
	mean /= relOffsetPoints.size();
	meanDist /= relOffsetPoints.size();

	// Compute covariance matrix
	Eigen::Matrix3d covMatrix = Eigen::Matrix3d::Zero();

	for (auto& sample : relOffsetPoints) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				covMatrix(i, j) += (sample(i) - mean(i)) * (sample(j) - mean(j));
			}
		}
	}
	covMatrix /= relOffsetPoints.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
	solver.compute(covMatrix);

	dbgStream << "[Independence Solver]\nEigenValues: " << solver.eigenvalues() << "\n";
	dbgStream << "EigenVectors:\n" << solver.eigenvectors() << "\n";

	// Perform change of basis
	Eigen::Matrix3d basis = solver.eigenvectors().real();
	for (int i = 0; i < 3; i++) {
		basis.col(i) = basis.col(i).normalized();
	}

	Eigen::Matrix3d changeBasis = basis.inverse();

	// Compute standard deviation on each axis
	Eigen::Vector3d newBasisMean = Eigen::Vector3d::Zero();
	for (auto& sample : relOffsetPoints) {
		sample /= meanDist;

		auto inNewBasis = changeBasis * sample;
		newBasisMean += inNewBasis;
	}
	newBasisMean /= relOffsetPoints.size();
	Eigen::Vector3d sumDeviation = Eigen::Vector3d::Zero();
	for (auto& sample : relOffsetPoints) {
		auto inNewBasis = changeBasis * sample;
		auto diff = newBasisMean - inNewBasis;
		
		for (int i = 0; i < 3; i++) {
			sumDeviation(i) += diff(i) * diff(i);
		}
	}

	Eigen::Vector3d stddev = sumDeviation / relOffsetPoints.size();

	dbgStream << "Axis deviation: " << stddev << "\n";

	OutputDebugStringA(dbgStream.str().c_str());

	if (stddev(0) < 0.00005) {
		CalCtx.Log("Calibration points are nearly coplanar. Try moving around more?\n");
		return true;
	}

	return false;
}


/**
 * Determines how sensitive the sampled data is to changes in the calibrated rot/trans values.
 */
bool ComputeSensitivity(
	CalibrationContext& CalCtx,
	const std::vector<Sample>& samples,
	const vr::HmdVector3d_t& vrTrans,
	const vr::HmdQuaternion_t& vrRotQuat
) {
	bool reject = false;
	const auto posOffset = DeriveRefToTargetOffset(samples, vrTrans, vrRotQuat);
	char buf[256];

	snprintf(buf, sizeof buf, "HMD to target offset: (%.2f, %.2f, %.2f)\n", posOffset(0), posOffset(1), posOffset(2));
	CalCtx.Log(buf);

	double baseError = RetargetingErrorRMS(samples, posOffset, vrTrans, vrRotQuat);
	snprintf(buf, sizeof buf, "Position error (RMS error): %.2f\n", baseError);
	CalCtx.Log(buf);
	if (baseError > 0.1) reject = true;

	// Compute errors with rotation perturbations

	double deltaError = RetargetingErrorRMS(samples, posOffset, vrTrans, VRRotationQuat(Eigen::Vector3d(10, 0, 0)) * vrRotQuat) - baseError;

	snprintf(buf, sizeof buf, "Sensitivity rotation X (RMS error delta): %.2f\n", deltaError);
	CalCtx.Log(buf);

	deltaError = RetargetingErrorRMS(samples, posOffset, vrTrans, VRRotationQuat(Eigen::Vector3d(0, 10, 0)) * vrRotQuat) - baseError;

	snprintf(buf, sizeof buf, "Sensitivity rotation Y (RMS error delta): %.2f\n", deltaError);
	CalCtx.Log(buf);

	deltaError = RetargetingErrorRMS(samples, posOffset, vrTrans, VRRotationQuat(Eigen::Vector3d(0, 0, 10)) * vrRotQuat) - baseError;

	snprintf(buf, sizeof buf, "Sensitivity rotation Z (RMS error delta): %.2f\n", deltaError);
	CalCtx.Log(buf);

	ComputeIndependence(CalCtx, samples, vrTrans, vrRotQuat);
	return reject; 
}

void StartCalibration()
{
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	ctx.timeLastTick = time;
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		bool ok = true;

		char referenceSerial[256], targetSerial[256];
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.referenceID, vr::Prop_SerialNumber_String, referenceSerial, 256);
		vr::VRSystem()->GetStringTrackedDeviceProperty(ctx.targetID, vr::Prop_SerialNumber_String, targetSerial, 256);

		char buf[256];
		snprintf(buf, sizeof buf, "Reference device ID: %d, serial: %s\n", ctx.referenceID, referenceSerial);
		CalCtx.Log(buf);
		snprintf(buf, sizeof buf, "Target device ID: %d, serial %s\n", ctx.targetID, targetSerial);
		CalCtx.Log(buf);

		if (ctx.referenceID == -1)
		{
			CalCtx.Log("Missing reference device\n"); ok = false;
		}
		else if (!ctx.devicePoses[ctx.referenceID].bPoseIsValid)
		{
			CalCtx.Log("Reference device is not tracking\n"); ok = false;
		}

		if (ctx.targetID == -1)
		{
			CalCtx.Log("Missing target device\n"); ok = false;
		}
		else if (!ctx.devicePoses[ctx.targetID].bPoseIsValid)
		{
			CalCtx.Log("Target device is not tracking\n"); ok = false;
		}

		if (!ok)
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("Aborting calibration!\n");
			return;
		}

		ResetAndDisableOffsets(ctx.targetID);
		ctx.state = CalibrationState::Rotation;
		ctx.wantedUpdateInterval = 0.0;

		CalCtx.Log("Starting calibration...\n");
		return;
	}

	auto sample = CollectSample(ctx);
	if (!sample.valid)
	{
		return;
	}

	static std::vector<Sample> samples;
	samples.push_back(sample);

	CalCtx.Progress(samples.size(), CalCtx.SampleCount());

	if (samples.size() == CalCtx.SampleCount())
	{
		CalCtx.Log("\n");

		auto calibratedRotation = CalibrateRotation(samples);
		
		auto vrRotQuat = VRRotationQuat(calibratedRotation);

		std::vector<Sample> samplesOriginal = samples;

		for (auto &sample : samples) {
			const auto rotMat = quaternionRotateMatrix(vrRotQuat);
			sample.target.rot = rotMat * sample.target.rot;
			sample.target.trans = rotMat * sample.target.trans;
		}

		auto calibratedTranslation = CalibrateTranslation(samples);

		auto vrTrans = VRTranslationVec(calibratedTranslation);

		if (ComputeSensitivity(CalCtx, samplesOriginal, vrTrans, vrRotQuat)) {
			CalCtx.Log("\n\n!!! Rejecting low quality calibration !!!\n");
			ctx.state = CalibrationState::None;
			samples.clear();
			return;
		}

		ctx.calibratedRotation = calibratedRotation;
		ctx.calibratedTranslation = calibratedTranslation;

		protocol::Request req(protocol::RequestSetDeviceTransform);
		req.setDeviceTransform = { ctx.targetID, true, vrTrans, vrRotQuat };
		Driver.SendBlocking(req);

		ctx.validProfile = true;
		SaveProfile(ctx);
		std::ostringstream oss;
		oss << "Final rotation: " << ctx.calibratedRotation << "\n";
		CalCtx.Log(oss.str());
		CalCtx.Log("Finished calibration, profile saved\n");

		ctx.state = CalibrationState::None;

		samples.clear();
	}
}

void LoadChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
