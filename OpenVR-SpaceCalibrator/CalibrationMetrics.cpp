#include "stdafx.h"
#include "CalibrationMetrics.h"
#include <shlobj_core.h>
#include <fstream>
#include <vector>

namespace Metrics {
	double TimeSpan = 30, CurrentTime;

	TimeSeries<Eigen::Vector3d> posOffset_rawComputed; // , rotOffset_rawComputed;
	TimeSeries<Eigen::Vector3d> posOffset_currentCal; // , rotOffset_currentCal;
	TimeSeries<Eigen::Vector3d> posOffset_lastSample; // , rotOffset_lastSample;
	TimeSeries<Eigen::Vector3d> posOffset_byRelPose;

	TimeSeries<double> error_rawComputed, error_currentCal, error_byRelPose, error_currentCalRelPose;
	TimeSeries<double> axisIndependence;
	TimeSeries<double> computationTime;

	// true - full calibration, false - static calibration
	TimeSeries<bool> calibrationApplied;

	double timestamp() {
		static long long ts_start = ~0LL;
		
		LARGE_INTEGER ts, freq;
		QueryPerformanceCounter(&ts);
		QueryPerformanceFrequency(&freq);

		if (ts_start == ~0LL) ts_start = ts.QuadPart;

		ts.QuadPart -= ts_start;

		return ts.QuadPart / (double)freq.QuadPart;
	}

	void RecordTimestamp() {
		CurrentTime = timestamp();
	}

	bool enableLogs = false;

	static std::ofstream logFile;
	static bool logFileIsOpen = false;
	static bool failedToOpenLogFile = false;

	struct CsvField {
		const char* name;
		void (*writer)(std::ofstream& s);
	};

#define TS_FIELD(n) \
	{ #n, [](auto &s) { s << n.last(); } }
	
#define TS_VECTOR_FIELD(n) \
	{ #n ".x", [](auto &s) { s << n.last()(0); } }, \
	{ #n ".y", [](auto &s) { s << n.last()(1); } }, \
	{ #n ".z", [](auto &s) { s << n.last()(2); } }

	static const CsvField fields[] = {
		{
			"Timestamp",
			[](auto& s) { s << CurrentTime; }
		},

		TS_VECTOR_FIELD(posOffset_rawComputed),
		TS_VECTOR_FIELD(posOffset_currentCal),
		TS_VECTOR_FIELD(posOffset_lastSample),
		TS_VECTOR_FIELD(posOffset_byRelPose),
		
		TS_FIELD(error_rawComputed),
		TS_FIELD(error_currentCal),
		TS_FIELD(error_byRelPose),
		TS_FIELD(error_currentCalRelPose),
		TS_FIELD(axisIndependence),
		TS_FIELD(computationTime),

		{
			"calibrationApplied", 
			[](auto& s) {
				if (calibrationApplied.lastTs() == CurrentTime) {
					if (calibrationApplied.last()) {
						s << "FULL";
					}
					else {
						s << "STATIC";
					}
				}
			}
		}
	};
	
	
	static void ClearOldLogs(const std::wstring& path) {
		std::wstring search_path = path + L"\\spacecal_log.*.txt";
		WIN32_FIND_DATA find_data;

		SYSTEMTIME st_now;
		FILETIME ft_now;
		GetSystemTime(&st_now);
		SystemTimeToFileTime(&st_now, &ft_now);

		ULARGE_INTEGER ft_tmp;
		ft_tmp.HighPart = ft_now.dwHighDateTime;
		ft_tmp.LowPart = ft_now.dwLowDateTime;
		// one day ago
		uint64_t limit = ft_tmp.QuadPart - (uint64_t)(24LL * 3600LL * 10LL * 1000LL * 1000LL);

		HANDLE find_handle = FindFirstFile(search_path.c_str(), &find_data);
		if (find_handle != INVALID_HANDLE_VALUE) {
			do {
				ft_tmp.HighPart = find_data.ftLastWriteTime.dwHighDateTime;
				ft_tmp.LowPart = find_data.ftLastWriteTime.dwLowDateTime;
				if (ft_tmp.QuadPart < limit) {
					std::wstring file_path = path + L"\\" + find_data.cFileName;
					DeleteFile(file_path.c_str());
				}
			} while (FindNextFile(find_handle, &find_data));
			FindClose(find_handle);
		}
	}

	static bool OpenLogFile() {
		PWSTR RootPath = NULL;
		if (S_OK != SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, NULL, &RootPath)) {
			CoTaskMemFree(RootPath);
			return false;
		}

		std::wstring path(RootPath);
		CoTaskMemFree(RootPath);
		
		path += LR"(\OpenVR-SpaceCalibrator)";
		if (CreateDirectoryW(path.c_str(), 0) == 0 && GetLastError() != ERROR_ALREADY_EXISTS) {
			return false;
		}

		path += LR"(\Logs)";
		if (CreateDirectoryW(path.c_str(), 0) == 0 && GetLastError() != ERROR_ALREADY_EXISTS) {
			return false;
		}

		ClearOldLogs(path);

		SYSTEMTIME now;
		GetSystemTime(&now);

		size_t dateBufLen = GetDateFormatW(LOCALE_USER_DEFAULT, 0, &now, L"yyyy-MM-dd", NULL, 0);
		std::vector<WCHAR> dateBuf(dateBufLen);
		if (!GetDateFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"yyyy-MM-dd", &dateBuf[0], dateBufLen, NULL)) return false;
		
		size_t timeBufLen = GetTimeFormatW(LOCALE_USER_DEFAULT, 0, &now, L"HH-mm-ss", NULL, 0);
		std::vector<WCHAR> timeBuf(timeBufLen);
		if (!GetTimeFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"HH-mm-ss", &timeBuf[0], timeBufLen)) return false;

		path += LR"(\spacecal_log.)";
		path += &dateBuf[0];
		path += L"T";
		path += &timeBuf[0];
		path += L".txt";

		logFile.open(path);
		if (logFile.fail()) {
			return false;
		}

		for (int i = 0; i < sizeof fields / sizeof fields[0]; i++) {
			if (i > 0) logFile << ",";
			logFile << fields[i].name;
		}
		logFile << "\n";

		logFileIsOpen = true;

		return true;
	}
	
	static bool CheckLogOpen() {
		if (!enableLogs) {
			if (logFileIsOpen) {
				logFile.close();
			}
			logFileIsOpen = false;
			failedToOpenLogFile = false;

			return false;
		}

		if (failedToOpenLogFile) return false;
		if (!logFileIsOpen && !OpenLogFile()) {
			failedToOpenLogFile = true;
			return false;
		}
		return true;
	}

	void WriteLogAnnotation(const char *s) {
		if (!CheckLogOpen()) return;

		logFile << "# [" << timestamp() << "] " << s << "\n";
		logFile.flush();
	}

	void WriteLogEntry() {
		if (!CheckLogOpen()) return;

		if (logFileIsOpen) {
			for (int i = 0; i < sizeof fields / sizeof fields[0]; i++) {
				if (i > 0) logFile << ",";
				fields[i].writer(logFile);
			}
			logFile << "\n";
		}
		logFile.flush();
	}

	// --- Pose telemetry log ---

	static std::ofstream poseLogFile;
	static bool poseLogFileIsOpen = false;
	static bool poseLogFailedToOpen = false;

	static bool OpenPoseLogFile()
	{
		PWSTR RootPath = NULL;
		if (S_OK != SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, NULL, &RootPath)) {
			CoTaskMemFree(RootPath);
			return false;
		}

		std::wstring path(RootPath);
		CoTaskMemFree(RootPath);

		path += LR"(\OpenVR-SpaceCalibrator)";
		if (CreateDirectoryW(path.c_str(), 0) == 0 && GetLastError() != ERROR_ALREADY_EXISTS)
			return false;

		path += LR"(\Logs)";
		if (CreateDirectoryW(path.c_str(), 0) == 0 && GetLastError() != ERROR_ALREADY_EXISTS)
			return false;

		SYSTEMTIME now;
		GetSystemTime(&now);

		size_t dateBufLen = GetDateFormatW(LOCALE_USER_DEFAULT, 0, &now, L"yyyy-MM-dd", NULL, 0);
		std::vector<WCHAR> dateBuf(dateBufLen);
		if (!GetDateFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"yyyy-MM-dd", &dateBuf[0], dateBufLen, NULL)) return false;

		size_t timeBufLen = GetTimeFormatW(LOCALE_USER_DEFAULT, 0, &now, L"HH-mm-ss", NULL, 0);
		std::vector<WCHAR> timeBuf(timeBufLen);
		if (!GetTimeFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"HH-mm-ss", &timeBuf[0], timeBufLen)) return false;

		path += LR"(\pose_log.)";
		path += &dateBuf[0];
		path += L"T";
		path += &timeBuf[0];
		path += L".csv";

		poseLogFile.open(path);
		if (poseLogFile.fail())
			return false;

		poseLogFile <<
			"Timestamp,"
			"RefSampleTime,TargetSampleTime,TimeDelta,"
			"Ref_PoseTimeOffset,Target_PoseTimeOffset,"
			"Accepted,"
			"Ref_Before_Pos_X,Ref_Before_Pos_Y,Ref_Before_Pos_Z,"
			"Ref_Before_Rot_W,Ref_Before_Rot_X,Ref_Before_Rot_Y,Ref_Before_Rot_Z,"
			"Ref_Before_Vel_X,Ref_Before_Vel_Y,Ref_Before_Vel_Z,"
			"Ref_Before_AngVel_X,Ref_Before_AngVel_Y,Ref_Before_AngVel_Z,"
			"Target_Before_Pos_X,Target_Before_Pos_Y,Target_Before_Pos_Z,"
			"Target_Before_Rot_W,Target_Before_Rot_X,Target_Before_Rot_Y,Target_Before_Rot_Z,"
			"Target_Before_Vel_X,Target_Before_Vel_Y,Target_Before_Vel_Z,"
			"Target_Before_AngVel_X,Target_Before_AngVel_Y,Target_Before_AngVel_Z,"
			"Ref_After_Pos_X,Ref_After_Pos_Y,Ref_After_Pos_Z,"
			"Ref_After_Rot_W,Ref_After_Rot_X,Ref_After_Rot_Y,Ref_After_Rot_Z,"
			"Target_After_Pos_X,Target_After_Pos_Y,Target_After_Pos_Z,"
			"Target_After_Rot_W,Target_After_Rot_X,Target_After_Rot_Y,Target_After_Rot_Z\n";

		poseLogFileIsOpen = true;
		return true;
	}

	static bool CheckPoseLogOpen()
	{
		if (!enableLogs) {
			if (poseLogFileIsOpen)
				poseLogFile.close();
			poseLogFileIsOpen = false;
			poseLogFailedToOpen = false;
			return false;
		}
		if (poseLogFailedToOpen) return false;
		if (!poseLogFileIsOpen && !OpenPoseLogFile()) {
			poseLogFailedToOpen = true;
			return false;
		}
		return true;
	}

	void WritePoseLogEntry(
		long long refSampleTime,
		long long targetSampleTime,
		double delta,
		const vr::DriverPose_t &refBefore,
		const vr::DriverPose_t &targetBefore,
		const vr::DriverPose_t &refAfter,
		const vr::DriverPose_t &targetAfter,
		bool accepted)
	{
		if (!CheckPoseLogOpen()) return;

		poseLogFile
			<< timestamp() << ","
			<< refSampleTime << "," << targetSampleTime << "," << delta << ","
			<< refBefore.poseTimeOffset << "," << targetBefore.poseTimeOffset << ","
			<< (accepted ? 1 : 0) << ","
			// ref before: position, rotation, velocity, angular velocity
			<< refBefore.vecPosition[0] << "," << refBefore.vecPosition[1] << "," << refBefore.vecPosition[2] << ","
			<< refBefore.qRotation.w << "," << refBefore.qRotation.x << "," << refBefore.qRotation.y << "," << refBefore.qRotation.z << ","
			<< refBefore.vecVelocity[0] << "," << refBefore.vecVelocity[1] << "," << refBefore.vecVelocity[2] << ","
			<< refBefore.vecAngularVelocity[0] << "," << refBefore.vecAngularVelocity[1] << "," << refBefore.vecAngularVelocity[2] << ","
			// target before: position, rotation, velocity, angular velocity
			<< targetBefore.vecPosition[0] << "," << targetBefore.vecPosition[1] << "," << targetBefore.vecPosition[2] << ","
			<< targetBefore.qRotation.w << "," << targetBefore.qRotation.x << "," << targetBefore.qRotation.y << "," << targetBefore.qRotation.z << ","
			<< targetBefore.vecVelocity[0] << "," << targetBefore.vecVelocity[1] << "," << targetBefore.vecVelocity[2] << ","
			<< targetBefore.vecAngularVelocity[0] << "," << targetBefore.vecAngularVelocity[1] << "," << targetBefore.vecAngularVelocity[2] << ","
			// ref after: position, rotation
			<< refAfter.vecPosition[0] << "," << refAfter.vecPosition[1] << "," << refAfter.vecPosition[2] << ","
			<< refAfter.qRotation.w << "," << refAfter.qRotation.x << "," << refAfter.qRotation.y << "," << refAfter.qRotation.z << ","
			// target after: position, rotation
			<< targetAfter.vecPosition[0] << "," << targetAfter.vecPosition[1] << "," << targetAfter.vecPosition[2] << ","
			<< targetAfter.qRotation.w << "," << targetAfter.qRotation.x << "," << targetAfter.qRotation.y << "," << targetAfter.qRotation.z << "\n";

		poseLogFile.flush();
	}
}
