#pragma once

#include <deque>
#include <utility>
#include <Eigen/Dense>

namespace Metrics {
	extern double TimeSpan, CurrentTime;

	double timestamp();
	void RecordTimestamp();

	template<typename T>
	class TimeSeries {
		std::deque<std::pair<double, T>> Data;
		
	public:
		const std::deque<std::pair<double, T>> &data() const { return Data; }

		void Push(const T& data) {
			Data.push_back(std::make_pair(CurrentTime, data));

			double cutoff = CurrentTime - TimeSpan;
			while (!Data.empty() && (Data.front().first < cutoff || Data.size() > INT_MAX)) {
				Data.pop_front();
			}
		}

		int size() const { return (int)Data.size(); }
		const std::pair<double, T>& operator[](int index) const { return Data[index]; }

		const T& last() const {
			static const T fallback;
			return Data.size() > 0 ? Data.back().second : fallback;
		}

		const double lastTs() const {
			return Data.size() > 0 ? Data.back().first : 0;
		}
	};


	extern TimeSeries<Eigen::Vector3d> posOffset_rawComputed; // , rotOffset_rawComputed;
	extern TimeSeries<Eigen::Vector3d> posOffset_currentCal; // , rotOffset_currentCal;
	extern TimeSeries<Eigen::Vector3d> posOffset_lastSample; // , rotOffset_lastSample;
	extern TimeSeries<Eigen::Vector3d> posOffset_byRelPose;
	
	extern TimeSeries<double> error_rawComputed, error_currentCal, error_byRelPose, error_currentCalRelPose;
	extern TimeSeries<double> axisIndependence;
	extern TimeSeries<double> computationTime;

	extern TimeSeries<bool> calibrationApplied;

	extern bool enableLogs;

	void WriteLogAnnotation(const char* s);
	void WriteLogEntry();
}