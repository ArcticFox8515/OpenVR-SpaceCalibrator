#include "stdafx.h"
#include "CalibrationMetrics.h"

namespace Metrics {
	double TimeSpan = 30, CurrentTime;

	TimeSeries<Eigen::Vector3d> posOffset_rawComputed; // , rotOffset_rawComputed;
	TimeSeries<Eigen::Vector3d> posOffset_currentCal; // , rotOffset_currentCal;
	TimeSeries<Eigen::Vector3d> posOffset_lastSample; // , rotOffset_lastSample;

	TimeSeries<double> error_rawComputed, error_currentCal;
	TimeSeries<double> axisIndependence;

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
}
