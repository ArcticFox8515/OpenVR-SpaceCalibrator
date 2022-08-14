#include "stdafx.h"

#include <vector>

#include <implot/implot.h>
#include "CalibrationCalc.h"
#include "UserInterface.h"

namespace {
	const int LOG_LENGTH_MS = 300 * 1000;
	
	struct CalibrationStep {
		double timestamp;

		Eigen::Vector3d posOffset;
		double m_newCalRMS, m_oldCalRMS, m_axisVariance;
	};


	long long ts_start = ~0LL;
	double TimeSpan = 30;

	struct Buffer {
		std::vector<ImVec2> Data;

		int firstValid = 0;

		void Push(double t, double d) {
			Data.push_back(ImVec2(t, d));

			while (firstValid < Data.size() && Data[firstValid].x < t - TimeSpan) {
				firstValid++;
			}

			if (firstValid > Data.size() / 2) {
				Data.erase(Data.begin(), Data.begin() + firstValid);
				firstValid = 0;
			}
		}
	};

	std::vector<double> calibrationAppliedTime;

	Buffer pos_offset_x, pos_offset_y, pos_offset_z, newCalRMS, oldCalRMS, axisVariance;

	long lastCycle = 0;
	double refTime;

	double timestamp() {
		LARGE_INTEGER ts, freq;
		QueryPerformanceCounter(&ts);
		QueryPerformanceFrequency(&freq);

		if (ts_start == ~0LL) ts_start = ts.QuadPart;

		ts.QuadPart -= ts_start;

		return ts.QuadPart / (double)freq.QuadPart;
	}

	ImPlotPoint BufferGetter(void* data, int index) {
		Buffer* buf = (Buffer*)data;
		return ImPlotPoint(buf->Data[index].x - refTime, buf->Data[index].y);
	}

	ImPlotPoint FetchBelowThreshold(void* data, int index) {
		auto point = BufferGetter(data, index);

		if (point.y > CalibrationCalc::AxisVarianceThreshold)
			point.y = CalibrationCalc::AxisVarianceThreshold;

		return point;
	}

	ImPlotPoint FetchAboveThreshold(void* data, int index) {
		auto point = BufferGetter(data, index);

		if (point.y < CalibrationCalc::AxisVarianceThreshold)
			point.y = CalibrationCalc::AxisVarianceThreshold;

		return point;
	}

	ImPlotPoint VarianceRef(void* data, int index) {
		auto point = BufferGetter(data, index);
		point.y = CalibrationCalc::AxisVarianceThreshold;
			
		return point;
	}

	ImPlotPoint ZeroRef(void* data, int index) {
		auto point = BufferGetter(data, index);
		point.y = 0;

		return point;
	}

	double lastMouseX = -INFINITY;
	bool wasHovered;

	std::vector<double> calAppliedTimeBuffer;

	void PrepApplyTicks() {
		calAppliedTimeBuffer.clear();
		calAppliedTimeBuffer.reserve(calibrationAppliedTime.size());

		for (auto t : calibrationAppliedTime) {
			calAppliedTimeBuffer.push_back(t - refTime);
		}
	}

	void AddApplyTicks() {
		ImPlot::PlotVLines("##CalibrationAppliedTime", &calAppliedTimeBuffer[0], calAppliedTimeBuffer.size());

		ImPlot::SetNextLineStyle(ImVec4(0.5, 0.5, 1, 1));
		ImPlot::PlotVLines("##TagLine", &lastMouseX, 1);

		if (ImPlot::IsPlotHovered()) {
			auto mousePos = ImPlot::GetPlotMousePos();
			lastMouseX = mousePos.x;
			wasHovered = true;
		}
	}
}

void PushCalibrationDebugData(const class CalibrationCalc& calc) {
	if (calc.m_calcCycle == lastCycle) return;
	lastCycle = calc.m_calcCycle;

	double t = timestamp();

	pos_offset_x.Push(t, calc.m_posOffset(0) * 1000);
	pos_offset_y.Push(t, calc.m_posOffset(1) * 1000);
	pos_offset_z.Push(t, calc.m_posOffset(2) * 1000);

	newCalRMS.Push(t, calc.m_newCalRMS * 1000);
	oldCalRMS.Push(t, calc.m_oldCalRMS * 1000);
	axisVariance.Push(t, calc.m_axisVariance);
}

void PushCalibrationApplyTime() {
	double t = timestamp();
	calibrationAppliedTime.push_back(t);

	double cutoff = t - TimeSpan;

	if (calibrationAppliedTime[0] < t - (TimeSpan * 2)) {
		auto it = calibrationAppliedTime.begin();
		
		while (it != calibrationAppliedTime.end() && *it < cutoff) {
			it++;
		}
		it--;
		calibrationAppliedTime.erase(calibrationAppliedTime.begin(), it);
	}
}



void ShowCalibrationDebug() {
	//ImGui::ShowDemoWindow();
	//		ImPlot::ShowDemoWindow();

	double initMouseX = lastMouseX;
	wasHovered = false;

	if (pos_offset_x.Data.empty()) return;

	auto avail = ImGui::GetContentRegionAvail();

	if (!ImPlot::BeginSubplots("##CalibrationDebug", 1, 3, avail,
		ImPlotSubplotFlags_LinkAllX | ImPlotSubplotFlags_NoResize
	)) return;

	static bool firstrun = true;
	static ImPlotColormap axisVarianceColormap;
	if (firstrun) {
		firstrun = false;

		auto defaultFirst = ImPlot::GetColormapColor(0);

		ImVec4 colors[] = {
			defaultFirst,
			{ 1, 0, 0, 1 },
			{ 0, 1, 0, 1 },
			{ 0.5, 0.5, 0.5, 1 },
		};

		axisVarianceColormap = ImPlot::AddColormap("AxisVarianceColormap", colors, sizeof(colors)/sizeof(colors[0]));
	}

	double t = refTime = timestamp();
	PrepApplyTicks();

	//ImGui::TableNextRow();
	// Axis offset
	//ImGui::TableSetColumnIndex(0);
	if (ImPlot::BeginPlot("posOffset")) {
		ImPlot::SetupAxes(NULL, "mm", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
		ImPlot::SetupAxisLimits(ImAxis_X1, -TimeSpan, 0, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, -200, 200, ImGuiCond_Appearing);

		AddApplyTicks();

		ImPlot::PlotLineG("X", BufferGetter, &pos_offset_x, pos_offset_x.Data.size());
		ImPlot::PlotLineG("Y", BufferGetter, &pos_offset_y, pos_offset_y.Data.size());
		ImPlot::PlotLineG("Z", BufferGetter, &pos_offset_z, pos_offset_z.Data.size());

		ImPlot::EndPlot();
	}

	ImGui::TableSetColumnIndex(1);
	if (ImPlot::BeginPlot("Position error")) {
		ImPlot::SetupAxes(NULL, "mm (RMS)");
		ImPlot::SetupAxisLimits(ImAxis_X1, -TimeSpan, 0, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 25, ImGuiCond_Appearing);

		AddApplyTicks();

		ImPlot::PlotLineG("Candidate", BufferGetter, &newCalRMS, newCalRMS.Data.size());
		ImPlot::PlotLineG("Active", BufferGetter, &oldCalRMS, oldCalRMS.Data.size());
		ImPlot::EndPlot();
	}
	
	ImGui::TableSetColumnIndex(2);
	if (ImPlot::BeginPlot("Axis variance", ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
		ImPlot::SetupAxes(NULL, NULL, 0, 0);
		ImPlot::SetupAxisLimits(ImAxis_X1, -TimeSpan, 0, ImGuiCond_Always);
		ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.003, ImGuiCond_Always);

		AddApplyTicks();

		ImPlot::PushColormap(axisVarianceColormap);
		ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.5f);
		ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 1));
		ImPlot::PlotShadedG("VarianceLow", FetchBelowThreshold, &axisVariance, ZeroRef, &axisVariance, axisVariance.Data.size());
		ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 1));
		ImPlot::PlotShadedG("VarianceHigh", FetchAboveThreshold, &axisVariance, VarianceRef, &axisVariance, axisVariance.Data.size());
		
		ImPlot::PlotLineG("Datapoint", BufferGetter, &axisVariance, axisVariance.Data.size());
		
		ImPlot::PopStyleVar(1);
		ImPlot::PopColormap(1);

		ImPlot::EndPlot();
	}
	
	ImPlot::EndSubplots();

	if (!wasHovered) {
		lastMouseX = -INFINITY;
	}

	if (lastMouseX != initMouseX) {
		RequestImmediateRedraw();
	}
}