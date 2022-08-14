#include "stdafx.h"
#include "VRState.h"

VRState VRState::Load()
{
	VRState state;
	auto& trackingSystems = state.trackingSystems;

	char buffer[vr::k_unMaxPropertyStringSize];

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		if (deviceClass != vr::TrackedDeviceClass_TrackingReference)
		{
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

			if (err == vr::TrackedProp_Success)
			{
				std::string system(buffer);
				auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
				if (existing != trackingSystems.end())
				{
					if (deviceClass == vr::TrackedDeviceClass_HMD)
					{
						trackingSystems.erase(existing);
						trackingSystems.insert(trackingSystems.begin(), system);
					}
				}
				else
				{
					trackingSystems.push_back(system);
				}

				VRDevice device;
				device.id = id;
				device.deviceClass = deviceClass;
				device.trackingSystem = system;

				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
				device.model = std::string(buffer);

				vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, vr::k_unMaxPropertyStringSize, &err);
				device.serial = std::string(buffer);

				device.controllerRole = (vr::ETrackedControllerRole)vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &err);

				state.devices.push_back(device);
			}
			else
			{
				printf("failed to get tracking system name for id %d\n", id);
			}
		}
	}

	return state;
}

int VRState::FindDevice(const std::string& trackingSystem, const std::string& model, const std::string& serial) const {
	for (int i = 0; i < devices.size(); i++) {
		const auto& device = devices[i];
		
		if (device.trackingSystem == trackingSystem && device.model == model && device.serial == serial) return device.id;
	}

	return -1;
}