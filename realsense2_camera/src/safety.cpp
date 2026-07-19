// Copyright 2024 RealSense, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/base_realsense_node.h"
#include <safety_table_params.h>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace realsense2_camera;
using namespace rs2;

void BaseRealSenseNode::setSafetySensorIfAvailable()
{
    // Find if the Safety Sensor is available.
    auto iter = std::find_if(_dev_sensors.begin(), _dev_sensors.end(),
                             [](rs2::sensor sensor)
                             { return sensor.is<rs2::safety_sensor>(); });
    if (iter != _dev_sensors.end())
    {
        _safety_sensor = &(*iter);
    }
}

void BaseRealSenseNode::publishSafetyServices()
{
    _safety_preset_read_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyPresetRead>(
        "~/safety_preset_read",
        [&](const realsense2_camera_msgs::srv::SafetyPresetRead::Request::SharedPtr req,
            realsense2_camera_msgs::srv::SafetyPresetRead::Response::SharedPtr res)
        { SafetyPresetReadService(req, res); });

    _safety_preset_write_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyPresetWrite>(
        "~/safety_preset_write",
        [&](const realsense2_camera_msgs::srv::SafetyPresetWrite::Request::SharedPtr req,
            realsense2_camera_msgs::srv::SafetyPresetWrite::Response::SharedPtr res)
        { SafetyPresetWriteService(req, res); });

    _safety_interface_config_read_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyInterfaceConfigRead>(
        "~/safety_interface_config_read",
        [&](const realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Request::SharedPtr req,
            realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Response::SharedPtr res)
        { SafetyInterfaceConfigReadService(req, res); });

    _safety_interface_config_write_srv = _node.create_service<realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite>(
        "~/safety_interface_config_write",
        [&](const realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Request::SharedPtr req,
            realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Response::SharedPtr res)
        { SafetyInterfaceConfigWriteService(req, res); });

    _application_config_read_srv = _node.create_service<realsense2_camera_msgs::srv::ApplicationConfigRead>(
        "~/application_config_read",
        [&](const realsense2_camera_msgs::srv::ApplicationConfigRead::Request::SharedPtr req,
            realsense2_camera_msgs::srv::ApplicationConfigRead::Response::SharedPtr res)
        { ApplicationConfigReadService(req, res); });

    _application_config_write_srv = _node.create_service<realsense2_camera_msgs::srv::ApplicationConfigWrite>(
        "~/application_config_write",
        [&](const realsense2_camera_msgs::srv::ApplicationConfigWrite::Request::SharedPtr req,
            realsense2_camera_msgs::srv::ApplicationConfigWrite::Response::SharedPtr res)
        { ApplicationConfigWriteService(req, res); });

    _hardware_monitor_command_send_srv = _node.create_service<realsense2_camera_msgs::srv::HardwareMonitorCommandSend>(
        "~/hardware_monitor_command_send",
        [&](const realsense2_camera_msgs::srv::HardwareMonitorCommandSend::Request::SharedPtr req,
            realsense2_camera_msgs::srv::HardwareMonitorCommandSend::Response::SharedPtr res)
        { HardwareMonitorCommandSendService(req, res); });
}

void BaseRealSenseNode::SafetyPresetReadService(const realsense2_camera_msgs::srv::SafetyPresetRead::Request::SharedPtr req,
                                                realsense2_camera_msgs::srv::SafetyPresetRead::Response::SharedPtr res)
{
    try
    {
        res->safety_preset = _safety_sensor->as<rs2::safety_sensor>().get_safety_preset(req->index);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::SafetyPresetWriteService(const realsense2_camera_msgs::srv::SafetyPresetWrite::Request::SharedPtr req,
                                                 realsense2_camera_msgs::srv::SafetyPresetWrite::Response::SharedPtr res)
{
    try
    {
        _safety_sensor->as<rs2::safety_sensor>().set_safety_preset(req->index, req->safety_preset);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::SafetyInterfaceConfigReadService(const realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Request::SharedPtr req,
                                                         realsense2_camera_msgs::srv::SafetyInterfaceConfigRead::Response::SharedPtr res)
{
    try
    {
        rs2_calib_location location = static_cast<rs2_calib_location>(req->calib_location);
        res->safety_interface_config = _safety_sensor->as<rs2::safety_sensor>().get_safety_interface_config(location);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::SafetyInterfaceConfigWriteService(const realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Request::SharedPtr req,
                                                          realsense2_camera_msgs::srv::SafetyInterfaceConfigWrite::Response::SharedPtr res)
{
    try
    {
        _safety_sensor->as<rs2::safety_sensor>().set_safety_interface_config(req->safety_interface_config);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::ApplicationConfigReadService(const realsense2_camera_msgs::srv::ApplicationConfigRead::Request::SharedPtr req,
                                                     realsense2_camera_msgs::srv::ApplicationConfigRead::Response::SharedPtr res)
{
    try
    {
        (void)req; // silence unused parameter warning
        res->application_config = _safety_sensor->as<rs2::safety_sensor>().get_application_config();
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::ApplicationConfigWriteService(const realsense2_camera_msgs::srv::ApplicationConfigWrite::Request::SharedPtr req,
                                                      realsense2_camera_msgs::srv::ApplicationConfigWrite::Response::SharedPtr res)
{
    try
    {
        _safety_sensor->as<rs2::safety_sensor>().set_application_config(req->application_config);
        res->success = true;
    }
    catch (const std::exception &e)
    {
        res->success = false;
        res->error_message = std::string("Exception occurred: ") + e.what();
    }
}

void BaseRealSenseNode::HardwareMonitorCommandSendService(const realsense2_camera_msgs::srv::HardwareMonitorCommandSend::Request::SharedPtr req,
                                                          realsense2_camera_msgs::srv::HardwareMonitorCommandSend::Response::SharedPtr res)
{
    try
    {
        auto dp = _dev.as<debug_protocol>();

        // Build a debug protocol command and send it
        std::vector<uint8_t> cmd_to_send = dp.build_command(req->opcode, req->param1, req->param2, req->param3, req->param4, req->data);
        std::vector<uint8_t> result = dp.send_and_receive_raw_data(cmd_to_send);

        unsigned returned_opcode = *result.data();

        // check returned opcode
        if (req->opcode != returned_opcode)
        {
            // Failure
            res->success = false;
            res->result.clear();
            std::stringstream error_message;
            error_message << "opcodes do not match! Sent 0x" << std::hex << req->opcode << " but received 0x" << std::hex << (returned_opcode) << "!";
            res->error_message = error_message.str();
        }
        else
        {
            // Success
            res->success = true;
            res->result = result;
            res->error_message = "";
        }
    }
    catch (const rs2::error &e)
    {
        // Handle exceptions and set the failure response
        res->success = false;
        res->result.clear();
        res->error_message = std::string("Error sending hardware monitor command: ") + e.what();
    }
}

// Seconds to let the safety firmware finish booting after a provisioning
// hardware reset. The device re-enumerates a few seconds before its safety
// module is ready, and streams started in that window fail on internal
// safety-config reads. A readiness poll is not reliable here - table reads
// already succeed while the failing subsystem is still booting - so this
// mirrors the fixed 10 s wait the provisioning tooling uses after a reset.
// A more complete fix would be retrying the sensor start in the driver
// itself; this is the simpler approach for now.
constexpr int SAFETY_FW_SETTLE_SEC = 8;

bool BaseRealSenseNode::applySafetyTableParams()
{
    const bool height_set = (_safety_mount_height >= 0.0);
    const bool cell_size_set = (_safety_occupancy_cell_size >= 0.0);
    if (!height_set && !cell_size_set)
        return false;
    if (!_safety_allow_table_write)
    {
        ROS_WARN("safety_camera.mount_height / occupancy_cell_size set but"
                 " safety_camera.allow_table_write is false - device flash left untouched");
        return false;
    }
    // Reject configuration mistakes (typically a millimeters/meters mix-up)
    // before any device I/O: firmware may accept an absurd value, and the
    // matching read-back would then report the bad provisioning as success.
    if (height_set && !safety_table_params::mountHeightInRange(_safety_mount_height))
    {
        ROS_ERROR_STREAM("safety_camera.mount_height " << _safety_mount_height
                         << " is outside [" << safety_table_params::MOUNT_HEIGHT_MIN_M
                         << ", " << safety_table_params::MOUNT_HEIGHT_MAX_M
                         << "] m - no safety table will be written");
        return false;
    }
    if (cell_size_set && !safety_table_params::cellSizeInRange(_safety_occupancy_cell_size))
    {
        ROS_ERROR_STREAM("safety_camera.occupancy_cell_size " << _safety_occupancy_cell_size
                         << " is outside [" << safety_table_params::CELL_SIZE_MIN_M
                         << ", " << safety_table_params::CELL_SIZE_MAX_M
                         << "] m - no safety table will be written");
        return false;
    }
    if (!_safety_sensor)
    {
        ROS_WARN("safety_camera.mount_height / occupancy_cell_size set but no safety sensor found - ignoring");
        return false;
    }

    // The hardware reset below destroys and re-creates this node, so the
    // retry guard must outlive it and must be keyed per device serial: a
    // composed process can host several safety cameras, and a plain static
    // counter would let two cameras drain each other's retry budget. Two
    // cycles per serial allow the normal write -> reset -> verify sequence
    // and nothing more. The cycle is consumed BEFORE the write attempt on
    // purpose - it bounds hardware resets even when the write itself fails
    // partway through, not just when it fully succeeds.
    static std::mutex s_write_cycles_mutex;
    static std::map<std::string, int> s_write_cycles; // per-serial, survives node re-creation

    try
    {
        auto safety = _safety_sensor->as<rs2::safety_sensor>();
        const std::string serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        // A device this process already provisioned and reset needs time to
        // finish booting before the tables are touched again and the streams
        // start (see SAFETY_FW_SETTLE_SEC above).
        bool provisioned_earlier = false;
        {
            std::lock_guard<std::mutex> lock(s_write_cycles_mutex);
            const auto it = s_write_cycles.find(serial_no);
            provisioned_earlier = (it != s_write_cycles.end() && it->second > 0);
        }
        if (provisioned_earlier)
        {
            ROS_INFO("Waiting for the safety firmware to settle after the provisioning reset");
            std::this_thread::sleep_for(std::chrono::seconds(SAFETY_FW_SETTLE_SEC));
        }

        std::string sic = safety.get_safety_interface_config(RS2_CALIB_LOCATION_FLASH);
        const bool sic_changed = safety_table_params::patchSafetyInterfaceConfig(
            sic, _safety_mount_height, _safety_occupancy_cell_size);

        std::string calib;
        std::string preset0;
        bool calib_changed = false;
        bool presets_changed = false;
        if (height_set)
        {
            calib = _dev.as<rs2::auto_calibrated_device>().get_calibration_config();
            calib_changed = safety_table_params::patchCalibrationConfig(calib, _safety_mount_height);
            // Presets are written as a batch, so preset 0 stands in for all 64.
            preset0 = safety.get_safety_preset(0);
            presets_changed = safety_table_params::patchSafetyPreset(preset0, _safety_mount_height);
        }

        if (!sic_changed && !calib_changed && !presets_changed)
        {
            // Provisioning converged (or was never needed) - clear the cycle
            // budget so later reconnects of this device neither wait for a
            // settle nor inherit a drained retry budget.
            std::lock_guard<std::mutex> lock(s_write_cycles_mutex);
            s_write_cycles[serial_no] = 0;
            return false; // flash already matches the requested values
        }

        std::vector<std::string> presets;
        std::vector<bool> preset_write_needed;
        if (presets_changed)
        {
            // Read and patch the WHOLE preset bank before consuming a write
            // cycle or entering service mode: patchAllSafetyPresets is
            // all-or-nothing, so a structurally invalid preset anywhere in
            // the bank aborts here - before any write could leave the flash
            // presets split across two mount heights. This dry run performs
            // no writes, which is why it sits before the budget check.
            presets.resize(64);
            for (int i = 0; i < 64; ++i)
                presets[i] = safety.get_safety_preset(i);
            preset_write_needed =
                safety_table_params::patchAllSafetyPresets(presets, _safety_mount_height);
        }

        {
            std::lock_guard<std::mutex> lock(s_write_cycles_mutex);
            int& cycles = s_write_cycles[serial_no];
            if (cycles >= 2)
            {
                ROS_ERROR("Safety tables still differ from the requested parameters after writing."
                          " Not retrying - check units and firmware acceptance.");
                return false;
            }
            ++cycles;
        }

        safety.set_option(RS2_OPTION_SAFETY_MODE, static_cast<float>(RS2_SAFETY_MODE_SERVICE));
        try
        {
            if (sic_changed)
            {
                // The interface config only takes effect after a reset, and the
                // remaining tables are written on the post-reset pass.
                safety.set_safety_interface_config(sic);
                ROS_WARN("Safety interface config updated - resetting device to apply");
                hardwareResetRequest();
                return true;
            }
            if (presets_changed)
            {
                // Write preset 0 (the change sentinel) LAST. If the batch fails
                // partway through presets 1..63, preset 0 stays stale, so the
                // next launch's read-back of preset 0 still shows a mismatch
                // and the whole batch is retried instead of being skipped.
                for (int i = 1; i < 64; ++i)
                {
                    if (preset_write_needed[i])
                        safety.set_safety_preset(i, presets[i]);
                }
                safety.set_safety_preset(0, presets[0]);
            }
            if (calib_changed)
                _dev.as<rs2::auto_calibrated_device>().set_calibration_config(calib);
        }
        catch (...)
        {
            safety.set_option(RS2_OPTION_SAFETY_MODE, static_cast<float>(RS2_SAFETY_MODE_RUN));
            throw;
        }
        safety.set_option(RS2_OPTION_SAFETY_MODE, static_cast<float>(RS2_SAFETY_MODE_RUN));
        ROS_INFO("Safety table parameters applied");
    }
    catch (const std::exception& e)
    {
        // Best effort by design: the driver keeps streaming with the values
        // already in flash.
        ROS_ERROR_STREAM("Failed to apply safety table parameters: " << e.what());
    }
    return false;
}
