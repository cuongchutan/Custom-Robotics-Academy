/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{190};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	actuator_armed = 0,
	actuator_controls = 1,
	actuator_controls_0 = 2,
	actuator_controls_1 = 3,
	actuator_controls_2 = 4,
	actuator_controls_3 = 5,
	actuator_controls_4 = 6,
	actuator_controls_5 = 7,
	actuator_controls_virtual_fw = 8,
	actuator_controls_virtual_mc = 9,
	actuator_outputs = 10,
	adc_report = 11,
	airspeed = 12,
	airspeed_validated = 13,
	airspeed_wind = 14,
	battery_status = 15,
	camera_capture = 16,
	camera_trigger = 17,
	camera_trigger_secondary = 18,
	cellular_status = 19,
	collision_constraints = 20,
	collision_report = 21,
	commander_state = 22,
	control_allocator_status = 23,
	cpuload = 24,
	debug_array = 25,
	debug_key_value = 26,
	debug_value = 27,
	debug_vect = 28,
	differential_pressure = 29,
	distance_sensor = 30,
	ekf2_timestamps = 31,
	ekf_gps_drift = 32,
	esc_report = 33,
	esc_status = 34,
	estimator_attitude = 35,
	estimator_global_position = 36,
	estimator_innovation_test_ratios = 37,
	estimator_innovation_variances = 38,
	estimator_innovations = 39,
	estimator_local_position = 40,
	estimator_odometry = 41,
	estimator_optical_flow_vel = 42,
	estimator_selector_status = 43,
	estimator_sensor_bias = 44,
	estimator_states = 45,
	estimator_status = 46,
	estimator_status_flags = 47,
	estimator_visual_odometry_aligned = 48,
	estimator_wind = 49,
	follow_target = 50,
	fw_virtual_attitude_setpoint = 51,
	generator_status = 52,
	geofence_result = 53,
	gimbal_device_attitude_status = 54,
	gimbal_device_information = 55,
	gimbal_device_set_attitude = 56,
	gimbal_manager_information = 57,
	gimbal_manager_set_attitude = 58,
	gimbal_manager_set_manual_control = 59,
	gimbal_manager_status = 60,
	gps_dump = 61,
	gps_inject_data = 62,
	heater_status = 63,
	home_position = 64,
	hover_thrust_estimate = 65,
	input_rc = 66,
	iridiumsbd_status = 67,
	irlock_report = 68,
	landing_gear = 69,
	landing_target_innovations = 70,
	landing_target_pose = 71,
	led_control = 72,
	log_message = 73,
	logger_status = 74,
	mag_worker_data = 75,
	manual_control_setpoint = 76,
	manual_control_switches = 77,
	mavlink_log = 78,
	mc_virtual_attitude_setpoint = 79,
	mission = 80,
	mission_result = 81,
	mount_orientation = 82,
	multirotor_motor_limits = 83,
	navigator_mission_item = 84,
	obstacle_distance = 85,
	obstacle_distance_fused = 86,
	offboard_control_mode = 87,
	onboard_computer_status = 88,
	optical_flow = 89,
	orb_multitest = 90,
	orb_test = 91,
	orb_test_large = 92,
	orb_test_medium = 93,
	orb_test_medium_multi = 94,
	orb_test_medium_queue = 95,
	orb_test_medium_queue_poll = 96,
	orb_test_medium_wrap_around = 97,
	orbit_status = 98,
	parameter_update = 99,
	ping = 100,
	position_controller_landing_status = 101,
	position_controller_status = 102,
	position_setpoint = 103,
	position_setpoint_triplet = 104,
	power_button_state = 105,
	power_monitor = 106,
	pwm_input = 107,
	px4io_status = 108,
	qshell_req = 109,
	qshell_retval = 110,
	radio_status = 111,
	rate_ctrl_status = 112,
	rc_channels = 113,
	rc_parameter_map = 114,
	rpm = 115,
	rtl_flight_time = 116,
	safety = 117,
	satellite_info = 118,
	sensor_accel = 119,
	sensor_accel_fifo = 120,
	sensor_baro = 121,
	sensor_combined = 122,
	sensor_correction = 123,
	sensor_gps = 124,
	sensor_gyro = 125,
	sensor_gyro_fft = 126,
	sensor_gyro_fifo = 127,
	sensor_mag = 128,
	sensor_preflight_mag = 129,
	sensor_selection = 130,
	sensors_status_imu = 131,
	system_power = 132,
	takeoff_status = 133,
	task_stack_info = 134,
	tecs_status = 135,
	telemetry_status = 136,
	test_motor = 137,
	timesync = 138,
	timesync_status = 139,
	trajectory_bezier = 140,
	trajectory_setpoint = 141,
	trajectory_waypoint = 142,
	transponder_report = 143,
	tune_control = 144,
	uavcan_parameter_request = 145,
	uavcan_parameter_value = 146,
	ulog_stream = 147,
	ulog_stream_ack = 148,
	vehicle_acceleration = 149,
	vehicle_actuator_setpoint = 150,
	vehicle_air_data = 151,
	vehicle_angular_acceleration = 152,
	vehicle_angular_acceleration_setpoint = 153,
	vehicle_angular_velocity = 154,
	vehicle_angular_velocity_groundtruth = 155,
	vehicle_attitude = 156,
	vehicle_attitude_groundtruth = 157,
	vehicle_attitude_setpoint = 158,
	vehicle_command = 159,
	vehicle_command_ack = 160,
	vehicle_constraints = 161,
	vehicle_control_mode = 162,
	vehicle_global_position = 163,
	vehicle_global_position_groundtruth = 164,
	vehicle_gps_position = 165,
	vehicle_imu = 166,
	vehicle_imu_status = 167,
	vehicle_land_detected = 168,
	vehicle_local_position = 169,
	vehicle_local_position_groundtruth = 170,
	vehicle_local_position_setpoint = 171,
	vehicle_magnetometer = 172,
	vehicle_mocap_odometry = 173,
	vehicle_odometry = 174,
	vehicle_rates_setpoint = 175,
	vehicle_roi = 176,
	vehicle_status = 177,
	vehicle_status_flags = 178,
	vehicle_thrust_setpoint = 179,
	vehicle_torque_setpoint = 180,
	vehicle_trajectory_bezier = 181,
	vehicle_trajectory_waypoint = 182,
	vehicle_trajectory_waypoint_desired = 183,
	vehicle_vision_attitude = 184,
	vehicle_visual_odometry = 185,
	vtol_vehicle_status = 186,
	wheel_encoders = 187,
	wind = 188,
	yaw_estimator_status = 189,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
