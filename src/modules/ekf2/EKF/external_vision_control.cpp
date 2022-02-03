/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/**
 * @file external_vision_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {
		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if (_params.fusion_mode & MASK_ROTATE_EV && !_control_status.flags.ev_yaw) {
			// update the rotation matrix which rotates EV measurements into the EKF's navigation frame
			// Calculate the quaternion delta that rotates from the EV to the EKF reference frame at the EKF fusion time horizon.
			const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
			_R_ev_to_ekf = Dcmf(q_error);

		} else {
			_R_ev_to_ekf.setIdentity();
		}


		controlEvPosFusion();
		controlEvVelFusion();
		controlEvYawFusion();

		// record observation and estimate for use next time
		_ev_sample_delayed_prev = _ev_sample_delayed;
		_hpos_pred_prev = _state.pos.xy();
		_hpos_prev_available = true;

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvFusion();
		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlEvPosFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVPOS)) {
		stopEvPosFusion();
		return;
	}

	if (_ev_data_ready) {

		if (_control_status_prev.flags.ev_pos && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
			resetHorizontalPosition();
		}

		// determine if we should use the horizontal position observations
		const bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;

		if (_control_status.flags.ev_pos) {

			if (_control_status_prev.flags.ev_pos && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
				resetHorizontalPosition();
			}

			if (continuing_conditions_passing) {

				fuseEvPosition();

				const bool is_fusion_failing = isTimedOut(_time_last_ev_pos_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_ev_pos_reset_available > 0) {
						// Data seems good, attempt a reset
						resetHorizontalPosition();

						if (_control_status.flags.in_air) {
							_nb_ev_pos_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						//_control_status.flags.this_sensor_fault = true; TODO
						stopEvPosFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopEvPosFusion();
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvPosFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV position fusion
				startEvPosFusion();

				if (_control_status.flags.ev_pos) {
					_nb_ev_pos_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.ev_pos && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvPosFusion();
	}
}

void Ekf::controlEvVelFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVVEL)) {
		stopEvVelFusion();
		return;
	}

	if (_ev_data_ready) {

		const bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL);
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;

		if (_control_status.flags.ev_vel) {

			if (_control_status_prev.flags.ev_vel && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
				resetVelocity();

			} else {
				// check if we have been deadreckoning too long
				if (isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)) {
					// only reset velocity if we have no another source of aiding constraining it
					if (isTimedOut(_time_last_of_fuse, (uint64_t)1e6) && isTimedOut(_time_last_hor_pos_fuse, (uint64_t)1e6)) {

						resetVelocity();
					}
				}
			}

			if (continuing_conditions_passing) {

				fuseEvVelocity();

				const bool is_fusion_failing = isTimedOut(_time_last_ev_vel_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_ev_vel_reset_available > 0) {
						// Data seems good, attempt a reset
						resetVelocity();

						if (_control_status.flags.in_air) {
							_nb_ev_vel_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						//_control_status.flags.this_sensor_fault = true; TODO
						stopEvVelFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopEvVelFusion();
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvVelFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV velocity fusion
				startEvVelFusion();

				if (_control_status.flags.ev_vel) {
					_nb_ev_vel_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.ev_vel && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvVelFusion();
	}
}

void Ekf::controlEvYawFusion()
{
	if (!(_params.fusion_mode & MASK_USE_EVYAW)) {
		stopEvYawFusion();
		return;
	}

	if (_ev_data_ready) {

		const bool continuing_conditions_passing = isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL) && !_inhibit_ev_yaw_use;
		const bool starting_conditions_passing = continuing_conditions_passing && _control_status.flags.tilt_align; // TODO

		if (_control_status.flags.ev_yaw) {

			if (_control_status_prev.flags.ev_yaw && (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter)) {
				resetYawToEv();
			}

			if (continuing_conditions_passing) {

				fuseEvYaw();

				const bool is_fusion_failing = isTimedOut(_time_last_ev_yaw_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_ev_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToEv();

						if (_control_status.flags.in_air) {
							_nb_ev_yaw_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						//_control_status.flags.this_sensor_fault = true; TODO
						stopEvYawFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopEvYawFusion();
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV yaw fusion
				startEvYawFusion();

				if (_control_status.flags.ev_yaw) {
					_nb_ev_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.ev_yaw && isTimedOut(_time_last_ext_vision, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvYawFusion();
	}
}

void Ekf::startEvPosFusion()
{
	_control_status.flags.ev_pos = true;
	resetHorizontalPosition();
	_information_events.flags.starting_vision_pos_fusion = true;
	ECL_INFO("starting vision pos fusion");
}

void Ekf::startEvVelFusion()
{
	_control_status.flags.ev_vel = true;
	resetVelocity();
	_information_events.flags.starting_vision_vel_fusion = true;
	ECL_INFO("starting vision vel fusion");
}

void Ekf::startEvYawFusion()
{
	// turn on fusion of external vision yaw measurements and disable all magnetometer fusion
	_control_status.flags.ev_yaw = true;
	_control_status.flags.yaw_align = true;

	_control_status.flags.mag_dec = false;

	stopMagHdgFusion();
	stopMag3DFusion();

	resetYawToEv();

	_information_events.flags.starting_vision_yaw_fusion = true;
	ECL_INFO("starting vision yaw fusion");
}

void Ekf::stopEvFusion()
{
	stopEvPosFusion();
	stopEvVelFusion();
	stopEvYawFusion();
}

void Ekf::stopEvPosFusion()
{
	_control_status.flags.ev_pos = false;
	_ev_pos_innov.setZero();
	_ev_pos_innov_var.setZero();
	_ev_pos_test_ratio.setZero();

	_hpos_prev_available = false;
}

void Ekf::stopEvVelFusion()
{
	_control_status.flags.ev_vel = false;
	_ev_vel_innov.setZero();
	_ev_vel_innov_var.setZero();
	_ev_vel_test_ratio.setZero();
}

void Ekf::stopEvYawFusion()
{
	_control_status.flags.ev_yaw = false;
}

void Ekf::fuseEvPosition()
{
	Vector3f ev_pos_obs_var;

	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
	_ev_sample_delayed.pos -= pos_offset_earth;

	// Use an incremental position fusion method for EV position data if GPS is also used
	if (_params.fusion_mode & MASK_USE_GPS) {
		_fuse_hpos_as_odom = true;

	} else {
		_fuse_hpos_as_odom = false;
	}

	if (_fuse_hpos_as_odom) {
		if (_hpos_prev_available) {
			// calculate the change in position since the last measurement
			// rotate measurement into body frame is required when fusing with GPS
			Vector3f ev_delta_pos = _R_ev_to_ekf * Vector3f(_ev_sample_delayed.pos - _ev_sample_delayed_prev.pos);

			// use the change in position since the last measurement
			_ev_pos_innov(0) = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
			_ev_pos_innov(1) = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);

			// observation 1-STD error, incremental pos observation is expected to have more uncertainty
			Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
			ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
			ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.5f));
			ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.5f));
		}

	} else {
		// use the absolute position
		Vector3f ev_pos_meas = _R_ev_to_ekf * _ev_sample_delayed.pos;
		Matrix3f ev_pos_var = _R_ev_to_ekf * matrix::diag(_ev_sample_delayed.posVar) * _R_ev_to_ekf.transpose();

		_ev_pos_innov(0) = _state.pos(0) - ev_pos_meas(0);
		_ev_pos_innov(1) = _state.pos(1) - ev_pos_meas(1);

		ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.01f));
		ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.01f));

		// check if we have been deadreckoning too long
		if (isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)) {
			// only reset velocity if we have no another source of aiding constraining it
			if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
			    isTimedOut(_time_last_hor_vel_fuse, (uint64_t)1E6)) {

				resetVelocity();
			}

			resetHorizontalPosition();
		}
	}

	// innovation gate size
	const float ev_pos_innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.f);

	if (fuseHorizontalPosition(_ev_pos_innov, ev_pos_innov_gate, ev_pos_obs_var, _ev_pos_innov_var, _ev_pos_test_ratio)) {
		_time_last_ev_pos_fuse = _time_last_imu;
	}
}

void Ekf::fuseEvVelocity()
{
	_ev_vel_innov = _state.vel - getVisionVelocityInEkfFrame();

	const Vector3f obs_var = matrix::max(getVisionVelocityVarianceInEkfFrame(), sq(0.05f));

	const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);

	if (fuseHorizontalVelocity(_ev_vel_innov, innov_gate, obs_var, _ev_vel_innov_var, _ev_vel_test_ratio)) {
		_time_last_ev_vel_fuse = _time_last_imu;
	}

	if (fuseVerticalVelocity(_ev_vel_innov, innov_gate, obs_var, _ev_vel_innov_var, _ev_vel_test_ratio)) {
		_time_last_ev_vel_fuse = _time_last_imu;
	}
}

void Ekf::fuseEvYaw()
{
	if (shouldUse321RotationSequence(_R_to_earth)) {
		float measured_hdg = getEuler321Yaw(_ev_sample_delayed.quat);

		if (fuseYaw321(measured_hdg, _ev_sample_delayed.angVar)) {
			_time_last_ev_yaw_fuse = _time_last_imu;
		}

	} else {
		float measured_hdg = getEuler312Yaw(_ev_sample_delayed.quat);

		if (fuseYaw312(measured_hdg, _ev_sample_delayed.angVar)) {
			_time_last_ev_yaw_fuse = _time_last_imu;
		}
	}
}
