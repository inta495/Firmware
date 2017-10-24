/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/circuit_breaker.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/uORB.h>

#include <systemlib/mavlink_log.h>
#include <arch/board/board.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>

#include "drivers/drv_pwm_output.h"

using matrix::Eulerf;
using matrix::Quatf;

class TiltBodyByVectoring
{
public:
	TiltBodyByVectoring();
	~TiltBodyByVectoring();

	int start();
	bool task_running() { return _task_running; }

private:

	bool		_task_should_exit{false};		/**< if true, attitude control task should exit */
	bool		_task_running{false};			/**< if true, task is running in its mainloop */
	int		_control_task{-1};			/**< task handle */

	/* All subscription */
	int		_battery_status_sub{-1};		/**< battery status subscription */
	int		_ctrl_state_sub{-1};		/**< control state subscription */
	int		_manual_sub{-1};			/**< notification of manual control updates */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_vcontrol_mode_sub{-1};		/**< vehicle status subscription */
	int 	_actuator_armed_sub{-1};	

	/* All advertise */
	orb_advert_t	_actuators_0_pub{nullptr};		/**< actuator control group 0 setpoint */
	orb_advert_t	_att_sp_pub{nullptr};		/**< attitude setpoint */
	orb_advert_t	_rates_sp_pub{nullptr};		/**< rates setpoint */
	orb_advert_t	_mavlink_log_pub;

	/* All uORB topic struct */
	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	battery_status_s				_battery_status {};	/**< battery status */
	control_state_s				_ctrl_state {};	/**< control state */
	manual_control_setpoint_s		_manual {};		/**< r/c channel data */
	vehicle_control_mode_s			_vcontrol_mode {};		/**< vehicle control mode */
	vehicle_rates_setpoint_s		_v_rates_sp {};
	actuator_armed_s 			_actuator_armed {};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_debug{false};				/**< if set to true, print debug output */
	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	struct {

		int32_t bat_scale_en;			/**< Battery scaling enabled */

	} _parameters{};			/**< local copies of interesting parameters */

	struct {

		param_t bat_scale_en;

	} _parameter_handles{};		/**< handles for interesting parameters */

	PID_t			_roll_rate_ctrl{};
	PID_t			_pitch_rate_ctrl{};
	PID_t			_yaw_rate_ctrl{};

	math::Matrix<3,3> J{};

	/* Number of motor */
	int number_of_motor = 4;
	/* Force desired */
	float fx=0.0f ;
	float fy=0.0f ;
	float fz=0.0f ;

	/* Torque desired */
	float mx=0.0f ;
	float my=0.0f ;
	float mz=0.0f ;

	/* actuator output */
	float t1=0.0f;
	float t2=0.0f;
	float t3=0.0f;
	float t4=0.0f;

	float z1=0.0f;
	float z2=0.0f;
	float z3=0.0f;
	float z4=0.0f;

	/* Polling function */
	void		parameters_update();
	void		vehicle_control_mode_poll();
	void		manual_control_setpoint_poll();	
	void		battery_status_poll();
	void		actuator_armed_poll();

	/* Control function */
	void		attitude_control(float dt);
	void		body_rates_control(float dt);
	void		actuator_mixer(float dt);
	void		actuator_normalize();

	void		actuator_set();
	void		actuator_set_zero();
	
	/* publish function */
	void		publish_vehicle_rates_setpoint();

	/* utility */
	void		force_enable_motor();
	void		force_disable_motor();
	/* Main */
	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

};
