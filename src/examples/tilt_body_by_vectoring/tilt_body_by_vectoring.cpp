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

#include "tilt_body_by_vectoring.hpp"

/**
 * GroundRover attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int tilt_body_by_vectoring_main(int argc, char *argv[]);

namespace TBBV_control
{
	TiltBodyByVectoring	*g_control = nullptr;
}

TiltBodyByVectoring::TiltBodyByVectoring() :
	_mavlink_log_pub(nullptr),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt")),

	_nonfinite_input_perf(perf_alloc(PC_COUNT, "gnda_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "gnda_nano")),	
	_actuators_0_circuit_breaker_enabled(false)	
{
	_parameter_handles.bat_scale_en = param_find("TBBV_BAT_SCA_EN");

	/* fetch initial parameter values */
	parameters_update();
}

TiltBodyByVectoring::~TiltBodyByVectoring()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	TBBV_control::g_control = nullptr;
}

void
TiltBodyByVectoring::parameters_update()
{
	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* pid parameters*/
	pid_init(&_roll_rate_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
	pid_set_parameters(&_roll_rate_ctrl,
			   1.0f,
			   0.0f,
			   0.0f,
			   0.0f,
			   1.0f);
}

void
TiltBodyByVectoring::vehicle_control_mode_poll()
{
	bool updated = false;
	orb_check(_vcontrol_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
TiltBodyByVectoring::manual_control_setpoint_poll()
{
	bool updated = false;
	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
TiltBodyByVectoring::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
TiltBodyByVectoring::actuator_armed_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_actuator_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _actuator_armed_sub, &_actuator_armed);
	}
}

void 
TiltBodyByVectoring::attitude_control(float dt)
{	
	/* get current rotation matrix from control state quaternions */
	matrix::Quaternionf q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	matrix::Dcmf R_est(q_att);

	matrix::SquareMatrix<float, 3> I;
	I = matrix::eye<float, 3>();

	matrix::Dcmf R_des(I);
	R_des = R_des.transpose();

	matrix::Dcmf R_diff( R_est * R_des );

	matrix::AxisAnglef rotation_des(R_diff);
	/*math::Matrix<3, 3> R_est = q_att.to_dcm();
	
	math::Matrix<3, 3> R_des ;

	R_des(0,0) = 0.0f;
	R_des(0,1) = 0.0f;
	R_des(0,2) = 0.0f;

	R_des(1,0) = 0.0f;
	R_des(1,1) = 0.0f;
	R_des(1,2) = 0.0f;

	R_des(2,0) = 0.0f;
	R_des(2,1) = 0.0f;
	R_des(2,2) = 0.0f;

	R_des = R_des.transposed();

	math::Matrix<3, 3> R_diff = R_est * R_des ;

	math::Vector<3> rotation_angle = matrix::AxisAngle( R_diff ) ; 
	*/
	float att_roll_tc  = 0.1f;
	float att_pitch_tc = 0.1f;
	float att_yaw_tc   = 0.1f;
	
	{
		/* Future PID controller */
	}

	_v_rates_sp.roll  = -rotation_des(0)/att_roll_tc  ;
	_v_rates_sp.pitch = -rotation_des(1)/att_pitch_tc ;
	_v_rates_sp.yaw   = -rotation_des(2)/att_yaw_tc   ;
}

void
TiltBodyByVectoring::body_rates_control(float dt)
{
	math::Vector<3> rate_mea(_ctrl_state.roll_rate, _ctrl_state.pitch_rate, _ctrl_state.yaw_rate);
	math::Vector<3> rate_sp(_v_rates_sp.roll, _v_rates_sp.pitch, _v_rates_sp.yaw);

	math::Vector<3> rate_error = rate_sp - rate_mea ;	

	/* time constant (inverse of P gain ) */
	float roll_tc = 10.0f;
	float pitch_tc = 10.0f;
	float yaw_tc = 10.0f;

	{
		/* Future PID controller */
	}

	math::Vector<3> rate_tc(1.0f/roll_tc,1.0f/pitch_tc,1.0f/yaw_tc);
	math::Vector<3> torque_des = rate_tc.emult( J * rate_error ) ;

	//torque_des += ( rate_mea % (J * rate_mea) ) ;

	mx = torque_des(0);
	my = torque_des(1);
	mz = torque_des(2);
}

void 
TiltBodyByVectoring::actuator_mixer(float dt)
{
	float Kmx = 0.0f;
	float Kmy = 1.0f;
	float Kmz = 0.0f;

	fz = -10.0f; /* Newton */

	fz = math::max( -fz , 0.0f );
	int sign_my ;
	if(my >=0.0f){
		sign_my = 1;
	}else{
		sign_my = -1;
	}

	my = math::min( fabsf(Kmy*my) , fz/4 )*sign_my ;

	float A1_long = fz/4 - Kmx*mx ;
	float A1_lat  = fy/4 - Kmy*my + Kmz*mz ;

	float A2_long = fz/4 + Kmx*mx ;
	float A2_lat  = fy/4 - Kmy*my - Kmz*mz ;

	float A3_long = fz/4 - Kmx*mx ;
	float A3_lat  = fy/4 + Kmy*my + Kmz*mz ;

	float A4_long = fz/4 + Kmx*mx ;
	float A4_lat  = fy/4 + Kmy*my - Kmz*mz ;

	A1_long = math::constrain(A1_long, 0.5f, 6.0f);
	A2_long = math::constrain(A2_long, 0.5f, 6.0f);
	A3_long = math::constrain(A3_long, 0.5f, 6.0f);
	A4_long = math::constrain(A4_long, 0.5f, 6.0f);

	t1 = sqrtf(A1_long*A1_long + A1_lat*A1_lat) ;
	t2 = sqrtf(A2_long*A2_long + A2_lat*A2_lat) ;
	t3 = sqrtf(A3_long*A3_long + A3_lat*A3_lat) ;
	t4 = sqrtf(A4_long*A4_long + A4_lat*A4_lat) ;

	z1 = atanf(A1_lat / (A1_long)) ;
	z2 = atanf(A2_lat / (A2_long)) ;
	z3 = atanf(A3_lat / (A3_long)) ;
	z4 = atanf(A4_lat / (A4_long)) ;
}

void
TiltBodyByVectoring::actuator_normalize()
{
	float Thrust_max = 6.0f ; /* Newton */

	t1 = t1/Thrust_max ;
	t2 = t2/Thrust_max ;
	t3 = t3/Thrust_max ;
	t4 = t4/Thrust_max ;

	float lower_lim = 0.0f ;
	float upper_lim = 1.0f ;
	t1 = math::constrain(t1 , lower_lim , upper_lim);
	t2 = math::constrain(t2 , lower_lim , upper_lim);
	t3 = math::constrain(t3 , lower_lim , upper_lim);
	t4 = math::constrain(t4 , lower_lim , upper_lim);

	float Tilt_max = math::radians(40.0f); /* radian */
	z1 = z1/Tilt_max ;
	z2 = z2/Tilt_max ;
	z3 = z3/Tilt_max ;
	z4 = z4/Tilt_max ;

	lower_lim = -1.0f;
	z1 = math::constrain(z1 , lower_lim , upper_lim);
	z2 = math::constrain(z2 , lower_lim  , upper_lim);
	z3 = math::constrain(z3 , lower_lim  , upper_lim);
	z4 = math::constrain(z4 , lower_lim  , upper_lim);

} 
void 
TiltBodyByVectoring::actuator_set()
{
	/* motor */
	_actuators.control[0] = t1 ;
	_actuators.control[1] = t2 ;
	_actuators.control[2] = t3 ;
	_actuators.control[3] = t4 ;

	/* tilt servo */
	_actuators.control[4] = z1 ;
	_actuators.control[5] = z2 ;
	_actuators.control[6] = z3 ;
	_actuators.control[7] = z4 ;

}

void
TiltBodyByVectoring::actuator_set_zero()
{
	/* motor */
	_actuators.control[0] = NAN_VALUE ;
	_actuators.control[1] = NAN_VALUE ;
	_actuators.control[2] = NAN_VALUE ;
	_actuators.control[3] = NAN_VALUE ;

	/* tilt servo */
	_actuators.control[4] = 0.0f ;
	_actuators.control[5] = 0.0f ;
	_actuators.control[6] = 0.0f ;
	_actuators.control[7] = 0.0f ;
}

void
TiltBodyByVectoring::publish_vehicle_rates_setpoint()
{
	if (_rates_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_rates_setpoint), _rates_sp_pub, &_v_rates_sp);
	} else {
		_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
	}
}

void 
TiltBodyByVectoring::force_enable_motor()
{
	int pwm_value = PWM_DEFAULT_MAX;
	int ret;
	unsigned servo_count;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_max_values;
	memset(&pwm_max_values, 0, sizeof(pwm_max_values));

	for (int i = 0; i < 4 ; i++) {
		pwm_max_values.values[i] = pwm_value;
		pwm_max_values.channel_count = 4;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);

	if (ret != OK) {
		PX4_WARN("failed setting max values");
	}

	px4_close(fd);
}

void 
TiltBodyByVectoring::force_disable_motor()
{
	int pwm_value = PWM_MOTOR_OFF;
	int ret;
	unsigned servo_count;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_max_values;
	memset(&pwm_max_values, 0, sizeof(pwm_max_values));

	for (int i = 0; i < 4 ; i++) {
		pwm_max_values.values[i] = pwm_value;
		pwm_max_values.channel_count = 4;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);
	/*
	for (int i = 0; i < 4 ; i++) {
		ret = px4_ioctl(fd, PWM_SERVO_SET(i), (unsigned long)pwm_value);
	}
	*/

	if (ret != OK) {
		PX4_WARN("failed setting max values");
	}

	px4_close(fd);
}

void
TiltBodyByVectoring::task_main_trampoline(int argc, char *argv[])
{
	TBBV_control::g_control->task_main();
}

void
TiltBodyByVectoring::task_main()
{	
	/* Wait */
	usleep(1000*1000);
	
	mavlink_log_info(&_mavlink_log_pub, "[TBBV] started");

	J(0,0) = 1.0f;
	J(0,1) = 0.0f;
	J(0,2) = 0.0f;

	J(1,0) = 0.0f;
	J(1,1) = 1.0f;
	J(1,2) = 0.0f;

	J(2,0) = 0.0f;
	J(2,1) = 0.0f;
	J(2,2) = 1.0f;

	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	_actuator_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_control_mode_poll();
	manual_control_setpoint_poll();
	battery_status_poll();
	actuator_armed_poll();
	
	//bool current_armed = false  ;
	//bool prev_armed = true ;

	//force_disable_motor();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f ||
			    fabsf(deltaT) < 0.00001f ||
			    !PX4_ISFINITE(deltaT)) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			vehicle_control_mode_poll();
			manual_control_setpoint_poll();
			battery_status_poll();
			actuator_armed_poll();

			/* Body rate control */
			float dt = deltaT ; 

			if (_vcontrol_mode.flag_control_attitude_enabled){
				attitude_control(dt);
			}
			if (_vcontrol_mode.flag_control_rates_enabled) {
				/* run rates controller */
				body_rates_control(dt);
				
				actuator_mixer(dt);
				actuator_normalize();

				actuator_set();				
			} else {
				actuator_set_zero();
				/* off */
			}


			/*
			if (_parameters.bat_scale_en && _battery_status.scale > 0.0f)
			{
				for (int i = 0; i < number_of_motor; ++i)
				{
					_actuators.control[i] *= _battery_status.scale;					
				}
			}*/

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			
			/* calling publish function */
			publish_vehicle_rates_setpoint();
			/*
			prev_armed = current_armed ;
			current_armed = _actuator_armed.armed ;

			if(current_armed && prev_armed){
				//armed - armed	
				warnx("1");						
			}
			else if(current_armed && !prev_armed){
				//armed - disarmed
				warnx("2");
				force_enable_motor();
			}
			else if(!current_armed && prev_armed){
				//disarmed - armed
				warnx("3");
				force_disable_motor();
			}
			else if(!current_armed && !prev_armed){
				warnx("4");
				//disarmed - disarmed
			}*/

			/* Only publish if any of the proper modes are enabled */
			
			if (!_actuators_0_circuit_breaker_enabled) {
				if (_vcontrol_mode.flag_control_rates_enabled) {

					/* publish the actuator controls */
					if (_actuators_0_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
					}
				}
			}
			
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}

int
TiltBodyByVectoring::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("tilt_body_by_vectoring",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&TiltBodyByVectoring::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int tilt_body_by_vectoring_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: gnd_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (TBBV_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		TBBV_control::g_control = new TiltBodyByVectoring;

		if (TBBV_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (PX4_OK != TBBV_control::g_control->start()) {
			delete TBBV_control::g_control;
			TBBV_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (TBBV_control::g_control == nullptr || !TBBV_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (TBBV_control::g_control == nullptr || !TBBV_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (TBBV_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete TBBV_control::g_control;
		TBBV_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (TBBV_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
