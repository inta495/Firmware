/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file bottle_drop.cpp
 *
 * Bottle drop module for Outback Challenge 2014, Team Swiss Fang
 *
 * @author Dominik Juchli <juchlid@ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>


/**
 * humming app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int humming_main(int argc, char *argv[]);

class Humming
{
public:
	/**
	 * Constructor
	 */
	Humming();

	/**
	 * Destructor, also kills task.
	 */
	~Humming();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */
	void		status();

	void		open_bay();
	void		close_bay();
	void		drop();
	void		lock_release();

	/*
	 *  new function
	 */
	void		goto_pos(float x , float y);
	void 		goto_home();
	void 		liquid_pos(float z);
	void 		liquid_back();
	void		probe_pos(float z);
	void		probe_back();
	
private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	orb_advert_t	_mavlink_log_pub;

	int		_command_sub;
	struct vehicle_command_s	_command;

	int		_params_sub;			/**< parameter updates subscription */
	
	bool 	_humming_sys_start;

	struct actuator_controls_s _actuators;
	orb_advert_t	_actuator_pub;

	float actuators_setpoint[4];

	int counter;

	struct {

		param_t act_change_rate;

	}	_params_handles;		/**< handles for interesting parameters */

	struct {

		float act_change_rate;	

	}	_params{};

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll(bool force);

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, unsigned result);

	/**
	 * Set the actuators
	 */
	int		actuators_publish();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	void ack_vehicle_command(orb_advert_t &vehicle_command_ack_pub, struct vehicle_command_s &cmd, uint32_t result);
};

namespace humming
{
Humming	*g_humming;
}

Humming::Humming() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_log_pub(nullptr),
	_command_sub(-1),
	_command {},
	_params_sub(-1),
	_humming_sys_start(false),	
	_actuators {},
	_actuator_pub(nullptr)
{	
	for (int i = 0; i < 4; ++i)
	{
		actuators_setpoint[i] = -1.0f;
	}
	counter = 0;

	_params_handles.act_change_rate = param_find("HUM_CHANGE_RATE");

	/* fetch initial parameter values */
	parameters_update();
}

Humming::~Humming()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	humming::g_humming = nullptr;
}

int
Humming::parameters_update()
{	
	float v;
	param_get(_params_handles.act_change_rate, &v);
	_params.act_change_rate = v;
	return OK;
}

void
Humming::parameter_update_poll(bool force)
{
	bool updated;
	struct parameter_update_s param_update;
	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {		
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);		
	}
	if (updated || force){		
		parameters_update();
	}
}

int
Humming::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("humming",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT ,
					2031,
					(px4_main_t)&Humming::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
Humming::status()
{
	warnx("running");
}

void
Humming::open_bay()
{
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = 1.0f;

	actuators_publish();

	usleep(500 * 1000);
}

void
Humming::close_bay()
{
	// closed door and locked survival kit
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = -1.0f;

	actuators_publish();

	// delay until the bay is closed
	usleep(500 * 1000);
}

void
Humming::drop()
{
	open_bay();

	_actuators.control[2] = 1.0f;

	actuators_publish();

	// Give it time to drop
	usleep(1000 * 1000);
}

void
Humming::lock_release()
{
	_actuators.control[2] = 1.0f;
	actuators_publish();

	usleep(1000 * 1000);
}
/*
 * new function
 */
void
Humming::goto_pos(float x , float y)
{	
	actuators_setpoint[0] = x;
	actuators_setpoint[1] = y;
	
	mavlink_log_info(&_mavlink_log_pub, "new setpoint [goto_pos]");
}


void
Humming::goto_home()
{	
	actuators_setpoint[0] = -1.0f;
	actuators_setpoint[1] = -1.0f;	
	mavlink_log_info(&_mavlink_log_pub, "new setpoint [goto_home]");
}

void
Humming::liquid_pos(float z)
{
	actuators_setpoint[2] = z;
	mavlink_log_info(&_mavlink_log_pub, "new setpoint [liquid_pos]");
}

void
Humming::liquid_back()
{
	actuators_setpoint[2] = -1.0f;
	mavlink_log_info(&_mavlink_log_pub, "new setpoint [liquid_back]");
}

void
Humming::probe_pos(float z)
{	
	actuators_setpoint[3] = z;
	mavlink_log_info(&_mavlink_log_pub, "new setpoint [probe_pos]");
}

void
Humming::probe_back()
{
	actuators_setpoint[3] = -1.0f;
	mavlink_log_info(&_mavlink_log_pub, "new setpoint [probe_back]");
}

int
Humming::actuators_publish()
{
	_actuators.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (_actuator_pub != nullptr) {
		return orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &_actuators);

	} else {
		_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);

		if (_actuator_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

void
Humming::task_main()
{
	/* Wait */
	usleep(1000*1000);

	mavlink_log_info(&_mavlink_log_pub, "[humming] started");

	_command_sub = orb_subscribe(ORB_ID(vehicle_command));	

	orb_advert_t vehicle_command_ack_pub = nullptr;

	bool updated = false;
	/* initialize parameters cache */
	parameters_update();

	// wakeup source(s)
	struct pollfd fds[1];

	// Setup of loop
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;

	/* start */
	/*
	goto_pos(1.0f,1.0f);
	liquid_pos(1.0f);
	probe_pos(1.0f);
	*/

	const unsigned sleeptime_us = 9500;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* vehicle commands updated */
		if (fds[0].revents & POLLIN) {

			parameter_update_poll(true);

			orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
			handle_command(&_command);			
			ack_vehicle_command(vehicle_command_ack_pub, _command,vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		}

		while( _humming_sys_start ){

			parameter_update_poll(true);

			orb_check(_command_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
				handle_command(&_command);
				ack_vehicle_command(vehicle_command_ack_pub, _command,vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
			}
			
			for (uint8_t i = 0; i < 4; i++)
			{
				if( actuators_setpoint[i] - _actuators.control[i] > 0 ){
					_actuators.control[i] = _actuators.control[i] + _params.act_change_rate ;
				}
				else if( actuators_setpoint[i] < _actuators.control[i] ){
					_actuators.control[i] = _actuators.control[i] - _params.act_change_rate ;
				}
				float error = actuators_setpoint[i] - _actuators.control[i] ;
				_actuators.control[i] =  _actuators.control[i] + math::constrain( error , - _params.act_change_rate , + _params.act_change_rate);
			}
			actuators_publish();

			usleep(sleeptime_us);
		}	
			
		usleep(sleeptime_us);
	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

void
Humming::handle_command(struct vehicle_command_s *cmd)
{
	switch (cmd->command) {
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:

		/*
		 * param1 and param2 set to 1: open and drop
		 * param1 set to 1: open
		 * else: close (and don't drop)
		 */
		if (cmd->param1 > 0.5f && cmd->param2 > 0.5f) {
			open_bay();
			drop();
			mavlink_log_critical(&_mavlink_log_pub, "drop bottle");

		} else if (cmd->param1 > 0.5f) {
			open_bay();
			mavlink_log_critical(&_mavlink_log_pub, "opening bay");

		} else {
			lock_release();
			close_bay();
			mavlink_log_critical(&_mavlink_log_pub, "closing bay");
		}

		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
		/* param1 is 1 */
		if (cmd->param1 > 0.5f && cmd->param1 < 1.5f ){
			if(_humming_sys_start){
				goto_home();				
			}
			else{
				mavlink_log_critical(&_mavlink_log_pub, "please start Humming system first");
			}
		}
		/* param1 is 2 */
		else if (cmd->param1 > 1.5f && cmd->param1 < 2.5f ){
			if(_humming_sys_start){
				liquid_back();				
			}
			else{
				mavlink_log_critical(&_mavlink_log_pub, "please start Humming system first");
			}
		}
		/* param1 is 3 */
		else if (cmd->param1 > 2.5f && cmd->param1 < 3.5f ){
			if(_humming_sys_start){
				probe_back();				
			}
			else{
				mavlink_log_critical(&_mavlink_log_pub, "please start Humming system first");
			}
		}
		else if (cmd->param1 > 3.5f && cmd->param1 < 4.5f){
			if(!_humming_sys_start){
				_humming_sys_start = true;
				goto_home();
				liquid_back();
				probe_back();		
				//mavlink_log_critical(&_mavlink_log_pub, "Humming system started");
			}
			else{
				mavlink_log_critical(&_mavlink_log_pub, "Humming system already has started");
			}
		}
		else if (cmd->param1 > 4.5f && cmd->param1 < 5.5f){
			if(_humming_sys_start){				
				float x = cmd->param2;
				float y = cmd->param3;
				goto_pos(x , y);
				//mavlink_log_critical(&_mavlink_log_pub, "goto position %d %d" , (int)(x*1000.0f), (int)(y*1000.0f) );
			}
			else{
				mavlink_log_critical(&_mavlink_log_pub, "please start Humming system first");
			}
		}
		else if (cmd->param1 > 5.5f && cmd->param1 < 6.5f){
			if(_humming_sys_start){				
				float z = cmd->param4;				
				liquid_pos(z);
				//mavlink_log_critical(&_mavlink_log_pub, "liquid position %d" , (int)(z*1000.0f) );
			}
			else{
				mavlink_log_critical(&_mavlink_log_pub, "please start Humming system first");
			}
		}
		else if (cmd->param1 > 6.5f && cmd->param1 < 7.5f){
			if(_humming_sys_start){				
				float z = cmd->param5;				
				probe_pos(z);
				//mavlink_log_critical(&_mavlink_log_pub, "probe position %d" , (int)(z*1000.0f) );
			}
			else{
				mavlink_log_critical(&_mavlink_log_pub, "please start Humming system first");
			}
		}		
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;
	default:
		break;
	}
}

void
Humming::answer_command(struct vehicle_command_s *cmd, unsigned result)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(&_mavlink_log_pub, "command denied: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(&_mavlink_log_pub, "command failed: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(&_mavlink_log_pub, "command temporarily rejected: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(&_mavlink_log_pub, "command unsupported: %u", cmd->command);
		break;

	default:
		break;
	}
}

void
Humming::task_main_trampoline(int argc, char *argv[])
{
	humming::g_humming->task_main();
}

void 
Humming::ack_vehicle_command(orb_advert_t &vehicle_command_ack_pub, struct vehicle_command_s &cmd, uint32_t result)
{
	vehicle_command_ack_s vehicle_command_ack = {
		.timestamp = hrt_absolute_time(),
		.result_param2 = 0,
		.command = cmd.command,
		.result = (uint8_t)result,
		.from_external = 0,
		.result_param1 = 0,
		.target_system = cmd.source_system,
		.target_component = cmd.source_component
	};

	if (vehicle_command_ack_pub == nullptr) {
		vehicle_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &vehicle_command_ack,
	    vehicle_command_ack_s::ORB_QUEUE_LENGTH);
	} else {
		orb_publish(ORB_ID(vehicle_command_ack), vehicle_command_ack_pub, &vehicle_command_ack);
	}
} 

static void usage()
{
	errx(1, "usage: humming {start|stop|status}");
}

int humming_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (humming::g_humming != nullptr) {
			errx(1, "already running");
		}

		humming::g_humming = new Humming;

		if (humming::g_humming == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != humming::g_humming->start()) {
			delete humming::g_humming;
			humming::g_humming = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (humming::g_humming == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete humming::g_humming;
		humming::g_humming = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		humming::g_humming->status();

	} else {
		usage();
	}

	return 0;
}
