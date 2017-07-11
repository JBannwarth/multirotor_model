% %***************************************************************************
%  *
%  *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
%  *
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions
%  * are met:
%  *
%  * 1. Redistributions of source code must retain the above copyright
%  *    notice, this list of conditions and the following disclaimer.
%  * 2. Redistributions in binary form must reproduce the above copyright
%  *    notice, this list of conditions and the following disclaimer in
%  *    the documentation and/or other materials provided with the
%  *    distribution.
%  * 3. Neither the name PX4 nor the names of its contributors may be
%  *    used to endorse or promote products derived from this software
%  *    without specific prior written permission.
%  *
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  * 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
%  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
%  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  *
%  ****************************************************************************/

% %
%  * @file attitude_estimator_q_main.cpp
%  *
%  * Attitude estimator (quaternion based)
%  *
%  * @author Anton Babushkin <anton.babushkin@me.com>
% 

% #include <px4_config.h>
% #include <px4_posix.h>
% #include <px4_tasks.h>
% #include <unistd.h>
% #include <stdlib.h>
% #include <stdio.h>
% #include <stdbool.h>
% #include <poll.h>
% #include <fcntl.h>
% #include <float.h>
% #include <errno.h>
% #include <limits.h>
% #include <math.h>
% #include <uORB/uORB.h>
% #include <uORB/topics/sensor_combined.h>
% #include <uORB/topics/vehicle_attitude.h>
% #include <uORB/topics/control_state.h>
% #include <uORB/topics/vehicle_control_mode.h>
% #include <uORB/topics/vehicle_global_position.h>
% #include <uORB/topics/att_pos_mocap.h>
% #include <uORB/topics/airspeed.h>
% #include <uORB/topics/parameter_update.h>
% #include <uORB/topics/estimator_status.h>
% #include <drivers/drv_hrt.h>
% 
% #include <mathlib/mathlib.h>
% #include <mathlib/math/filter/LowPassFilter2p.hpp>
% #include <lib/geo/geo.h>
% 
% #include <systemlib/param/param.h>
% #include <systemlib/perf_counter.h>
% #include <systemlib/err.h>
% #include <systemlib/mavlink_log.h>
% 
% extern 'C' self._EXPORT int attitude_estimator_q_main(int argc, char *argv[]);
% 
% using math::Vector;
% using math::Matrix;
% using math::Quaternion;
% 
% class AttitudeEstimatorQ;
% 
% namespace attitude_estimator_q
% 
% AttitudeEstimatorQ *instance;
% }


% class AttitudeEstimatorQ
% 
% public:
% 	%*
% 	 * Constructor
% 	
% 	AttitudeEstimatorQ();
% 
% 	%*
% 	 * Destructor, also kills task.
% 	
% 	~AttitudeEstimatorQ();
% 
% 	%*
% 	 * Start task.
% 	 *
% 	 * @return		true on success.
% 	
% 	int		start();
% 
% 	static void	task_main_trampoline(int argc, char *argv[]);
% 
% 	void		task_main();
% 
% 	void		print();
% 
% private:
% 	static constexpr float self.dt_max = 0.02;
% 	bool		self.task_should_exit = false;		%*< if true, task should exit
% 	int		self.control_task = -1;			%*< task handle for task
% 
% 	int		self.sensors_sub = -1;
% 	int		self.params_sub = -1;
% 	int		self.vision_sub = -1;
% 	int		self.mocap_sub = -1;
% 	int		self.airspeed_sub = -1;
% 	int		self.global_pos_sub = -1;
% 	orb_advert_t	self.att_pub = nullptr;
% 	orb_advert_t	self.ctrl_state_pub = nullptr;
% 	orb_advert_t	self.est_state_pub = nullptr;
% 
% 	struct 
% 		param_t	w_acc;
% 		param_t	w_mag;
% 		param_t	w_ext_hdg;
% 		param_t	w_gyro_bias;
% 		param_t	mag_decl;
% 		param_t	mag_decl_auto;
% 		param_t	acc_comp;
% 		param_t	bias_max;
% 		param_t	ext_hdg_mode;
% 		param_t airspeed_mode;
% 	}		self.params_handles;		%*< handles for interesting parameters
% 
% 	float		self.w_accel = 0.0;
% 	float		self.w_mag = 0.0;
% 	float		self.w_ext_hdg = 0.0;
% 	float		self.w_gyro_bias = 0.0;
% 	float		self.mag_decl = 0.0;
% 	bool		self.mag_decl_auto = false;
% 	bool		self.acc_comp = false;
% 	float		self.bias_max = 0.0;
% 	int		self.ext_hdg_mode = 0;
% 	int 	self.airspeed_mode = 0;
% 
% 	Vector<3>	self.gyro;
% 	Vector<3>	self.accel;
% 	Vector<3>	self.mag;
% 
% 	vehicle_attitude_s self.vision = {};
% 	Vector<3>	self.vision_hdg;
% 
% 	att_pos_mocap_s self.mocap = {};
% 	Vector<3>	self.mocap_hdg;
% 
% 	airspeed_s self.airspeed = {};
% 
% 	Quaternion	self.q;
% 	Vector<3>	self.rates;
% 	Vector<3>	self.gyro_bias;
% 
% 	vehicle_global_position_s self.gpos = {};
% 	Vector<3>	self.vel_prev;
% 	Vector<3>	self.pos_acc;
% 
% 	% Low pass filter for accel/gyro
% 	math::LowPassFilter2p self.lp_accel_x;
% 	math::LowPassFilter2p self.lp_accel_y;
% 	math::LowPassFilter2p self.lp_accel_z;
% 	math::LowPassFilter2p self.lp_gyro_x;
% 	math::LowPassFilter2p self.lp_gyro_y;
% 	math::LowPassFilter2p self.lp_gyro_z;
% 
% 	hrt_abstime self.vel_prev_t = 0;
% 
% 	bool		self.inited = false;
% 	bool		self.data_good = false;
% 	bool		self.ext_hdg_good = false;
% 
% 	orb_advert_t	self.mavlink_log_pub = nullptr;
% 
% 	void update_parameters(bool force);
% 
% 	int update_subscriptions();
% 
% 	bool init();
% 
% 	bool update(float dt);
% 
% 	% Update magnetic declination (in rads) immediately changing yaw rotation
% 	void update_mag_declination(float new_declination);
% };


% AttitudeEstimatorQ:: :
function self = self = AttitudeEstimatorQ( self )
% 	self.vel_prev(0, 0, 0),
% 	self.pos_acc(0, 0, 0),
% 	self.lp_accel_x(250.0, 30.0),
% 	self.lp_accel_y(250.0, 30.0),
% 	self.lp_accel_z(250.0, 30.0),
% 	self.lp_gyro_x(250.0, 30.0),
% 	self.lp_gyro_y(250.0, 30.0),
% 	self.lp_gyro_z(250.0, 30.0)
	self.params_handles.w_acc		= param_find('ATT_W_ACC');
	self.params_handles.w_mag		= param_find('ATT_W_MAG');
	self.params_handles.w_ext_hdg	= param_find('ATT_W_EXT_HDG');
	self.params_handles.w_gyro_bias	= param_find('ATT_W_GYRO_BIAS');
	self.params_handles.mag_decl	= param_find('ATT_MAG_DECL');
	self.params_handles.mag_decl_auto	= param_find('ATT_MAG_DECL_A');
	self.params_handles.acc_comp	= param_find('ATT_ACC_COMP');
	self.params_handles.bias_max	= param_find('ATT_BIAS_MAX');
	self.params_handles.ext_hdg_mode	= param_find('ATT_EXT_HDG_M');
	self.params_handles.airspeed_mode = param_find('FW_ARSP_MODE');
end

%  Destructor, also kills task.
% AttitudeEstimatorQ::~
% function self = self = AttitudeEstimatorQDestructor( self )

% 	if (self.control_task ~= -1) 
% 		% task wakes up every 100ms or so at the longest
% 		self.task_should_exit = true;

% 		% wait for a second for the task to quit at our request
% 		unsigned i = 0;

% 		do 
% 			% wait 20ms
% 			usleep(20000);

% 			% if we have given up, kill it
% 			if (++i > 50) 
% 				px4_task_delete(self.control_task);
% 				break;
% 			end
% 		end while (self.control_task ~= -1);
% 	end

% 	attitude_estimator_q::instance = nullptr;
% end

% Probably not needed
% int AttitudeEstimatorQ::
% function [ self, output ] = start( self )
% %START
% 	ASSERT(self.control_task == -1);
% 
% 	% start the task
% % 	self.control_task = px4_task_spawn_cmd('attitude_estimator_q', ...
% % 					   SCHED_DEFAULT, ...
% % 					   SCHED_PRIORITY_MAX - 5, ...
% % 					   2000, ...
% % 					   (px4_main_t)&AttitudeEstimatorQ::task_main_trampoline,
% % 					   nullptr);
% 
% 	if (self.control_task < 0) 
% 		warn('task start failed');
% 		output = -errno;
%         return;
% 	end
% 
% 	output = true;
% end

% void AttitudeEstimatorQ::
% function self = print( self )
% %PRINT
%     % Empty
% end

% void AttitudeEstimatorQ::
% function self = task_main_trampoline( self )
% %TASK_MAIN_TRAMPOLINE
% 	% attitude_estimator_q::instance->
%     self = task_main( self );
% end

% void AttitudeEstimatorQ::
function self = task_main( self )
% ifdef self._PX4_POSIX
%     % All perf_counter_t
% 	self.perf_accel(perf_alloc_once(PC_ELAPSED, 'sim_accel_delay'));
% 	self.perf_mpu(perf_alloc_once(PC_ELAPSED, 'sim_mpu_delay'));
% 	self.perf_mag(perf_alloc_once(PC_ELAPSED, 'sim_mag_delay'));
% endif

    % Subscriptions
% 	self.sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
% 	self.vision_sub = orb_subscribe(ORB_ID(vehicle_vision_attitude));
% 	self.mocap_sub = orb_subscribe(ORB_ID(att_pos_mocap));
% 	self.airspeed_sub = orb_subscribe(ORB_ID(airspeed));
% 	self.params_sub = orb_subscribe(ORB_ID(parameter_update));
% 	self.global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	self = update_parameters( self, true );

	% hrt_abstime
    last_time = 0;

	% px4_pollfd_struct_t fds[1] = {};
	% fds[0].fd = self.sensors_sub;
	% fds[0].events = POLLIN;

	while ( ~self.task_should_exit ) 
		%int
        ret = px4_poll(fds, 1, 1000);

% 		if (ret < 0) 
% 			% Poll error, sleep and try again
% 			usleep(10000);
% 			% PX4_WARN('Q POLL ERROR');
% 			continue;
% 
% 		elseif (ret == 0) 
% 			% Poll timeout, do nothing
% 			% PX4_WARN('Q POLL TIMEOUT');
% 			continue;
% 		end

		self = update_parameters( self, false );

		% Update sensors
		% sensor_combined_s sensors ;

		% if (~orb_copy(ORB_ID(sensor_combined), self.sensors_sub, sensors))
        if sensor_combined.updated
			% Feed validator with recent sensor data
			if (sensors.timestamp > 0) 
				% Filter gyro signal since it is not fildered in the drivers.
				self.gyro(0) = self.lp_gyro_x.apply( sensors.gyro_rad(0) );
				self.gyro(1) = self.lp_gyro_y.apply( sensors.gyro_rad(1) );
				self.gyro(2) = self.lp_gyro_z.apply( sensors.gyro_rad(2) );
			end

			if ( sensors.accelerometer_timestamp_relative ~= sensor_combined_s.RELATIVE_TIMESTAMP_INVALID ) 
				% Filter accel signal since it is not fildered in the drivers.
				self.accel(0) = self.lp_accel_x.apply( sensors.accelerometer_m_s2(0) );
				self.accel(1) = self.lp_accel_y.apply( sensors.accelerometer_m_s2(1) );
				self.accel(2) = self.lp_accel_z.apply( sensors.accelerometer_m_s2(2) );

				if (self.accel.length() < 0.01) 
					% disp('WARNING: degenerate accel~');
					continue;
				end
			end

			if ( sensors.magnetometer_timestamp_relative ~= sensor_combined_s.RELATIVE_TIMESTAMP_INVALID ) 
				self.mag(0) = sensors.magnetometer_ga(0);
				self.mag(1) = sensors.magnetometer_ga(1);
				self.mag(2) = sensors.magnetometer_ga(2);

				if (self.mag.length() < 0.01) 
					% disp('WARNING: degenerate mag~');
					continue;
				end
			end

			self.data_good = true;
        end

		% Update vision and motion capture heading
        % bool
		vision_updated = self.vision_sub.updated;
		% orb_check(self.vision_sub, vision_updated);

		% bool
        mocap_updated = self.mocap_sub.updated;
		% orb_check(self.mocap_sub, mocap_updated);

		if (vision_updated) 
			% orb_copy(ORB_ID(vehicle_vision_attitude), self.vision_sub, &self.vision);
            self.vision = self.vision_sub;
			% math::Quaternion
            q = self.vision.q;

			% math::Matrix<3, 3>
            Rvis = q.to_dcm();
			% math::Vector<3>
            v = [1.0; 0.0; 0.4];

			% Rvis is Rwr (robot respect to world) while v is respect to world.
			% Hence Rvis must be transposed having (Rwr)' * Vw
			% Rrw * Vw = vn. This way we have consistency
			self.vision_hdg = Rvis' * v;
		end

		if (mocap_updated) 
			% orb_copy(ORB_ID(att_pos_mocap), self.mocap_sub, &self.mocap);
            self.mocap = self.mocap_sub;
			% math::Quaternion
            q = self.mocap.q;
			% math::Matrix<3, 3>
            Rmoc = q.to_dcm();

			% math::Vector<3>
            v = [1.0; 0.0; 0.4];

			% Rmoc is Rwr (robot respect to world) while v is respect to world.
			% Hence Rmoc must be transposed having (Rwr)' * Vw
			% Rrw * Vw = vn. This way we have consistency
			self.mocap_hdg = Rmoc' * v;
		end

		% Update airspeed
		% bool
        airspeed_updated = false;
		% orb_check(self.airspeed_sub, &airspeed_updated);

		if (airspeed_updated) 
			% orb_copy(ORB_ID(airspeed), self.airspeed_sub, &self.airspeed);
            self.airspeed = self.airspeed_sub;
		end

		% Check for timeouts on data
		if (self.ext_hdg_mode == 1) 
			self.ext_hdg_good = (self.vision.timestamp > 0) && ( hrt_elapsed_time(self.vision.timestamp) < 500000 );

		elseif (self.ext_hdg_mode == 2) 
			self.ext_hdg_good = self.mocap.timestamp > 0 && (hrt_elapsed_time(self.mocap.timestamp) < 500000);
		end

		% bool
        gpos_updated = false;
		% orb_check(self.global_pos_sub, &gpos_updated);

		if (gpos_updated) 
			%orb_copy(ORB_ID(vehicle_global_position), self.global_pos_sub, &self.gpos);
            self.gpos = self.global_pos_sub;

			if ( self.mag_decl_auto && (self.gpos.eph < 20.0) && hrt_elapsed_time(self.gpos.timestamp) < 1000000) 
				% set magnetic declination automatically
				update_mag_declination( ( get_mag_declination(self.gpos.lat, self.gpos.lon) ) * pi / 180 );
			end
		end

		if ( self.acc_comp && ...
             (self.gpos.timestamp ~= 0) && ...
             (hrt_absolute_time() < (self.gpos.timestamp + 20000)) && ...
             (self.gpos.eph < 5.0) && ...
             self.inited ) 
			% position data is actual
			if (gpos_updated) 
				% Vector<3>
                vel = [self.gpos.vel_n; self.gpos.vel_e; self.gpos.vel_d];

				% velocity updated
				if ( (self.vel_prev_t ~= 0) && (self.gpos.timestamp ~= self.vel_prev_t) )
					% float
                    vel_dt = (self.gpos.timestamp - self.vel_prev_t) / 1000000.0;
					% calculate acceleration in body frame
					self.pos_acc = self.q.conjugate_inversed((vel - self.vel_prev) / vel_dt); % DO THIS
				end

				self.vel_prev_t = self.gpos.timestamp;
				self.vel_prev = vel;
			end

		else 
			% position data is outdated, reset acceleration
			self.pos_acc.zero();
			self.vel_prev.zero();
			self.vel_prev_t = 0;
		end

		% time from previous iteration
		% hrt_abstime
        now = hrt_absolute_time();
		% float
        dt = (last_time > 0) ? ((now  - last_time) / 1000000.0) : 0.00001;
		last_time = now;

		if ( dt > self.dt_max ) 
			dt = self.dt_max;
		end

		if ( ~update(dt) ) 
			continue;
		end

		
			%vehicle_attitude_s
			att.timestamp = sensors.timestamp;
			att.rollspeed = self.rates(0);
			att.pitchspeed = self.rates(1);
			att.yawspeed = self.rates(2);
			att.q = [self.q(0); self.q(1); self.q(2); self.q(3)];

			% the instance count is not used here
			% int att_inst;
			% orb_publish_auto(ORB_ID(vehicle_attitude), &self.att_pub, &att, &att_inst, ORB_PRIO_HIGH);
            self.att_pub = att;
		
			% struct
            control_state_s ctrl_state = {};

			ctrl_state.timestamp = sensors.timestamp;

			% attitude quaternions for control state
			ctrl_state.q[0] = self.q(0);
			ctrl_state.q[1] = self.q(1);
			ctrl_state.q[2] = self.q(2);
			ctrl_state.q[3] = self.q(3);

			ctrl_state.x_acc = self.accel(0);
			ctrl_state.y_acc = self.accel(1);
			ctrl_state.z_acc = self.accel(2);

			% attitude rates for control state
			ctrl_state.roll_rate = self.rates(0);
			ctrl_state.pitch_rate = self.rates(1);
			ctrl_state.yaw_rate = self.rates(2);

			% TODO get bias estimates from estimator
			ctrl_state.roll_rate_bias = 0.0;
			ctrl_state.pitch_rate_bias = 0.0;
			ctrl_state.yaw_rate_bias = 0.0;

			ctrl_state.airspeed_valid = false;

			if (self.airspeed_mode == control_state_s.AIRSPD_MODE_MEAS) 
				% use measured airspeed
				if (isfinite(self.airspeed.indicated_airspeed_m_s) && ...
                        (hrt_absolute_time() - self.airspeed.timestamp < 1e6) && ...
                        (self.airspeed.timestamp > 0) ) 
					ctrl_state.airspeed = self.airspeed.indicated_airspeed_m_s;
					ctrl_state.airspeed_valid = true;
				end
			elseif (self.airspeed_mode == control_state_s.AIRSPD_MODE_EST) 
				% use estimated body velocity as airspeed estimate
				if (hrt_absolute_time() - self.gpos.timestamp < 1e6) 
					ctrl_state.airspeed = sqrtf(self.gpos.vel_n * self.gpos.vel_n + self.gpos.vel_e * self.gpos.vel_e + self.gpos.vel_d * self.gpos.vel_d);
					ctrl_state.airspeed_valid = true;
				end

			elseif (self.airspeed_mode == control_state_s.AIRSPD_MODE_DISABLED) 
				% do nothing, airspeed has been declared as non-valid above, controllers
				% will handle this assuming always trim airspeed
			end

			% the instance count is not used here
			% int ctrl_inst;
			% publish to control state topic
			orb_publish_auto(ORB_ID(control_state), &self.ctrl_state_pub, &ctrl_state, &ctrl_inst, ORB_PRIO_HIGH);
            self.ctrl_state_pub = ctrl_state;

			%struct estimator_status_s est = {};

			%est.timestamp = sensors.timestamp;

			% the instance count is not used here
			%int est_inst;
			% publish to control state topic
			% TODO handle attitude states in position estimators instead so we can publish all data at once
			% or we need to enable more thatn just one estimator_status topic
			% orb_publish_auto(ORB_ID(estimator_status), &self.est_state_pub, &est, &est_inst, ORB_PRIO_HIGH);
	end
% 
% #ifdef self._PX4_POSIX
% 	perf_end(self.perf_accel);
% 	perf_end(self.perf_mpu);
% 	perf_end(self.perf_mag);
% #endif
% 
% 	orb_unsubscribe(self.sensors_sub);
% 	orb_unsubscribe(self.vision_sub);
% 	orb_unsubscribe(self.mocap_sub);
% 	orb_unsubscribe(self.airspeed_sub);
% 	orb_unsubscribe(self.params_sub);
% 	orb_unsubscribe(self.global_pos_sub);
end

% void AttitudeEstimatorQ:: bool
function self = update_parameters( self,  force)

	% bool
    updated = force;

	if (~updated) 
		orb_check(self.params_sub, &updated);
	end

	if (updated) 
		parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), self.params_sub, &param_update);

		param_get(self.params_handles.w_acc, &self.w_accel);
		param_get(self.params_handles.w_mag, &self.w_mag);
		param_get(self.params_handles.w_ext_hdg, &self.w_ext_hdg);
		param_get(self.params_handles.w_gyro_bias, &self.w_gyro_bias);
		% float
        mag_decl_deg = 0.0;
		param_get(self.params_handles.mag_decl, &mag_decl_deg);
		update_mag_declination(math::radians(mag_decl_deg));
		% int32_t mag_decl_auto_int;
		param_get(self.params_handles.mag_decl_auto, &mag_decl_auto_int);
		self.mag_decl_auto = mag_decl_auto_int ~= 0;
		% int32_t acc_comp_int;
		param_get(self.params_handles.acc_comp, &acc_comp_int);
		self.acc_comp = acc_comp_int ~= 0;
		param_get(self.params_handles.bias_max, &self.bias_max);
		param_get(self.params_handles.ext_hdg_mode, &self.ext_hdg_mode);
		param_get(self.params_handles.airspeed_mode, &self.airspeed_mode);
	end
end

% bool AttitudeEstimatorQ::
function self = init( self )

	% Rotation matrix can be easily constructed from acceleration and mag field vectors
	% 'k' is Earth Z axis (Down) unit vector in body frame
	% Vector<3>
    k = -self.accel;
	k.normalize();

	% 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	% Vector<3>
    i = (self.mag - k * (self.mag * k));
	i.normalize();

	% 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	% Vector<3>
    j = k % i; % Fix this shit

	% Fill rotation matrix
	% Matrix<3, 3>
    R = zeros(3, 3);
	R(1,:) = i; %.set_row(0, i);
	R(2,:) = j; %.set_row(1, j);
	R(3,:) = k;% .set_row(2, k);

	% Convert to quaternion
	self.q.from_dcm(R);

	% Compensate for magnetic declination
	% Quaternion
    decl_rotation = decl_rotation.from_yaw(self.mag_decl);
	self.q = decl_rotation * self.q;

	self.q.normalize();

	if (isfinite(self.q(0)) && isfinite(self.q(1)) && ...
	    isfinite(self.q(2)) && isfinite(self.q(3)) && ...
	    (self.q.length() > 0.95) && (self.q.length() < 1.05) ) 
		self.inited = true;

	else 
		self.inited = false;
	end

	return self.inited;
end

% bool AttitudeEstimatorQ::
function [ self, output ] = update( self, float dt)

	if (~self.inited) 

		if (~self.data_good)
            output = false;
			return;
        end

        self = init( self );
		return;
	end

	% Quaternion
    q_last = self.q;

	% Angular rate of correction
	% Vector<3>
    corr  = zeros(3,1);
	% float
    spinRate = self.gyro.length();

	if ( (self.ext_hdg_mode > 0) && self.ext_hdg_good) 
		if (self.ext_hdg_mode == 1) 
			% Vision heading correction
			% Project heading to global frame and extract XY component
			% Vector<3>
            vision_hdg_earth = self.q.conjugate(self.vision_hdg);
			% float
            vision_hdg_err = self.wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
			% Project correction to body frame
			corr = corr + self.q.conjugate_inversed( [0.0; 0.0; -vision_hdg_err] ) * self.w_ext_hdg;
		end

		if (self.ext_hdg_mode == 2) 
			% Mocap heading correction
			% Project heading to global frame and extract XY component
			% Vector<3>
            mocap_hdg_earth = self.q.conjugate(self.mocap_hdg);
			% float
            mocap_hdg_err = self.wrap_pi(atan2(mocap_hdg_earth(1), mocap_hdg_earth(0)));
			% Project correction to body frame
			corr = corr + self.q.conjugate_inversed( [0.0, 0.0, -mocap_hdg_err] ) * self.w_ext_hdg;
		end
	end

	if ( (self.ext_hdg_mode == 0) || (~self.ext_hdg_good) )
		% Magnetometer correction
		% Project mag field vector to global frame and extract XY component
		% Vector<3>
        mag_earth = self.q.conjugate(self.mag);
		% float
        mag_err = self.wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - self.mag_decl);
		% float
        gainMult = 1.0;
		% const float
        fifty_dps = 0.873;

		if (spinRate > fifty_dps) 
			gainMult = min(spinRate / fifty_dps, 10.0);
		end

		% Project magnetometer correction to body frame
		corr = corr + self.q.conjugate_inversed( [0.0, 0.0, -mag_err] ) * self.w_mag * gainMult;
	end

	self.q.normalize();


	% Accelerometer correction
	% Project 'k' unit vector of earth frame to body frame
	% Vector<3> k = self.q.conjugate_inversed(Vector<3>(0.0, 0.0, 1.0));
	% Optimized version with dropped zeros
	% Vector<3>
    k = [ ...
		2.0 * (self.q(1) * self.q(3) - self.q(0) * self.q(2));
		2.0 * (self.q(2) * self.q(3) + self.q(0) * self.q(1));
		(self.q(0) * self.q(0) - self.q(1) * self.q(1) - self.q(2) * self.q(2) + self.q(3) * self.q(3)) ...
	];

	corr = corr + (k % (self.accel - self.pos_acc).normalized()) * self.w_accel;

	% Gyro bias estimation
	if (spinRate < 0.175) 
		self.gyro_bias = self.gyro_bias + corr * (self.w_gyro_bias * dt);

		% for (int i = 0; i < 3; i++) 
        for i = 1:3
			self.gyro_bias(i) = constrain(self.gyro_bias(i), -self.bias_max, self.bias_max); % CHECK
		end

	end

	self.rates = self.gyro + self.gyro_bias;

	% Feed forward gyro
	corr = corr + self.rates;

	% Apply correction to state
	self.q = self.q + self.q.derivative(corr) * dt;

	% Normalize quaternion
	self.q.normalize();

	if (~(isfinite(self.q(0)) && isfinite(self.q(1)) && ...
	      isfinite(self.q(2)) && isfinite(self.q(3)))) 
		% Reset quaternion to last good state
		self.q = q_last;
		self.rates.zero();
		self.gyro_bias.zero();
        output = false
		return;
    end
    
    output = true;
end

% void AttitudeEstimatorQ:: float 
function self = update_mag_declination( self, new_declination)

	% Apply initial declination or trivial rotations without changing estimation
	if (~self.inited || (abs(new_declination - self.mag_decl) < 0.0001) )
		self.mag_decl = new_declination;
	else 
		% Immediately rotate current estimation to avoid gyro bias growth
		% Quaternion
        decl_rotation = zeros(4,1);
		decl_rotation.from_yaw(new_declination - self.mag_decl);
		self.q = decl_rotation * self.q;
		self.mag_decl = new_declination;
	end
end

% int
% function self = attitude_estimator_q_main(int argc, char *argv[])
% 
% 	if (argc < 2)
% 		warnx('usage: attitude_estimator_q {start|stop|status}');
% 		return 1;
% 	end
% 
% 	if (~strcmp(argv[1], 'start')) 
% 
% 		if (attitude_estimator_q::instance ~= nullptr) 
% 			warnx('already running');
% 			return 1;
% 		end
% 
% 		attitude_estimator_q::instance = new AttitudeEstimatorQ;
% 
% 		if (attitude_estimator_q::instance == nullptr) 
% 			warnx('alloc failed');
% 			return 1;
% 		end
% 
% 		if (true ~= attitude_estimator_q::instance->start()) 
% 			delete attitude_estimator_q::instance;
% 			attitude_estimator_q::instance = nullptr;
% 			warnx('start failed');
% 			return 1;
% 		end
% 
% 		return 0;
% 	end
% 
% 	if (~strcmp(argv[1], 'stop')) 
% 		if (attitude_estimator_q::instance == nullptr) 
% 			warnx('not running');
% 			return 1;
% 		end
% 
% 		delete attitude_estimator_q::instance;
% 		attitude_estimator_q::instance = nullptr;
% 		return 0;
% 	end
% 
% 	if (~strcmp(argv[1], 'status')) 
% 		if (attitude_estimator_q::instance) 
% 			attitude_estimator_q::instance->print();
% 			warnx('running');
% 			return 0;
% 
% 		else 
% 			warnx('not running');
% 			return 1;
% 		end
% 	end
% 
% 	warnx('unrecognized command');
% 	return 1;
% end