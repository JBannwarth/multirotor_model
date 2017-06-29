% #include "BlockLocalPositionEstimator.hpp"
% #include <drivers/drv_hrt.h>
% #include <systemlib/mavlink_log.h>
% #include <fcntl.h>
% #include <systemlib/err.h>
% #include <matrix/math.hpp>
% #include <cstdlib>

% orb_advert_t mavlink_log_pub = nullptr;

% required standard deviation of estimate for estimator to publish data
static const uint32_t 		EST_STDDEV_XY_VALID = 2.0; % 2.0 m
static const uint32_t 		EST_STDDEV_Z_VALID = 2.0; % 2.0 m
static const uint32_t 		EST_STDDEV_TZ_VALID = 2.0; % 2.0 m

static const float P_MAX = 1.0e6; % max allowed value in state covariance
static const float LAND_RATE = 10.0; % rate of land detector correction

static const char *msg_label = '[lpe] ';  % rate of land detector correction

% BlockLocalPositionEstimator::
function BlockLocalPositionEstimator()
	%% START
	% % this block has no parent, and has name LPE
	% SuperBlock(nullptr, "LPE"),
	% % subscriptions, set rate, add to list
	% self.sub_armed(ORB_ID(actuator_armed), 1000 / 2, 0, &getSubscriptions()),
	% self.sub_land(ORB_ID(vehicle_land_detected), 1000 / 2, 0, &getSubscriptions()),
	% self.sub_att(ORB_ID(vehicle_attitude), 1000 / 100, 0, &getSubscriptions()),
	% % set flow max update rate higher than expected to we don't lose packets
	% self.sub_flow(ORB_ID(optical_flow), 1000 / 100, 0, &getSubscriptions()),
	% % main prediction loop, 100 hz
	% self.sub_sensor(ORB_ID(sensor_combined), 1000 / 100, 0, &getSubscriptions()),
	% % status updates 2 hz
	% self.sub_param_update(ORB_ID(parameter_update), 1000 / 2, 0, &getSubscriptions()),
	% self.sub_manual(ORB_ID(manual_control_setpoint), 1000 / 2, 0, &getSubscriptions()),
	% % gps 10 hz
	% self.sub_gps(ORB_ID(vehicle_gps_position), 1000 / 10, 0, &getSubscriptions()),
	% % vision 50 hz
	% self.sub_vision_pos(ORB_ID(vehicle_vision_position), 1000 / 50, 0, &getSubscriptions()),
	% % mocap 50 hz
	% self.sub_mocap(ORB_ID(att_pos_mocap), 1000 / 50, 0, &getSubscriptions()),
	% % all distance sensors, 10 hz
	% self.sub_dist0(ORB_ID(distance_sensor), 1000 / 10, 0, &getSubscriptions()),
	% self.sub_dist1(ORB_ID(distance_sensor), 1000 / 10, 1, &getSubscriptions()),
	% self.sub_dist2(ORB_ID(distance_sensor), 1000 / 10, 2, &getSubscriptions()),
	% self.sub_dist3(ORB_ID(distance_sensor), 1000 / 10, 3, &getSubscriptions()),
	% self.dist_subs(),
	% self.sub_lidar(nullptr),
	% self.sub_sonar(nullptr),

	% % publications
	% self.pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	% self.pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	% self.pub_est_status(ORB_ID(estimator_status), -1, &getPublications()),
	% self.pub_innov(ORB_ID(ekf2_innovations), -1, &getPublications()),

	% % map projection
	% self.map_ref(),

	% % block parameters
	% self.fusion(this, "FUSION"),
	% self.vxy_pub_thresh(this, "VXY_PUB"),
	% self.z_pub_thresh(this, "Z_PUB"),
	% self.sonar_z_stddev(this, "SNR_Z"),
	% self.sonar_z_offset(this, "SNR_OFF_Z"),
	% self.lidar_z_stddev(this, "LDR_Z"),
	% self.lidar_z_offset(this, "LDR_OFF_Z"),
	% self.accel_xy_stddev(this, "ACC_XY"),
	% self.accel_z_stddev(this, "ACC_Z"),
	% self.baro_stddev(this, "BAR_Z"),
	% self.gps_delay(this, "GPS_DELAY"),
	% self.gps_xy_stddev(this, "GPS_XY"),
	% self.gps_z_stddev(this, "GPS_Z"),
	% self.gps_vxy_stddev(this, "GPS_VXY"),
	% self.gps_vz_stddev(this, "GPS_VZ"),
	% self.gps_eph_max(this, "EPH_MAX"),
	% self.gps_epv_max(this, "EPV_MAX"),
	% self.vision_xy_stddev(this, "VIS_XY"),
	% self.vision_z_stddev(this, "VIS_Z"),
	% self.vision_delay(this, "VIS_DELAY"),
	% self.mocap_p_stddev(this, "VIC_P"),
	% self.flow_z_offset(this, "FLW_OFF_Z"),
	% self.flow_scale(this, "FLW_SCALE"),
	% //_flow_board_x_offs(NULL, "SENS_FLW_XOFF"),
	% //_flow_board_y_offs(NULL, "SENS_FLW_YOFF"),
	% self.flow_min_q(this, "FLW_QMIN"),
	% self.flow_r(this, "FLW_R"),
	% self.flow_rr(this, "FLW_RR"),
	% self.land_z_stddev(this, "LAND_Z"),
	% self.land_vxy_stddev(this, "LAND_VXY"),
	% self.pn_p_noise_density(this, "PN_P"),
	% self.pn_v_noise_density(this, "PN_V"),
	% self.pn_b_noise_density(this, "PN_B"),
	% self.pn_t_noise_density(this, "PN_T"),
	% self.t_max_grade(this, "T_MAX_GRADE"),

	% % init origin
	% self.fake_origin(this, "FAKE_ORIGIN"),
	% self.init_origin_lat(this, "LAT"),
	% self.init_origin_lon(this, "LON"),

	% % flow gyro
	% self.flow_gyro_x_high_pass(this, "FGYRO_HP"),
	% self.flow_gyro_y_high_pass(this, "FGYRO_HP"),

	% % stats
	% self.baroStats(this, ""),
	% self.sonarStats(this, ""),
	% self.lidarStats(this, ""),
	% self.flowQStats(this, ""),
	% self.visionStats(this, ""),
	% self.mocapStats(this, ""),
	% self.gpsStats(this, ""),

	% % low pass
	% self.xLowPass(this, "X_LP"),
	% % use same lp constant for agl
	% self.aglLowPass(this, "X_LP"),

	% % delay
	% self.xDelay(this, ""),
	% self.tDelay(this, ""),

	% % misc
	% self.polls(),
	% self.timeStamp(hrt_absolute_time()),
	% self.time_origin(0),
	% self.timeStampLastBaro(hrt_absolute_time()),
	% self.time_last_hist(0),
	% self.time_last_flow(0),
	% self.time_last_baro(0),
	% self.time_last_gps(0),
	% self.time_last_lidar(0),
	% self.time_last_sonar(0),
	% self.time_init_sonar(0),
	% self.time_last_vision_p(0),
	% self.time_last_mocap(0),
	% self.time_last_land(0),

	% % reference altitudes
	% self.altOrigin(0),
	% self.altOriginInitialized(false),
	% self.baroAltOrigin(0),
	% self.gpsAltOrigin(0),

	% % status
	% self.receivedGps(false),
	% self.lastArmedState(false),

	% % masks
	% self.sensorTimeout(255),
	% self.sensorFault(0),
	% self.estimatorInitialized(0)
	%% INIT END

	% assign distance subs to array
	self.dist_subs[0] = &self.sub_dist0;
	self.dist_subs[1] = &self.sub_dist1;
	self.dist_subs[2] = &self.sub_dist2;
	self.dist_subs[3] = &self.sub_dist3;

	% setup event triggering based on new flow messages to integrate
	self.polls[POLL_FLOW].fd = self.sub_flow.getHandle();
	self.polls[POLL_FLOW].events = POLLIN;

	self.polls[POLL_PARAM].fd = self.sub_param_update.getHandle();
	self.polls[POLL_PARAM].events = POLLIN;

	self.polls[POLL_SENSORS].fd = self.sub_sensor.getHandle();
	self.polls[POLL_SENSORS].events = POLLIN;

	% initialize A, B,  P, x, u
	self.x.setZero();
	self.u.setZero();
	initSS();

	% map
	self.map_ref.init_done = false;

	% intialize parameter dependent matrices
	updateParams();

	% print fusion settings to console
	printf([ '[lpe] fuse gps: %d, flow: %d, vis_pos: %d, ' ...
	       'vis_yaw: %d, land: %d, pub_agl_z: %d, flow_gyro: %d\n' ], ...
	       (self.fusion.get() & FUSE_GPS) ~= 0,       ...
	       (self.fusion.get() & FUSE_FLOW) ~= 0,      ...
	       (self.fusion.get() & FUSE_VIS_POS) ~= 0,   ...
	       (self.fusion.get() & FUSE_VIS_YAW) ~= 0,   ...
	       (self.fusion.get() & FUSE_LAND) ~= 0,      ...
	       (self.fusion.get() & FUSE_PUB_AGL_Z) ~= 0, ...
	       (self.fusion.get() & FUSE_FLOW_GYRO_COMP) ~= 0);
end

% BlockLocalPositionEstimator::~BlockLocalPositionEstimator()
% {
% }

%Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::
%float t
%const Vector<float, BlockLocalPositionEstimator::n_x> &x
%const Vector<float, BlockLocalPositionEstimator::n_u> &u
function output = dynamics( t, x, u)
	output = self.A * x + self.B * u;
end

% void BlockLocalPositionEstimator::
function update()
	% wait for a sensor update, check for exit condition every 100 ms int
	ret = px4_poll(self.polls, 3, 100);

	if (ret < 0)
		return;
	end

	%uint64_t
	newTimeStamp = hrt_absolute_time();
	%float
	dt = (newTimeStamp - self.timeStamp) / 1.0e6;
	self.timeStamp = newTimeStamp;

	% set dt for all child blocks
	setDt(dt);

	% auto-detect connected rangefinders while not armed bool
	armedState = self.sub_armed.get().armed;

	if (~armedState && (self.sub_lidar == nullptr || self.sub_sonar == nullptr))

		% detect distance sensors
		for i = 0:DIST_SUBS-1 %i < N_DIST_SUBS; i++)
			uORB::Subscription<distance_sensor_s> *s = self.dist_subs[i];

			if (s == self.sub_lidar || s == self.sub_sonar)
				continue;
			end

			if (s->updated())
				s->update();

				if (s->get().timestamp == 0)
					continue;
				end

				if (s->get().type == \ ...
				    distance_sensor_s::MAV_DISTANCE_SENSOR_LASER && ...
				    self.sub_lidar == nullptr)
					self.sub_lidar = s;
					mavlink_and_console_log_info(&mavlink_log_pub, '%sLidar detected with ID %i', msg_label, i);
				else if (s->get().type == \ ...
					   distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND && ...
					   self.sub_sonar == nullptr)
					self.sub_sonar = s;
					mavlink_and_console_log_info(&mavlink_log_pub, '%sSonar detected with ID %i', msg_label, i);
				end
			end
		end
	end

	% reset pos, vel, and terrain on arming

	% XXX this will be re-enabled for indoor use cases using a
	% selection param, but is really not helping outdoors
	% right now.

	% if (~self.lastArmedState && armedState) {

	% 	% we just armed, we are at origin on the ground
	% 	self.x(X_x) = 0;
	% 	self.x(X_y) = 0;
	% 	% reset Z or not? self.x(X_z) = 0;

	% 	% we aren't moving, all velocities are zero
	% 	self.x(X_vx) = 0;
	% 	self.x(X_vy) = 0;
	% 	self.x(X_vz) = 0;

	% 	% assume we are on the ground, so terrain alt is local alt
	% 	self.x(X_tz) = self.x(X_z);

	% 	% reset lowpass filter as well
	% 	self.xLowPass.setState(self.x);
	% 	self.aglLowPass.setState(0);
	% }

	self.lastArmedState = armedState;

	% see which updates are available all bool
	paramsUpdated = self.sub_param_update.updated();
	baroUpdated = false;

	if ((self.fusion.get() & FUSE_BARO) && self.sub_sensor.updated())
		int32_t baro_timestamp_relative = self.sub_sensor.get().baro_timestamp_relative;

		if (baro_timestamp_relative ~= self.sub_sensor.get().RELATIVE_TIMESTAMP_INVALID)
			uint64_t baro_timestamp = self.sub_sensor.get().timestamp + \
						  self.sub_sensor.get().baro_timestamp_relative;

			if (baro_timestamp ~= self.timeStampLastBaro)
				baroUpdated = true;
				self.timeStampLastBaro = baro_timestamp;
			end
		end
	end

	% All bool
	flowUpdated = (self.fusion.get() & FUSE_FLOW) && self.sub_flow.updated();
	gpsUpdated = (self.fusion.get() & FUSE_GPS) && self.sub_gps.updated();
	visionUpdated = (self.fusion.get() & FUSE_VIS_POS) && self.sub_vision_pos.updated();
	mocapUpdated = self.sub_mocap.updated();
	lidarUpdated = (self.sub_lidar ~= nullptr) && self.sub_lidar->updated();
	sonarUpdated = (self.sub_sonar ~= nullptr) && self.sub_sonar->updated();
	landUpdated = landed() ...
			   && ((self.timeStamp - self.time_last_land) > 1.0e6 / LAND_RATE); % throttle rate

	% get new data
	updateSubscriptions();

	% update parameters
	if (paramsUpdated)
		updateParams();
		updateSSParams();
	end

	% is xy valid? bool
	vxy_stddev_ok = false;

	if (max(self.P(X_vx, X_vx), self.P(X_vy, X_vy)) < self.vxy_pub_thresh.get() * self.vxy_pub_thresh.get())
		vxy_stddev_ok = true;
	end

	if (self.estimatorInitialized & EST_XY)
		% if valid and gps has timed out, set to not valid
		if (~vxy_stddev_ok && (self.sensorTimeout & SENSOR_GPS))
			self.estimatorInitialized &= ~EST_XY;
		end
	else
		if (vxy_stddev_ok)
			if (~(self.sensorTimeout & SENSOR_GPS)       ...
			    || ~(self.sensorTimeout & SENSOR_FLOW)   ...
			    || ~(self.sensorTimeout & SENSOR_VISION) ...
			    || ~(self.sensorTimeout & SENSOR_MOCAP)  ...
			    || ~(self.sensorTimeout & SENSOR_LAND)   ...
			   )
				self.estimatorInitialized |= EST_XY;
			end
		end
	end

	% is z valid? bool
	z_stddev_ok = sqrt(self.P(X_z, X_z)) < self.z_pub_thresh.get();

	if (self.estimatorInitialized & EST_Z)
		% if valid and baro has timed out, set to not valid
		if (~z_stddev_ok && (self.sensorTimeout & SENSOR_BARO))
			self.estimatorInitialized &= ~EST_Z;
		end

	else
		if (z_stddev_ok)
			self.estimatorInitialized |= EST_Z;
		end
	end

	% is terrain valid?
	tz_stddev_ok = sqrt(self.P(X_tz, X_tz)) < self.z_pub_thresh.get();

	if (self.estimatorInitialized & EST_TZ)
		if (~tz_stddev_ok)
			self.estimatorInitialized &= ~EST_TZ;
		end

	else
		if (tz_stddev_ok)
			self.estimatorInitialized |= EST_TZ;
		end
	end

	% check timeouts
	checkTimeouts();

	% if we have no lat, lon initialize projection to LPE_LAT, LPE_LON parameters
	if (~self.map_ref.init_done && (self.estimatorInitialized & EST_XY) && self.fake_origin.get())
		map_projection_init(&self.map_ref, ...
				    self.init_origin_lat.get(), ...
				    self.init_origin_lon.get());

		% set timestamp when origin was set to current time
		self.time_origin = self.timeStamp;

		mavlink_and_console_log_info(&mavlink_log_pub, '[lpe] global origin init (parameter) : lat %6.2f lon %6.2f alt %5.1f m', ...
					     double(self.init_origin_lat.get()), double(self.init_origin_lon.get()), double(self.altOrigin));

	end

	% reinitialize x if necessary bool
	reinit_x = false;

	for i = 0:n_x-1 %; i < n_x; i++)
		% should we do a reinit
		% of sensors here?
		% don't want it to take too long
		if (~PX4_ISFINITE(self.x(i)))
			reinit_x = true;
			mavlink_and_console_log_info(&mavlink_log_pub, '%sreinit x, x(%d) not finite', msg_label, i);
			break;
		end
	end

	if (reinit_x)
		for i = 0:n_x-1 %; i < n_x; i++)
			self.x(i) = 0;
		end

		mavlink_and_console_log_info(&mavlink_log_pub, '%sreinit x', msg_label);
	end

	% force P symmetry and reinitialize P if necessary bool
	reinit_P = false;

	for i = 0:(n_x-1) %; i < n_x; i++)
		for j = 0:i %; j <= i; j++)
			if (~PX4_ISFINITE(self.P(i, j)))
				mavlink_and_console_log_info(&mavlink_log_pub, ...
							     '%sreinit P (%d, %d) not finite', msg_label, i, j);
				reinit_P = true;
			end

			if (i == j)
				% make sure diagonal elements are positive
				if (self.P(i, i) <= 0)
					mavlink_and_console_log_info(&mavlink_log_pub, ...
								     '%sreinit P (%d, %d) negative', msg_label, i, j);
					reinit_P = true;
				end
			else
				% copy elememnt from upper triangle to force
				% symmetry
				self.P(j, i) = self.P(i, j);
			end

			if (reinit_P)
				break;
			end
		end

		if (reinit_P)
			break;
		end
	end

	if (reinit_P)
		initP();
	end

	% do prediction
	predict();

	% sensor corrections/ initializations
	if (gpsUpdated)
		if (self.sensorTimeout & SENSOR_GPS)
			gpsInit();
		else
			gpsCorrect();
		end
	end

	if (baroUpdated)
		if (self.sensorTimeout & SENSOR_BARO)
			baroInit();
		else
			baroCorrect();
		end
	end

	if (lidarUpdated)
		if (self.sensorTimeout & SENSOR_LIDAR)
			lidarInit();
		else
			lidarCorrect();
		end
	end

	if (sonarUpdated)
		if (self.sensorTimeout & SENSOR_SONAR)
			sonarInit();
		else
			sonarCorrect();
		end
	end

	if (flowUpdated)
		if (self.sensorTimeout & SENSOR_FLOW)
			flowInit();
		else
			flowCorrect();
		end
	end

	if (visionUpdated)
		if (self.sensorTimeout & SENSOR_VISION)
			visionInit();
		else
			visionCorrect();
		end
	end

	if (mocapUpdated)
		if (self.sensorTimeout & SENSOR_MOCAP)
			mocapInit();
		else
			mocapCorrect();
		end
	end

	if (landUpdated)
		if (self.sensorTimeout & SENSOR_LAND)
			landInit();
		else
			landCorrect();
		end
	end

	if (self.altOriginInitialized)
		% update all publications if possible
		publishLocalPos();
		publishEstimatorStatus();
		self.pub_innov.update();

		if ((self.estimatorInitialized & EST_XY) && (self.map_ref.init_done || self.fake_origin.get()))
			publishGlobalPos();
		end
	end

	% propagate delayed state, no matter what
	% if state is frozen, delayed state still
	% needs to be propagated with frozen state
	float dt_hist = 1.0e-6 * (self.timeStamp - self.time_last_hist);

	if (self.time_last_hist == 0 || ...
	    (dt_hist > HIST_STEP))
		self.tDelay.update(Scalar<uint64_t>(self.timeStamp));
		self.xDelay.update(self.x);
		self.time_last_hist = self.timeStamp;
	end
end

%void BlockLocalPositionEstimator::
function checkTimeouts()
	baroCheckTimeout();
	gpsCheckTimeout();
	lidarCheckTimeout();
	flowCheckTimeout();
	sonarCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
	landCheckTimeout();
end

% bool BlockLocalPositionEstimator::
function output = landed()
	if (~(self.fusion.get() & FUSE_LAND))
		% return false;
		output = false;
		return;
	end

	bool disarmed_not_falling = (~self.sub_armed.get().armed) && (~self.sub_land.get().freefall);

	output = self.sub_land.get().landed || disarmed_not_falling;
end

% void BlockLocalPositionEstimator::
function publishLocalPos()
	const Vector<float, n_x> &xLP = self.xLowPass.getState();

	% lie about eph/epv to allow visual odometry only navigation when velocity est. good
	% All float
	vxy_stddev = sqrt(self.P(X_vx, X_vx) + self.P(X_vy, X_vy));
	epv = sqrt(self.P(X_z, X_z));
	eph = sqrt(self.P(X_x, X_x) + self.P(X_y, X_y));
	eph_thresh = 3.0;
	epv_thresh = 3.0;

	if (vxy_stddev < self.vxy_pub_thresh.get())
		if (eph > eph_thresh)
			eph = eph_thresh;
		end

		if (epv > epv_thresh)
			epv = epv_thresh;
		end
	end

	% publish local position
	if (PX4_ISFINITE(self.x(X_x)) && PX4_ISFINITE(self.x(X_y)) && PX4_ISFINITE(self.x(X_z)) && ...
	    PX4_ISFINITE(self.x(X_vx)) && PX4_ISFINITE(self.x(X_vy)) ...
	    && PX4_ISFINITE(self.x(X_vz)))
		self.pub_lpos.get().timestamp = self.timeStamp;
		self.pub_lpos.get().xy_valid = self.estimatorInitialized & EST_XY;
		self.pub_lpos.get().z_valid = self.estimatorInitialized & EST_Z;
		self.pub_lpos.get().v_xy_valid = self.estimatorInitialized & EST_XY;
		self.pub_lpos.get().v_z_valid = self.estimatorInitialized & EST_Z;
		self.pub_lpos.get().x = xLP(X_x); 	% north
		self.pub_lpos.get().y = xLP(X_y);  	% east

		if (self.fusion.get() & FUSE_PUB_AGL_Z)
			self.pub_lpos.get().z = -self.aglLowPass.getState(); % agl

		else
			self.pub_lpos.get().z = xLP(X_z); 	% down
		end

		self.pub_lpos.get().vx = xLP(X_vx); % north
		self.pub_lpos.get().vy = xLP(X_vy); % east
		self.pub_lpos.get().vz = xLP(X_vz); % down

		% this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		self.pub_lpos.get().z_deriv = xLP(X_vz);

		self.pub_lpos.get().yaw = self.eul(2);
		self.pub_lpos.get().xy_global = self.estimatorInitialized & EST_XY;
		self.pub_lpos.get().z_global = ~(self.sensorTimeout & SENSOR_BARO);
		self.pub_lpos.get().ref_timestamp = self.time_origin;
		self.pub_lpos.get().ref_lat = self.map_ref.lat_rad * 180 / pi;
		self.pub_lpos.get().ref_lon = self.map_ref.lon_rad * 180 / pi;
		self.pub_lpos.get().ref_alt = self.altOrigin;
		self.pub_lpos.get().dist_bottom = self.aglLowPass.getState();
		self.pub_lpos.get().dist_bottom_rate = - xLP(X_vz);
		self.pub_lpos.get().surface_bottom_timestamp = self.timeStamp;
		% we estimate agl even when we don't have terrain info
		% if you are in terrain following mode this is important
		% so that if terrain estimation fails there isn't a
		% sudden altitude jump
		self.pub_lpos.get().dist_bottom_valid = self.estimatorInitialized & EST_Z;
		self.pub_lpos.get().eph = eph;
		self.pub_lpos.get().epv = epv;
		self.pub_lpos.update();
		% TODO provide calculated values for these
		self.pub_lpos.get().evh = 0.0;
		self.pub_lpos.get().evv = 0.0;
	end
end

% void BlockLocalPositionEstimator::
function publishEstimatorStatus()
	self.pub_est_status.get().timestamp = self.timeStamp;

	for i = 0:n_x-1 %; i < n_x; i++)
		self.pub_est_status.get().states[i] = self.x(i);
		self.pub_est_status.get().covariances[i] = self.P(i, i);
	end

	self.pub_est_status.get().n_states = n_x;
	self.pub_est_status.get().nan_flags = 0;
	self.pub_est_status.get().health_flags = self.sensorFault;
	self.pub_est_status.get().timeout_flags = self.sensorTimeout;
	self.pub_est_status.get().pos_horiz_accuracy = self.pub_gpos.get().eph;
	self.pub_est_status.get().pos_vert_accuracy = self.pub_gpos.get().epv;

	self.pub_est_status.update();
end

% void BlockLocalPositionEstimator::
function publishGlobalPos()
	% publish global position
	double lat = 0;
	double lon = 0;
	const Vector<float, n_x> &xLP = self.xLowPass.getState();
	map_projection_reproject(&self.map_ref, xLP(X_x), xLP(X_y), &lat, &lon);
	% float
	alt = -xLP(X_z) + self.altOrigin;

	% lie about eph/epv to allow visual odometry only navigation when velocity est. good all float
	vxy_stddev = sqrt(self.P(X_vx, X_vx) + self.P(X_vy, X_vy));
	epv = sqrt(self.P(X_z, X_z));
	eph = sqrt(self.P(X_x, X_x) + self.P(X_y, X_y));
	eph_thresh = 3.0;
	epv_thresh = 3.0;

	if (vxy_stddev < self.vxy_pub_thresh.get())
		if (eph > eph_thresh)
			eph = eph_thresh;
		end

		if (epv > epv_thresh)
			epv = epv_thresh;
		end
	end

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) && ...
	    PX4_ISFINITE(xLP(X_vx)) && PX4_ISFINITE(xLP(X_vy)) && ...
	    PX4_ISFINITE(xLP(X_vz)))
		self.pub_gpos.get().timestamp = self.timeStamp;
		self.pub_gpos.get().time_utc_usec = self.sub_gps.get().time_utc_usec;
		self.pub_gpos.get().lat = lat;
		self.pub_gpos.get().lon = lon;
		self.pub_gpos.get().alt = alt;
		self.pub_gpos.get().vel_n = xLP(X_vx);
		self.pub_gpos.get().vel_e = xLP(X_vy);
		self.pub_gpos.get().vel_d = xLP(X_vz);

		% this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		self.pub_gpos.get().pos_d_deriv = xLP(X_vz);

		self.pub_gpos.get().yaw = self.eul(2);
		self.pub_gpos.get().eph = eph;
		self.pub_gpos.get().epv = epv;
		self.pub_gpos.get().terrain_alt = self.altOrigin - xLP(X_tz);
		self.pub_gpos.get().terrain_alt_valid = self.estimatorInitialized & EST_TZ;
		self.pub_gpos.get().dead_reckoning = ~(self.estimatorInitialized & EST_XY);
		self.pub_gpos.get().pressure_alt = self.sub_sensor.get().baro_alt_meter;
		self.pub_gpos.update();
		% TODO provide calculated values for these
		self.pub_gpos.get().evh = 0.0;
		self.pub_gpos.get().evv = 0.0;
	end
end

% void BlockLocalPositionEstimator::
function initP()
	self.P.setZero();
	% initialize to twice valid condition
	self.P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	self.P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	self.P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	self.P(X_vx, X_vx) = 2 * self.vxy_pub_thresh.get() * self.vxy_pub_thresh.get();
	self.P(X_vy, X_vy) = 2 * self.vxy_pub_thresh.get() * self.vxy_pub_thresh.get();
	% use vxy thresh for vz init as well
	self.P(X_vz, X_vz) = 2 * self.vxy_pub_thresh.get() * self.vxy_pub_thresh.get();
	% initialize bias uncertainty to small values to keep them stable
	self.P(X_bx, X_bx) = 1e-6;
	self.P(X_by, X_by) = 1e-6;
	self.P(X_bz, X_bz) = 1e-6;
	self.P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
end

%void BlockLocalPositionEstimator::
function initSS()
	initP();

	% dynamics matrix
	self.A.setZero();
	% derivative of position is velocity
	self.A(X_x, X_vx) = 1;
	self.A(X_y, X_vy) = 1;
	self.A(X_z, X_vz) = 1;

	% input matrix
	self.B.setZero();
	self.B(X_vx, U_ax) = 1;
	self.B(X_vy, U_ay) = 1;
	self.B(X_vz, U_az) = 1;

	% update components that depend on current state
	updateSSStates();
	updateSSParams();
end

%void BlockLocalPositionEstimator::
function updateSSStates()
	% derivative of velocity is accelerometer acceleration
	% (in input matrix) - bias (in body frame)
	self.A(X_vx, X_bx) = -self.R_att(0, 0);
	self.A(X_vx, X_by) = -self.R_att(0, 1);
	self.A(X_vx, X_bz) = -self.R_att(0, 2);

	self.A(X_vy, X_bx) = -self.R_att(1, 0);
	self.A(X_vy, X_by) = -self.R_att(1, 1);
	self.A(X_vy, X_bz) = -self.R_att(1, 2);

	self.A(X_vz, X_bx) = -self.R_att(2, 0);
	self.A(X_vz, X_by) = -self.R_att(2, 1);
	self.A(X_vz, X_bz) = -self.R_att(2, 2);
end

%void BlockLocalPositionEstimator::
function updateSSParams()
	% input noise covariance matrix
	self.R.setZero();
	self.R(U_ax, U_ax) = self.accel_xy_stddev.get() * self.accel_xy_stddev.get();
	self.R(U_ay, U_ay) = self.accel_xy_stddev.get() * self.accel_xy_stddev.get();
	self.R(U_az, U_az) = self.accel_z_stddev.get() * self.accel_z_stddev.get();

	% process noise power matrix
	self.Q.setZero();
	%float
	pn_p_sq = self.pn_p_noise_density.get() * self.pn_p_noise_density.get();
	%float
	pn_v_sq = self.pn_v_noise_density.get() * self.pn_v_noise_density.get();
	self.Q(X_x, X_x) = pn_p_sq;
	self.Q(X_y, X_y) = pn_p_sq;
	self.Q(X_z, X_z) = pn_p_sq;
	self.Q(X_vx, X_vx) = pn_v_sq;
	self.Q(X_vy, X_vy) = pn_v_sq;
	self.Q(X_vz, X_vz) = pn_v_sq;

	% technically, the noise is in the body frame,
	% but the components are all the same, so
	% ignoring for now
	% float
	pn_b_sq = self.pn_b_noise_density.get() * self.pn_b_noise_density.get();
	self.Q(X_bx, X_bx) = pn_b_sq;
	self.Q(X_by, X_by) = pn_b_sq;
	self.Q(X_bz, X_bz) = pn_b_sq;

	% terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	%float
	pn_t_noise_density = ...
		self.pn_t_noise_density.get() + ...
		(self.t_max_grade.get() / 100.0) * sqrt(self.x(X_vx) * self.x(X_vx) + self.x(X_vy) * self.x(X_vy));
	self.Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;

end

% void BlockLocalPositionEstimator::
function predict()
	% get acceleration
	matrix::Quaternion<float> q(&self.sub_att.get().q[0]);
	self.eul = matrix::Euler<float>(q);
	self.R_att = matrix::Dcm<float>(q);
	Vector3f a(self.sub_sensor.get().accelerometer_m_s2);
	% note, bias is removed in dynamics function
	self.u = self.R_att * a;
	self.u(U_az) += 9.81; % add g

	% update state space based on new states
	updateSSStates();

	% continuous time kalman filter prediction
	% integrate runge kutta 4th order
	% TODO move rk4 algorithm to matrixlib
	% https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	h = getDt(); % float
	Vector<float, n_x> k1, k2, k3, k4;
	k1 = dynamics(0, self.x, self.u);
	k2 = dynamics(h / 2, self.x + k1 * h / 2, self.u);
	k3 = dynamics(h / 2, self.x + k2 * h / 2, self.u);
	k4 = dynamics(h, self.x + k3 * h, self.u);
	
	% Vector<float, n_x>
	dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	% don't integrate position if no valid xy data
	if (~(self.estimatorInitialized & EST_XY))
		dx(X_x) = 0;
		dx(X_vx) = 0;
		dx(X_y) = 0;
		dx(X_vy) = 0;
	end

	% don't integrate z if no valid z data
	if (~(self.estimatorInitialized & EST_Z))
		dx(X_z) = 0;
	end

	% don't integrate tz if no valid tz data
	if (~(self.estimatorInitialized & EST_TZ))
		dx(X_tz) = 0;
	end

	% saturate bias all float
	bx = dx(X_bx) + self.x(X_bx);
	by = dx(X_by) + self.x(X_by);
	bz = dx(X_bz) + self.x(X_bz);

	if (abs(bx) > BIAS_MAX)
		bx = BIAS_MAX * bx / abs(bx);
		dx(X_bx) = bx - self.x(X_bx);
	end

	if (abs(by) > BIAS_MAX)
		by = BIAS_MAX * by / abs(by);
		dx(X_by) = by - self.x(X_by);
	end

	if (abs(bz) > BIAS_MAX)
		bz = BIAS_MAX * bz / abs(bz);
		dx(X_bz) = bz - self.x(X_bz);
	end

	% propagate
	self.x = self.x + dx;
	Matrix<float, n_x, n_x> dP = (self.A * self.P + self.P * self.A.transpose() +
				      self.B * self.R * self.B.transpose() + self.Q) * getDt();

	% covariance propagation logic
	for i = 0:(n_x-1) %; i < n_x; i++)
		if (self.P(i, i) > P_MAX)
			% if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for j = 0:(n_x-1) %; j < n_x; j++)
				dP(i, j) = 0;
				dP(j, i) = 0;
			end
		end
	end

	self.P = self.P + dP;
	self.xLowPass.update(self.x);
	self.aglLowPass.update(agl());
end

% int BlockLocalPositionEstimator::
function output = getDelayPeriods(float delay, uint8_t *periods)
	%float
	t_delay = 0;
	%uint8_t
	i_hist = 0;

	for i_hist = 1:HIST_LEN-1 %; i_hist < HIST_LEN; i_hist++) {
		t_delay = 1.0e-6 * (self.timeStamp - self.tDelay.get(i_hist)(0, 0));

		if (t_delay > delay)
			break;
		end
	end

	*periods = i_hist;

	if (t_delay > DELAY_MAX)
		mavlink_and_console_log_info(&mavlink_log_pub, '%sdelayed data old: %8.4f', msg_label, double(t_delay));
		output = -1;
		return;
	end

	output = OK;
end