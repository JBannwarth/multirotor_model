function selfVector = LocalPositionEstimator( selfVector, timestamp )

	% required standard deviation of estimate for estimator to publish data
	% UINT32
	self.EST_STDDEV_XY_VALID = 2.0; % 2.0 m
	self.EST_STDDEV_Z_VALID = 2.0; % 2.0 m
	self.EST_STDDEV_TZ_VALID = 2.0; % 2.0 m

	% float
	self.P_MAX = 1.0e6; % max allowed value in state covariance
	self.LAND_RATE = 10.0; % rate of land detector correction

	% char
	self.msg_label = '(lpe) ';  % rate of land detector correction

	self.EST_XY = 2^0;
	self.EST_Z  = 2^1;
	self.EST_TZ = 2^2;

	self.BIAS_MAX = 1e-1;

	self.FUSE_GPS            = 2^0;
	self.FUSE_FLOW           = 2^1;
	self.FUSE_VIS_POS        = 2^2;
	self.FUSE_VIS_YAW        = 2^3;
	self.FUSE_LAND           = 2^4;
	self.FUSE_PUB_AGL_Z      = 2^5;
	self.FUSE_FLOW_GYRO_COMP = 2^6;
	self.FUSE_BARO           = 2^7;

	self.SENSOR_BARO   = 2^0;
	self.SENSOR_GPS    = 2^1;
	self.SENSOR_LIDAR  = 2^2;
	self.SENSOR_FLOW   = 2^3;
	self.SENSOR_SONAR  = 2^4;
	self.SENSOR_VISION = 2^5;
	self.SENSOR_MOCAP  = 2^6;
	self.SENSOR_LAND   = 2^7;

	self.BETA_TABLE = [ 0, ...
				        8.82050518214, ...
				        12.094592431,  ...
				        13.9876612368, ...
				        16.0875642296, ...
				        17.8797700658, ...
				        19.6465647819 ];

end

function self = DecodeSelfVector( selfVector )
%DECODESELFVECTOR Convert input vector to structure
end

function selfVector = EncodeSelfVector( self )
%ENCODESELFVECTOR Convert structure to vector
end

% #include "BlockLocalPositionEstimator.hpp"
% #include <drivers/drv_hrt.h>
% #include <systemlib/mavlink_log.h>
% #include <fcntl.h>
% #include <systemlib/err.h>
% #include <matrix/math.hpp>
% #include <cstdlib>

% orb_advert_t mavlink_log_pub = nullptr;

% NEED TO TAKE CARE OF INITIALIZATION
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
	% %_flow_board_x_offs(NULL, "SENS_FLW_XOFF"),
	% %_flow_board_y_offs(NULL, "SENS_FLW_YOFF"),
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
	self.dist_subs(0) = self.sub_dist0;
	self.dist_subs(1) = self.sub_dist1;
	self.dist_subs(2) = self.sub_dist2;
	self.dist_subs(3) = self.sub_dist3;

	% setup event triggering based on new flow messages to integrate
	% self.polls(POLL_FLOW).fd = self.sub_flow.getHandle();
	% self.polls(POLL_FLOW).events = POLLIN;

	% self.polls(POLL_PARAM).fd = self.sub_param_update.getHandle();
	% self.polls(POLL_PARAM).events = POLLIN;

	% self.polls(POLL_SENSORS).fd = self.sub_sensor.getHandle();
	% self.polls(POLL_SENSORS).events = POLLIN;

	% initialize A, B,  P, x, u
	n_x = 10; n_u = 3;
	self.x = zeros(n_x, 1);%.setZero();
	self.u = zeros(n_u,1);
	self = initSS( self );

	% map
	self.map_ref.init_done = false;

	% intialize parameter dependent matrices
	self = updateParams( self );

	% print fusion settings to console
	% printf(( ['(lpe) fuse gps: %d, flow: %d, vis_pos: %d, ' ...
	%        'vis_yaw: %d, land: %d, pub_agl_z: %d, flow_gyro: %d\n'] ), ...
	%        bitand( self.fusion, self.FUSE_GPS)             ~= 0, ...
	%        bitand( self.fusion, self.FUSE_FLOW )           ~= 0, ...
	%        bitand( self.fusion, self.FUSE_VIS_POS )        ~= 0, ...
	%        bitand( self.fusion, self.FUSE_VIS_YAW )        ~= 0, ...
	%        bitand( self.fusion, self.FUSE_LAND )           ~= 0, ...
	%        bitand( self.fusion, self.FUSE_PUB_AGL_Z )      ~= 0, ...
	%        bitand( self.fusion, self.FUSE_FLOW_GYRO_COMP ) ~= 0 );
end

% BlockLocalPositionEstimator::~BlockLocalPositionEstimator()
% {
% }

%Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::
%float t
%const Vector<float, BlockLocalPositionEstimator::n_x> &x
%const Vector<float, BlockLocalPositionEstimator::n_u> &u
function output = dynamics( t, x, u )
	output = self.A * x + self.B * u;
end

% NEED TO TAKE CARE OF SENSOR UPDATE
% NEED TO TAKE CARE OF nullptr
% NEED TO TAKE CARE OF updateSubscriptions
% NEED TO TAKE CARE OF updateParams()
% self.pub_innov.update( self );
% void BlockLocalPositionEstimator::
function self = update( self, timestamp )
%UPDATE

	% Probably bypass all this stuff
	% wait for a sensor update, check for exit condition every 100 ms int
	% ret = px4_poll(self.polls, 3, 100);

	% if (ret < 0)
	% 	return;
	% end

	%uint64_t
	newTimeStamp = timestamp;
	%float
	dt = (newTimeStamp - self.timeStamp) / 1.0e6;
	self.timeStamp = newTimeStamp;

	% set dt for all child blocks
	self = setDt( self, dt );

	% auto-detect connected rangefinders while not armed bool - maybe bypass?
	armedState = self.sub_armed.armed;

	if (~armedState && (self.sub_lidar == nullptr || self.sub_sonar == nullptr))

		% detect distance sensors
		for i = 1:DIST_SUBS %i < N_DIST_SUBS; i++)
			%uORB::Subscription<distance_sensor_s> *s = self.dist_subs(i);
			s = self.dist_subs(i);

			if (s == self.sub_lidar || s == self.sub_sonar)
				continue;
			end

			if ( s.updated )
				s->update();

				if (s.timestamp == 0)
					continue;
				end

				if (s.type == MAV_DISTANCE_SENSOR_LASER && ...
				    self.sub_lidar == nullptr)
					self.sub_lidar = s;
					% mavlink_and_console_log_info(&mavlink_log_pub, '%sLidar detected with ID %i', self.msg_label, i);
				else if (s.type == MAV_DISTANCE_SENSOR_ULTRASOUND && ...
					   self.sub_sonar == nullptr)
					self.sub_sonar = s;
					% mavlink_and_console_log_info(&mavlink_log_pub, '%sSonar detected with ID %i', self.msg_label, i);
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
	paramsUpdated = self.sub_param_update.updated;
	baroUpdated = false;

	if ( bitand(self.fusion, self.FUSE_BARO) && self.sub_sensor.updated )
		% int32_t
		baro_timestamp_relative = self.sub_sensor.baro_timestamp_relative;

		if ( baro_timestamp_relative ~= self.sub_sensor.RELATIVE_TIMESTAMP_INVALID )
			% uint64_t 
			baro_timestamp = self.sub_sensor.timestamp + ...
						  self.sub_sensor.baro_timestamp_relative;

			if (baro_timestamp ~= self.timeStampLastBaro)
				baroUpdated = true;
				self.timeStampLastBaro = baro_timestamp;
			end
		end
	end

	% All bool
	flowUpdated   = bitand(self.fusion, self.FUSE_FLOW) && self.sub_flow.updated();
	gpsUpdated    = bitand(self.fusion, self.FUSE_GPS) && self.sub_gps.updated();
	visionUpdated = bitand(self.fusion, self.FUSE_VIS_POS) && self.sub_vision_pos.updated();
	mocapUpdated  = self.sub_mocap.updated();
	lidarUpdated  = (self.sub_lidar ~= nullptr) && self.sub_lidar->updated();
	sonarUpdated  = (self.sub_sonar ~= nullptr) && self.sub_sonar->updated();
	landUpdated   = landed() ...
			   && ((self.timeStamp - self.time_last_land) > 1.0e6 / self.LAND_RATE); % throttle rate

	% get new data - maybe not necessary
	self = updateSubscriptions( self );

	% update parameters
	if (paramsUpdated)
		self = updateParams( self );
		self = updateSSParams( self );
	end

	% is xy valid? bool
	vxy_stddev_ok = false;

	if (max(self.P(X_vx, X_vx), self.P(X_vy, X_vy)) < (self.vxy_pub_thresh * self.vxy_pub_thresh))
		vxy_stddev_ok = true;
	end

	if bitand(self.estimatorInitialized, self.EST_XY)
		% if valid and gps has timed out, set to not valid
		if (~vxy_stddev_ok && bitand(self.sensorTimeout, self.SENSOR_GPS))
			self.estimatorInitialized = bitand(self.estimatorInitialized, ~self.EST_XY );
		end
	else
		if (vxy_stddev_ok)
			if (   ~bitand(self.sensorTimeout, self.SENSOR_GPS)       ...
			    || ~bitand(self.sensorTimeout, self.SENSOR_FLOW)   ...
			    || ~bitand(self.sensorTimeout, self.SENSOR_VISION) ...
			    || ~bitand(self.sensorTimeout, self.SENSOR_MOCAP)  ...
			    || ~bitand(self.sensorTimeout, self.SENSOR_LAND)   ...
			   )
				self.estimatorInitialized = bitor( self.estimatorInitialized, self.EST_XY );
			end
		end
	end

	% is z valid? bool
	z_stddev_ok = sqrt(self.P(X_z, X_z)) < self.z_pub_thresh;

	if bitand(self.estimatorInitialized, self.EST_Z)
		% if valid and baro has timed out, set to not valid
		if (~z_stddev_ok && bitand(self.sensorTimeout, self.SENSOR_BARO))
			self.estimatorInitialized = bitand(self.estimatorInitialized, ~self.EST_Z);
		end

	else
		if (z_stddev_ok)
			self.estimatorInitialized = bitor( self.estimatorInitialized, self.EST_Z );
		end
	end

	% is terrain valid?
	tz_stddev_ok = sqrt(self.P(X_tz, X_tz)) < self.z_pub_thresh;

	if bitand(self.estimatorInitialized, self.EST_TZ)
		if (~tz_stddev_ok)
			self.estimatorInitialized = (self.estimatorInitialized, ~self.EST_TZ);
		end

	else
		if (tz_stddev_ok)
			self.estimatorInitialized = bitor( self.estimatorInitialized, self.EST_TZ );
		end
	end

	% check timeouts
	checkTimeouts();

	% if we have no lat, lon initialize projection to LPE_LAT, LPE_LON parameters
	if (~self.map_ref.init_done && bitand(self.estimatorInitialized, self.EST_XY) && self.fake_origin)
		map_projection_init(&self.map_ref, ...
				    self.init_origin_lat, ...
				    self.init_origin_lon);

		% set timestamp when origin was set to current time
		self.time_origin = self.timeStamp;

		mavlink_and_console_log_info(&mavlink_log_pub, '(lpe) global origin init (parameter) : lat %6.2f lon %6.2f alt %5.1f m', ...
					     double(self.init_origin_lat), double(self.init_origin_lon), double(self.altOrigin));

	end

	% reinitialize x if necessary bool
	reinit_x = false;

	for i = 0:n_x-1 %; i < n_x; i++)
		% should we do a reinit
		% of sensors here?
		% don't want it to take too long
		if (~isfinite(self.x(i)))
			reinit_x = true;
			mavlink_and_console_log_info(&mavlink_log_pub, '%sreinit x, x(%d) not finite', self.msg_label, i);
			break;
		end
	end

	if (reinit_x)
		for i = 0:n_x-1 %; i < n_x; i++)
			self.x(i) = 0;
		end

		mavlink_and_console_log_info(&mavlink_log_pub, '%sreinit x', self.msg_label);
	end

	% force P symmetry and reinitialize P if necessary bool
	reinit_P = false;

	for i = 0:(n_x-1) %; i < n_x; i++)
		for j = 0:i %; j <= i; j++)
			if (~isfinite(self.P(i, j)))
				mavlink_and_console_log_info(&mavlink_log_pub, ...
							     '%sreinit P (%d, %d) not finite', self.msg_label, i, j);
				reinit_P = true;
			end

			if (i == j)
				% make sure diagonal elements are positive
				if (self.P(i, i) <= 0)
					mavlink_and_console_log_info(&mavlink_log_pub, ...
								     '%sreinit P (%d, %d) negative', self.msg_label, i, j);
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
		if bitand( self.sensorTimeout, self.SENSOR_GPS )
			gpsInit();
		else
			gpsCorrect();
		end
	end

	if (baroUpdated)
		if bitand( self.sensorTimeout, self.SENSOR_BARO )
			baroInit();
		else
			baroCorrect();
		end
	end

	if (lidarUpdated)
		if bitand( self.sensorTimeout, self.SENSOR_LIDAR )
			lidarInit();
		else
			lidarCorrect();
		end
	end

	if (sonarUpdated)
		if bitand( self.sensorTimeout, self.SENSOR_SONAR )
			sonarInit();
		else
			sonarCorrect();
		end
	end

	if (flowUpdated)
		if bitand( self.sensorTimeout, self.SENSOR_FLOW )
			flowInit();
		else
			flowCorrect();
		end
	end

	if (visionUpdated)
		if bitand( self.sensorTimeout, self.SENSOR_VISION )
			visionInit();
		else
			visionCorrect();
		end
	end

	if (mocapUpdated)
		if bitand( self.sensorTimeout, self.SENSOR_MOCAP )
			mocapInit();
		else
			mocapCorrect();
		end
	end

	if (landUpdated)
		if bitand( self.sensorTimeout, self.SENSOR_LAND )
			landInit();
		else
			landCorrect();
		end
	end

	if (self.altOriginInitialized)
		% update all publications if possible
		self = publishLocalPos( self );
		self = publishEstimatorStatus( self );
		self.pub_innov.update( self );

		if ( bitand(self.estimatorInitialized, self.EST_XY) && (self.map_ref.init_done || self.fake_origin))
			self = publishGlobalPos( self );
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

% Add all these functions
%void BlockLocalPositionEstimator::
function self = checkTimeouts( self )
%CHECKTIMEOUTS Check sensor timeouts
	self = baroCheckTimeout( self );
	self = gpsCheckTimeout( self );
	self = lidarCheckTimeout( self );
	self = flowCheckTimeout( self );
	self = sonarCheckTimeout( self );
	self = visionCheckTimeout( self );
	self = mocapCheckTimeout( self );
	self = landCheckTimeout( self );
end

% bool BlockLocalPositionEstimator::
function output = landed( self )
%LANDED
	if (~bitand(self.fusion, self.FUSE_LAND))
		% return false;
		output = false;
		return;
	end

	disarmed_not_falling = (~self.sub_armed.armed) && (~self.sub_land.freefall);

	output = self.sub_land.landed || disarmed_not_falling;
end

% void BlockLocalPositionEstimator::
function self = publishLocalPos( self )
%PUBLISHLOCALPOS
	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10; n_x = 10;

	%const Vector<float, n_x> &
	xLP = self.xLowPass.state;

	% lie about eph/epv to allow visual odometry only navigation when velocity est. good
	% All float
	vxy_stddev = sqrt(self.P(X_vx, X_vx) + self.P(X_vy, X_vy));
	epv = sqrt(self.P(X_z, X_z));
	eph = sqrt(self.P(X_x, X_x) + self.P(X_y, X_y));
	eph_thresh = 3.0;
	epv_thresh = 3.0;

	if (vxy_stddev < self.vxy_pub_thresh)
		if (eph > eph_thresh)
			eph = eph_thresh;
		end

		if (epv > epv_thresh)
			epv = epv_thresh;
		end
	end

	% publish local position
	if (isfinite(self.x(X_x)) && isfinite(self.x(X_y)) && isfinite(self.x(X_z)) && ...
	    isfinite(self.x(X_vx)) && isfinite(self.x(X_vy)) ...
	    && isfinite(self.x(X_vz)))
		self.pub_lpos.timestamp  = self.timeStamp;
		self.pub_lpos.xy_valid   = bitand( self.estimatorInitialized, self.EST_XY );
		self.pub_lpos.z_valid    = bitand( self.estimatorInitialized, self.EST_Z );
		self.pub_lpos.v_xy_valid = bitand( self.estimatorInitialized, self.EST_XY );
		self.pub_lpos.v_z_valid  = bitand( self.estimatorInitialized, self.EST_Z );
		self.pub_lpos.x = xLP(X_x); 	% north
		self.pub_lpos.y = xLP(X_y);  	% east

		if bitand(self.fusion, self.FUSE_PUB_AGL_Z)
			self.pub_lpos.z = -self.aglLowPass.state; % agl

		else
			self.pub_lpos.z = xLP(X_z); 	% down
		end

		self.pub_lpos.vx = xLP(X_vx); % north
		self.pub_lpos.vy = xLP(X_vy); % east
		self.pub_lpos.vz = xLP(X_vz); % down

		% this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		self.pub_lpos.z_deriv = xLP(X_vz);

		self.pub_lpos.yaw = self.eul(3);
		self.pub_lpos.xy_global = bitand(self.estimatorInitialized, self.EST_XY);
		self.pub_lpos.z_global = ~bitand(self.sensorTimeout, self.SENSOR_BARO);
		self.pub_lpos.ref_timestamp = self.time_origin;
		self.pub_lpos.ref_lat = self.map_ref.lat_rad * 180 / pi;
		self.pub_lpos.ref_lon = self.map_ref.lon_rad * 180 / pi;
		self.pub_lpos.ref_alt = self.altOrigin;
		self.pub_lpos.dist_bottom = self.aglLowPass.state;
		self.pub_lpos.dist_bottom_rate = - xLP(X_vz);
		self.pub_lpos.surface_bottom_timestamp = self.timeStamp;
		% we estimate agl even when we don't have terrain info
		% if you are in terrain following mode this is important
		% so that if terrain estimation fails there isn't a
		% sudden altitude jump
		self.pub_lpos.dist_bottom_valid = bitand(self.estimatorInitialized, self.EST_Z);
		self.pub_lpos.eph = eph;
		self.pub_lpos.epv = epv;
		% self.pub_lpos.update();
		% TODO provide calculated values for these
		self.pub_lpos.evh = 0.0;
		self.pub_lpos.evv = 0.0;
	end
end

% void BlockLocalPositionEstimator::
function self = publishEstimatorStatus( self )
%PUBLISHESTIMATORSTATUS
	n_x = 10;
	self.pub_est_status.timestamp = self.timeStamp;

	for i = 1:n_x %; i < n_x; i++)
		self.pub_est_status.states(i) = self.x(i);
		self.pub_est_status.covariances(i) = self.P(i, i);
	end

	self.pub_est_status.n_states = n_x;
	self.pub_est_status.nan_flags = 0;
	self.pub_est_status.health_flags = self.sensorFault;
	self.pub_est_status.timeout_flags = self.sensorTimeout;
	self.pub_est_status.pos_horiz_accuracy = self.pub_gpos.eph;
	self.pub_est_status.pos_vert_accuracy = self.pub_gpos.epv;

	% self.pub_est_status.update();
end

% void BlockLocalPositionEstimator::
function self = publishGlobalPos( self )
%PUBLISHGLOBALPOS
	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10; n_x = 10;

	% publish global position double
	lat = 0;
	lon = 0;
	% const Vector<float, n_x> &
	xLP = self.xLowPass.state;
	[ ~, lat, lon ] = map_projection_reproject(self.map_ref, xLP(X_x), xLP(X_y), lat, lon);
	% float
	alt = -xLP(X_z) + self.altOrigin;

	% lie about eph/epv to allow visual odometry only navigation when velocity est. good all float
	vxy_stddev = sqrt(self.P(X_vx, X_vx) + self.P(X_vy, X_vy));
	epv = sqrt(self.P(X_z, X_z));
	eph = sqrt(self.P(X_x, X_x) + self.P(X_y, X_y));
	eph_thresh = 3.0;
	epv_thresh = 3.0;

	if (vxy_stddev < self.vxy_pub_thresh)
		if (eph > eph_thresh)
			eph = eph_thresh;
		end

		if (epv > epv_thresh)
			epv = epv_thresh;
		end
	end

	if (isfinite(lat) && isfinite(lon) && isfinite(alt) && ...
	    isfinite(xLP(X_vx)) && isfinite(xLP(X_vy)) && ...
	    isfinite(xLP(X_vz)))
		self.pub_gpos.timestamp = self.timeStamp;
		self.pub_gpos.time_utc_usec = self.sub_gps.time_utc_usec;
		self.pub_gpos.lat = lat;
		self.pub_gpos.lon = lon;
		self.pub_gpos.alt = alt;
		self.pub_gpos.vel_n = xLP(X_vx);
		self.pub_gpos.vel_e = xLP(X_vy);
		self.pub_gpos.vel_d = xLP(X_vz);

		% this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		self.pub_gpos.pos_d_deriv = xLP(X_vz);

		self.pub_gpos.yaw = self.eul(3);
		self.pub_gpos.eph = eph;
		self.pub_gpos.epv = epv;
		self.pub_gpos.terrain_alt = self.altOrigin - xLP(X_tz);
		self.pub_gpos.terrain_alt_valid = bitand(self.estimatorInitialized, self.EST_TZ);
		self.pub_gpos.dead_reckoning = ~bitand(self.estimatorInitialized, self.EST_XY);
		self.pub_gpos.pressure_alt = self.sub_sensor.baro_alt_meter;
		% self.pub_gpos.update();
		% TODO provide calculated values for these
		self.pub_gpos.evh = 0.0;
		self.pub_gpos.evv = 0.0;
	end
end

% void BlockLocalPositionEstimator::
function self = initP( self )
%INIT_P Initialise P matrix

	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10; n_x = 10;

	self.P = zeros( n_x, n_x );
	% initialize to twice valid condition
	self.P(X_x, X_x) = 2 * self.EST_STDDEV_XY_VALID * self.EST_STDDEV_XY_VALID;
	self.P(X_y, X_y) = 2 * self.EST_STDDEV_XY_VALID * self.EST_STDDEV_XY_VALID;
	self.P(X_z, X_z) = 2 * self.EST_STDDEV_Z_VALID * self.EST_STDDEV_Z_VALID;
	self.P(X_vx, X_vx) = 2 * self.vxy_pub_thresh * self.vxy_pub_thresh;
	self.P(X_vy, X_vy) = 2 * self.vxy_pub_thresh * self.vxy_pub_thresh;
	% use vxy thresh for vz init as well
	self.P(X_vz, X_vz) = 2 * self.vxy_pub_thresh * self.vxy_pub_thresh;
	% initialize bias uncertainty to small values to keep them stable
	self.P(X_bx, X_bx) = 1e-6;
	self.P(X_by, X_by) = 1e-6;
	self.P(X_bz, X_bz) = 1e-6;
	self.P(X_tz, X_tz) = 2 * self.EST_STDDEV_TZ_VALID * self.EST_STDDEV_TZ_VALID;
end

% DONE
%void BlockLocalPositionEstimator::
function self = initSS( self )
%INITSS Initialize state space model
	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10;
	U_ax = 1; U_ay = 2; U_az = 3; n_u = 3;

	self = initP( self );

	% dynamics matrix
	self.A = zeros(3, 3);
	% derivative of position is velocity
	self.A(X_x, X_vx) = 1;
	self.A(X_y, X_vy) = 1;
	self.A(X_z, X_vz) = 1;

	% input matrix
	self.B = zeros(3, 3);
	self.B(X_vx, U_ax) = 1;
	self.B(X_vy, U_ay) = 1;
	self.B(X_vz, U_az) = 1;

	% update components that depend on current state
	self = updateSSStates( self );
	self = updateSSParams( self );
end

%void BlockLocalPositionEstimator::
function self = updateSSStates( self )
%UPDATESSStates
	% derivative of velocity is accelerometer acceleration
	% (in input matrix) - bias (in body frame)
	self.A(X_vx, X_bx) = -self.R_att(1, 1);
	self.A(X_vx, X_by) = -self.R_att(1, 2);
	self.A(X_vx, X_bz) = -self.R_att(1, 3);

	self.A(X_vy, X_bx) = -self.R_att(2, 1);
	self.A(X_vy, X_by) = -self.R_att(2, 2);
	self.A(X_vy, X_bz) = -self.R_att(2, 3);

	self.A(X_vz, X_bx) = -self.R_att(3, 1);
	self.A(X_vz, X_by) = -self.R_att(3, 2);
	self.A(X_vz, X_bz) = -self.R_att(3, 3);
end

%void BlockLocalPositionEstimator::
function self = updateSSParams( self )
%UPDATESSPARAMS Update steady state parameters
	
	% enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10; n_x = 10;

	% input noise covariance matrix
	self.R = zeros(3, 3); %.setZero();
	self.R(1, 1) = self.accel_xy_stddev * self.accel_xy_stddev;
	self.R(2, 2) = self.accel_xy_stddev * self.accel_xy_stddev;
	self.R(3, 3) = self.accel_z_stddev * self.accel_z_stddev;

	% process noise power matrix
	self.Q = zeros(n_x, n_x); %.setZero();
	%float
	pn_p_sq = self.pn_p_noise_density * self.pn_p_noise_density;
	%float
	pn_v_sq = self.pn_v_noise_density * self.pn_v_noise_density;
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
	pn_b_sq = self.pn_b_noise_density * self.pn_b_noise_density;
	self.Q(X_bx, X_bx) = pn_b_sq;
	self.Q(X_by, X_by) = pn_b_sq;
	self.Q(X_bz, X_bz) = pn_b_sq;

	% terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	%float
	pn_t_noise_density = ...
		self.pn_t_noise_density + ...
		(self.t_max_grade / 100.0) * ...
		sqrt(self.x(X_vx) * self.x(X_vx) + self.x(X_vy) * self.x(X_vy));
	self.Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
end

% void BlockLocalPositionEstimator::
function self = predict( self )
%PREDICT Prediction step
	% Definitions
	U_ax = 1; U_ay = 2; U_az = 3; n_u = 3;
	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10; n_x = 10;

	% get acceleration
	q = self.sub_att.q; % matrix::Quaternion<float> q(&self.sub_att.get().q(0));
	self.eul = quat_to_euler( q ); %matrix::Euler<float>(q);
	self.R_att = quat_to_dcm( q ); % matrix::Dcm<float>(q);
	a = self.sub_sensor.accelerometer_m_s2; % Vector3f a(self.sub_sensor.get().accelerometer_m_s2);
	% note, bias is removed in dynamics function
	self.u = self.R_att * a;
	self.u(U_az) = self.u(U_az) + 9.81; % add g

	% update state space based on new states
	self = updateSSStates( self );

	% continuous time kalman filter prediction
	% integrate runge kutta 4th order
	% TODO move rk4 algorithm to matrixlib
	% https:%en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	h = self.dt; % float
	% Vector<float, n_x> k1, k2, k3, k4;
	k1 = zeros( n_x, 1 ); k2 = k1; k3 = k1; k4 = k1;
	k1 = dynamics(0, self.x, self.u);
	k2 = dynamics(h / 2, self.x + k1 * h / 2, self.u);
	k3 = dynamics(h / 2, self.x + k2 * h / 2, self.u);
	k4 = dynamics(h, self.x + k3 * h, self.u);
	
	% Vector<float, n_x>
	dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	% don't integrate position if no valid xy data
	if ( ~( bitand( self.estimatorInitialized, self.EST_XY ) ) )
		dx(X_x) = 0;
		dx(X_vx) = 0;
		dx(X_y) = 0;
		dx(X_vy) = 0;
	end

	% don't integrate z if no valid z data
	if ( ~( bitand( self.estimatorInitialized, self.EST_Z) ) )
		dx(X_z) = 0;
	end

	% don't integrate tz if no valid tz data
	if ( ~( bitand( self.estimatorInitialized, self.EST_TZ ) ) )
		dx(X_tz) = 0;
	end

	% saturate bias all float
	bx = dx(X_bx) + self.x(X_bx);
	by = dx(X_by) + self.x(X_by);
	bz = dx(X_bz) + self.x(X_bz);

	if (abs(bx) > self.BIAS_MAX)
		bx = self.BIAS_MAX * bx / abs(bx);
		dx(X_bx) = bx - self.x(X_bx);
	end

	if (abs(by) > self.BIAS_MAX)
		by = self.BIAS_MAX * by / abs(by);
		dx(X_by) = by - self.x(X_by);
	end

	if (abs(bz) > self.BIAS_MAX)
		bz = self.BIAS_MAX * bz / abs(bz);
		dx(X_bz) = bz - self.x(X_bz);
	end

	% propagate
	self.x = self.x + dx;
	% Matrix<float, n_x, n_x>
	dP = (self.A * self.P + self.P * self.A' +
		self.B * self.R * self.B' + self.Q) * self.dt;

	% covariance propagation logic
	for i = 1:n_x %; i < n_x; i++)
		if (self.P(i, i) > self.P_MAX)
			% if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for j = 1:n_x %; j < n_x; j++)
				dP(i, j) = 0;
				dP(j, i) = 0;
			end
		end
	end

	self.P = self.P + dP;
	self.xLowPass = block_lowpass_update( self.x, self.xLowPass );
	agl = self.x(X_tz) - self.x(X_z);
	self.aglLowPass = block_lowpass_update( agl, self.aglLowPass );
end

% Not examined - Probably not used
% int BlockLocalPositionEstimator::
function [ self, output, periods ] = getDelayPeriods( self, delay )
%GETDELAYPERIODS
	%float
	t_delay = 0;
	%uint8_t
	i_hist = 0;

	HIST_LEN = 10; % DELAY_MAX / HIST_STEP
	for i_hist = 2:HIST_LEN %; i_hist < HIST_LEN; i_hist++) {
		tmp = (self.timeStamp - self.tDelay.get(i_hist);
		t_delay = 1.0e-6 * tmp(1, 1));

		if (t_delay > delay)
			break;
		end
	end

	periods = i_hist;

	DELAY_MAX = 0.5;
	if (t_delay > DELAY_MAX)
		% mavlink_and_console_log_info(&mavlink_log_pub, '%sdelayed data old: %8.4f', self.msg_label, double(t_delay));
		output = false;
		return;
	end

	output = true;
end


%% ADDITIONAL FUNCTIONS

function out = quat_to_euler( data )
%QUAT_TO_EULER Create Euler angles vector from the quaternion
    out = [atan2(2.0 * (data(1) * data(2) + data(3) * data(4)), 1.0 - 2.0 * (data(2) * data(2) + data(3) * data(3)));
    asin(2.0 * (data(1) * data(3) - data(4) * data(2)));
    atan2(2.0 * (data(1) * data(4) + data(2) * data(3)), 1.0 - 2.0 * (data(3) * data(3) + data(4) * data(4)))];
end

function R  = quat_to_dcm( data )
%QUAT_TO_DCM Convert quaternion to DCM
%	From https:%github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Quaternion.hpp
    aSq = data(1) * data(1);
    bSq = data(2) * data(2);
    cSq = data(3) * data(3);
    dSq = data(4) * data(4);
    R = zeros(3,3);
    R(1,1) = aSq + bSq - cSq - dSq;
    R(1,2) = 2.0 * ( data(2) * data(3) - data(1) * data(4) );
    R(1,3) = 2.0 * ( data(1) * data(3) + data(2) * data(4) );
    R(2,1) = 2.0 * ( data(2) * data(3) + data(1) * data(4) );
    R(2,2) = aSq - bSq + cSq - dSq;
    R(2,3) = 2.0 * ( data(3) * data(4) - data(1) * data(2) );
    R(3,1) = 2.0 * ( data(2) * data(4) - data(1) * data(3) );
    R(3,2) = 2.0 * ( data(1) * data(2) + data(3) * data(4) );
    R(3,3) = aSq - bSq - cSq + dSq;
end

function [ out, lat, lon ] = map_projection_reproject( ref, x, y, lat, lon )
%MAP_PROJECTION_REPROJECT
    CONSTANTS_RADIUS_OF_EARTH = 6371000;
    M_PI = 3.14159265358979323846;
    
    if ( ~map_projection_initialized(ref))
        out = -1;
        return
    end

    x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    c = sqrt(x_rad * x_rad + y_rad * y_rad);
    
    sin_c = sin(c);
    cos_c = cos(c);

    if (abs(c) > eps)
        lat_rad = asin(cos_c * ref.sin_lat + (x_rad * sin_c * ref.cos_lat) / c);
        lon_rad = (ref.lon_rad + atan2(y_rad * sin_c, c * ref.cos_lat * cos_c - x_rad * ref.sin_lat * sin_c));
    else
        lat_rad = ref.lat_rad;
        lon_rad = ref.lon_rad;
    end

    lat = lat_rad * 180.0 / M_PI;
    lon = lon_rad * 180.0 / M_PI;
    
    out = 0;
end

function [ out, x, y ] = map_projection_project( ref, lat, lon, x, y )
%MAP_PROJECTION_PROJECT
    CONSTANTS_RADIUS_OF_EARTH = 6371000;
    M_DEG_TO_RAD = 0.017453292519943295;
    
    if ( ~map_projection_initialized(ref))
        out = -1;
        return
    end
    
    lat_rad = lat * M_DEG_TO_RAD;
    lon_rad = lon * M_DEG_TO_RAD;

    sin_lat = sin(lat_rad);
    cos_lat = cos(lat_rad);
    cos_d_lon = cos(lon_rad - ref.lon_rad);

    c = acos(ref.sin_lat * sin_lat + ref.cos_lat * cos_lat * cos_d_lon);
    % k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));
    if (abs(c) < eps)
        k = 1.0;
    else
        k = c / sin(c);
    end

    x = k * (ref.cos_lat * sin_lat - ref.sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    y = k * cos_lat * sin(lon_rad - ref.lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

    out = 0;
end

function init = map_projection_initialized( ref )
% MAP_PROJECTION_INITIALIZED
    init = ref.init_done;
end

function [out, lp_block] = block_lowpass_update( input, lp_block )
%BLOCK_LOWPASS_UPADE Update low pass filter
    M_PI = 3.14159265358979323846;
    if (~isfinite(lp_block.state))
		lp_block.state = input;
    end

    b = 2 * M_PI * lp_block.fcut * lp_block.dt;
    a = b / (1 + b);
    lp_block.state = a * input + (1 - a) * lp_block.state;
    out = lp_block.state;
end

function self = initialize_blocks(dt, self)
%INITIALIZE_BLOCKS Initialise low pass blocks
    self.xLowPass.dt = dt;
    self.aglLowPass.dt = dt;
	self.xLowPass.fcut = self.LPE_X_LP; % TO MODIFY
    self.aglLowPass.fcut = self.LPE_X_LP; % TO MODIFY
end

function self = setDt( dt, self )
%SETDT Set dt for subblocks
    self.xLowPass.dt = dt;
    self.aglLowPass.dt = dt;
end

%% MOCAP
% void BlockLocalPositionEstimator::
function self = mocapInit( self )
%MOCAPINIT Initialise mocap
	% measure
	%Vector<float, n_y_mocap>
	n_y_mocap = 3;
	y = zeros(n_y_mocap, 1);

	[self, success, y] = mocapMeasure( self );

	if ( success ~= true )
		self.mocapStats.count = 0;
		return;
	end

	% if finished
	REQ_MOCAP_INIT_COUNT = 20;
	if (self.mocapStats.count > REQ_MOCAP_INIT_COUNT)
		% mavlink_and_console_log_info(&mavlink_log_pub, ['[lpe] mocap position init: '
		% 			     '%5.2f, %5.2f, %5.2f m std %5.2f, %5.2f, %5.2f m'],
		% 			     double(self.mocapStats.getMean()(0)),
		% 			     double(self.mocapStats.getMean()(1)),
		% 			     double(self.mocapStats.getMean()(2)),
		% 			     double(self.mocapStats.getStdDev()(0)),
		% 			     double(self.mocapStats.getStdDev()(1)),
		% 			     double(self.mocapStats.getStdDev()(2)));
		self.sensorTimeout = bitand( self.sensorTimeout, ~SENSOR_MOCAP );
		self.sensorFault = bitand( self.sensorFault, ~SENSOR_MOCAP );

		if (~self.altOriginInitialized)
			self.altOriginInitialized = true;
			self.altOrigin = 0;
		end
	end
end

% int BlockLocalPositionEstimator:: % Vector<float, n_y_mocap> &y
function [self, output, y] = mocapMeasure( self )
%MOCAPMEASURE Retrieve mocap measurement
	Y_mocap_x = 1; Y_mocap_y = 2; Y_mocap_z = 3; n_y_mocap = 3;
	y = zeros(n_y_mocap, 1);
	y(Y_mocap_x) = self.sub_mocap.x;
	y(Y_mocap_y) = self.sub_mocap.y;
	y(Y_mocap_z) = self.sub_mocap.z;
	% self.mocapStats.update( y );
	self.mocapStats.count = self.mocapStats.count + 1;
	self.time_last_mocap = self.sub_mocap.timestamp;
	output = true;
end

% void BlockLocalPositionEstimator::
function self = mocapCorrect( self )
%MOCAPCORRECT Perform correction step with mocap data
	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10;
	Y_mocap_x = 1; Y_mocap_y = 2; Y_mocap_z = 3; n_y_mocap = 3;

	% measure
	% Vector<float, n_y_mocap> y;
	y = zeros(n_y_mocap, 1);

	[self, success, y] = mocapMeasure( self );

	if ( success ~= true)
		return;
	end

	% mocap measurement matrix, measures position
	% Matrix<float, n_y_mocap, n_x> C;
	% C.setZero();
	C = zeros( n_y_mocap, n_x );
	C(Y_mocap_x, X_x) = 1;
	C(Y_mocap_y, X_y) = 1;
	C(Y_mocap_z, X_z) = 1;

	% noise matrix
	% Matrix<float, n_y_mocap, n_y_mocap> R;
	% R.setZero();

	R = zeros( n_y_mocap, n_y_mocap );

	% float
	mocap_p_var = self.mocap_p_stddev * self.mocap_p_stddev;
	R(Y_mocap_x, Y_mocap_x) = mocap_p_var;
	R(Y_mocap_y, Y_mocap_y) = mocap_p_var;
	R(Y_mocap_z, Y_mocap_z) = mocap_p_var;

	% residual
	% Matrix<float, n_y_mocap, n_y_mocap> inv<float, n_y_mocap>
	S_I = inv((C * self.P * C') + R);
	% Matrix<float, n_y_mocap, 1>
	r = y - C * self.x;

	% fault detection
	% float
	betaTmp = ( r' * (S_I * r));
	beta1 = betaTmp(1, 1);

	if ( beta1 > self.BETA_TABLE(n_y_mocap) ) 
		if ( ~bitand( self.sensorFault, self.SENSOR_MOCAP ) )
			%mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap fault, beta1 %5.2f", double(beta1));
			self.sensorFault = bitor( self.sensorFault, self.SENSOR_MOCAP );
		end
	else if bitand(self.sensorFault, self.SENSOR_MOCAP)
		self.sensorFault = bitand( self.sensorFault, ~self.SENSOR_MOCAP );
		%mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap OK");
	end

	% kalman filter correction always
	% Matrix<float, n_x, n_y_mocap>
	K = self.P * C' * S_I;
	% Vector<float, n_x>
	dx = K * r;
	self.x = self.x + dx;
	self.P = self.P - K * C * self.P;
end

% void BlockLocalPositionEstimator::
function self = mocapCheckTimeout( self )
%MOCAPCHECKTIMEOUT Check if mocap has timedout
	MOCAP_TIMEOUT = 200000; % 0.2 s

	if (self.timeStamp - self.time_last_mocap > MOCAP_TIMEOUT)
		if (~bitand(self.sensorTimeout, self.SENSOR_MOCAP))
			self.sensorTimeout = bitor( self.sensorTimeout, SENSOR_MOCAP ); % |=
			% self.mocapStats.reset();
			self.mocapStats.count = 0;
			% mavlink_and_console_log_info(mavlink_log_pub, '[lpe] mocap timeout ');
		end
	end
end

%% BARO
% void BlockLocalPositionEstimator::
function self = baroInit( self )
%BAROINIT Initialise barometer
	% measure
	y = 0;

	[self, success, y] = baroMeasure( self );

	if ( success ~= true )
		self.baroStats.count = 0;
		self.baroStats.total = 0;
		return;
	end

	% if finished
	REQ_BARO_INIT_COUNT = 100;
	if ( self.baroStats.count > REQ_BARO_INIT_COUNT )
		self.baroAltOrigin = self.baroStats.total / self.baroStats.count; % TAKE CARE OF THIS
		% mavlink_and_console_log_info(&mavlink_log_pub,
		% 			     '[lpe] baro init %d m std %d cm',
		% 			     (int)self.baroStats.getMean()(0),
		% 			     (int)(100 * self.baroStats.getStdDev()(0)));
		self.sensorTimeout = bitand( self.sensorTimeout, ~SENSOR_BARO );
		self.sensorFault = bitand( self.sensorFault, ~SENSOR_BARO );

		if (~self.altOriginInitialized)
			self.altOriginInitialized = true;
			self.altOrigin = self.baroAltOrigin;
		end
	end
end

% int BlockLocalPositionEstimator:: Vector<float, n_y_baro> &y
function [self, output, y ] baroMeasure( self )
%BAROMEASURE Retrieve barometer measurement
	%measure
	y = 0;
	y = self.sub_sensor.baro_alt_meter;
	self.baroStats.count = self.baroStats.count + 1; %update(y);
	self.baroStats.total = self.baroStats.total + y;
	self.time_last_baro = self.timeStamp;
	output = true;
end

void BlockLocalPositionEstimator::
function self = baroCorrect( self )
%BAROCORRECT Correction step for barometer measurement
	X_x = 1; X_y = 2; X_z = 3; X_vx = 4; X_vy = 5; X_vz = 6;
	X_bx = 7; X_by = 8; X_bz = 9; X_tz = 10;
	Y_baro_z = 1; n_y_baro = 1;

	% measure
	%Vector<float, n_y_baro>
	y = 0;

	[self, success, y] = baroMeasure( self );

	if ( success ~= true )
		self.baroStats.count = 0;
		self.baroStats.total = 0;
		return;
	end

	% subtract baro origin alt
	y = y - self.baroAltOrigin;

	% baro measurement matrix
	% Matrix<float, n_y_baro, n_x> C;
	% C.setZero();
	C = zeros( n_y_baro, n_x );

	C(Y_baro_z, X_z) = -1; % measured altitude, negative down dir.

	% Matrix<float, n_y_baro, n_y_baro> R;
	% R.setZero();
	R = zeros( n_y_baro, n_y_baro );
	R(0, 0) = self.baro_stddev * self.baro_stddev;

	% residual
	% Matrix<float, n_y_baro, n_y_baro> <float, n_y_baro>
	S_I = inv((C * self.P * C') + R);
	% Vector<float, n_y_baro>
	r = y - (C * self.x);

	% fault detection
	% float
	betaTmp = (r' * (S_I * r));
	beta1 = betaTmp(1, 1);

	if (beta1 > self.BETA_TABLE(n_y_baro) )
		if (~bitand(self.sensorFault, self.SENSOR_BARO))
			% mavlink_log_critical(&mavlink_log_pub, '[lpe] baro fault, r %5.2f m, beta1 %5.2f', ...
			% 		     double(r(0)), double(beta1));
			self.sensorFault = bitor( self.sensorFault, self.SENSOR_BARO );
		end

	else if bitand(self.sensorFault, self.SENSOR_BARO)
		self.sensorFault = bitand( self.sensorFault, ~self.SENSOR_BARO );
		% mavlink_and_console_log_info(mavlink_log_pub, '[lpe] baro OK');
	end

	% kalman filter correction always
	%Matrix<float, n_x, n_y_baro>
	K = self.P * C' * S_I;
	%Vector<float, n_x>
	dx = K * r;
	self.x = self.x + dx;
	self.P = self.P - K * C * self.P;
end

% void BlockLocalPositionEstimator::
function self = baroCheckTimeout( self )
%BAROCHECKTIMEOUT Check barometer timeout
	BARO_TIMEOUT = 100000;
	if ( (self.timeStamp - self.time_last_baro) > BARO_TIMEOUT)
		if (~bitand(self.sensorTimeout, SENSOR_BARO))
			self.sensorTimeout = bitor( self.sensorTimeout, self.SENSOR_BARO );
			self.baroStats.count = 0;
			self.baroStats.total = 0;
			% mavlink_and_console_log_info(mavlink_log_pub, '[lpe] baro timeout ');
		end
	end
end