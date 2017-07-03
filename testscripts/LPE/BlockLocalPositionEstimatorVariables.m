% self.sub_armed(ORB_ID(actuator_armed), 1000 / 2, 0, &getSubscriptions()),
% self.sub_land(ORB_ID(vehicle_land_detected), 1000 / 2, 0, &getSubscriptions()),
% self.sub_att(ORB_ID(vehicle_attitude), 1000 / 100, 0, &getSubscriptions()),
% self.sub_flow(ORB_ID(optical_flow), 1000 / 100, 0, &getSubscriptions()),
% self.sub_sensor(ORB_ID(sensor_combined), 1000 / 100, 0, &getSubscriptions()),
% self.sub_param_update(ORB_ID(parameter_update), 1000 / 2, 0, &getSubscriptions()),
% self.sub_manual(ORB_ID(manual_control_setpoint), 1000 / 2, 0, &getSubscriptions()),
% self.sub_gps(ORB_ID(vehicle_gps_position), 1000 / 10, 0, &getSubscriptions()),
% self.sub_vision_pos(ORB_ID(vehicle_vision_position), 1000 / 50, 0, &getSubscriptions()),
% self.sub_mocap(ORB_ID(att_pos_mocap), 1000 / 50, 0, &getSubscriptions()),
% self.sub_dist0(ORB_ID(distance_sensor), 1000 / 10, 0, &getSubscriptions()),
% self.sub_dist1(ORB_ID(distance_sensor), 1000 / 10, 1, &getSubscriptions()),
% self.sub_dist2(ORB_ID(distance_sensor), 1000 / 10, 2, &getSubscriptions()),
% self.sub_dist3(ORB_ID(distance_sensor), 1000 / 10, 3, &getSubscriptions()),
% self.dist_subs(),
% self.sub_lidar(nullptr),
% self.sub_sonar(nullptr),
% self.pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
% self.pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
% self.pub_est_status(ORB_ID(estimator_status), -1, &getPublications()),
% self.pub_innov(ORB_ID(ekf2_innovations), -1, &getPublications()),
% self.map_ref(),
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
% self.fake_origin(this, "FAKE_ORIGIN"),
% self.init_origin_lat(this, "LAT"),
% self.init_origin_lon(this, "LON"),
% self.flow_gyro_x_high_pass(this, "FGYRO_HP"),
% self.flow_gyro_y_high_pass(this, "FGYRO_HP"),
% self.baroStats(this, ""),
% self.sonarStats(this, ""),
% self.lidarStats(this, ""),
% self.flowQStats(this, ""),
% self.visionStats(this, ""),
% self.mocapStats(this, ""),
% self.gpsStats(this, ""),
% self.xLowPass(this, "X_LP"),
% self.aglLowPass(this, "X_LP"),
% self.xDelay(this, ""),
% self.tDelay(this, ""),
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
% self.altOrigin(0),
% self.altOriginInitialized(false),
% self.baroAltOrigin(0),
% self.gpsAltOrigin(0),
% self.receivedGps(false),
% self.lastArmedState(false),
% self.sensorTimeout(255),
% self.sensorFault(0),
% self.estimatorInitialized(0)
self.dist_subs(0) = &self.sub_dist0;
self.dist_subs(1) = &self.sub_dist1;
self.dist_subs(2) = &self.sub_dist2;
self.dist_subs(3) = &self.sub_dist3;
self.polls(POLL_FLOW).fd = self.sub_flow.getHandle();
self.polls(POLL_FLOW).events = POLLIN;
self.polls(POLL_PARAM).fd = self.sub_param_update.getHandle();
self.polls(POLL_PARAM).events = POLLIN;
self.polls(POLL_SENSORS).fd = self.sub_sensor.getHandle();
self.polls(POLL_SENSORS).events = POLLIN;
self.x.setZero();
self.u.setZero();
self.map_ref.init_done = false;
   (self.fusion & FUSE_GPS) ~= 0,       ...
   (self.fusion & FUSE_FLOW) ~= 0,      ...
   (self.fusion & FUSE_VIS_POS) ~= 0,   ...
   (self.fusion & FUSE_VIS_YAW) ~= 0,   ...
   (self.fusion & FUSE_LAND) ~= 0,      ...
   (self.fusion & FUSE_PUB_AGL_Z) ~= 0, ...
   (self.fusion & FUSE_FLOW_GYRO_COMP) ~= 0 );
output = self.A * x + self.B * u;
ret = px4_poll(self.polls, 3, 100);
dt = (newTimeStamp - self.timeStamp) / 1.0e6;
self.timeStamp = newTimeStamp;
armedState = self.sub_armed.armed;
if (~armedState && (self.sub_lidar == nullptr || self.sub_sonar == nullptr))
	%uORB::Subscription<distance_sensor_s> *s = self.dist_subs(i);
	s = self.dist_subs(i);
	if (s == self.sub_lidar || s == self.sub_sonar)
			self.sub_lidar == nullptr)
			self.sub_lidar = s;
			   self.sub_sonar == nullptr)
			self.sub_sonar = s;
% if (~self.lastArmedState && armedState) {
% 	self.x(X_x) = 0;
% 	self.x(X_y) = 0;
% 	% reset Z or not? self.x(X_z) = 0;
% 	self.x(X_vx) = 0;
% 	self.x(X_vy) = 0;
% 	self.x(X_vz) = 0;
% 	self.x(X_tz) = self.x(X_z);
% 	self.xLowPass.setState(self.x);
% 	self.aglLowPass.setState(0);
self.lastArmedState = armedState;
paramsUpdated = self.sub_param_update.updated;
if ( (self.fusion & FUSE_BARO) && self.sub_sensor.updated )
baro_timestamp_relative = self.sub_sensor.baro_timestamp_relative;
if ( baro_timestamp_relative ~= self.sub_sensor.RELATIVE_TIMESTAMP_INVALID )
	baro_timestamp = self.sub_sensor.timestamp + ...
				  self.sub_sensor.baro_timestamp_relative;
	if (baro_timestamp ~= self.timeStampLastBaro)
		self.timeStampLastBaro = baro_timestamp;
flowUpdated = (self.fusion & FUSE_FLOW) && self.sub_flow.updated();
gpsUpdated = (self.fusion & FUSE_GPS) && self.sub_gps.updated();
visionUpdated = (self.fusion & FUSE_VIS_POS) && self.sub_vision_pos.updated();
mocapUpdated = self.sub_mocap.updated();
lidarUpdated = (self.sub_lidar ~= nullptr) && self.sub_lidar->updated();
sonarUpdated = (self.sub_sonar ~= nullptr) && self.sub_sonar->updated();
	   && ((self.timeStamp - self.time_last_land) > 1.0e6 / LAND_RATE); % throttle rate
if (max(self.P(X_vx, X_vx), self.P(X_vy, X_vy)) < self.vxy_pub_thresh * self.vxy_pub_thresh)
if (self.estimatorInitialized & EST_XY)
if (~vxy_stddev_ok && (self.sensorTimeout & SENSOR_GPS))
	self.estimatorInitialized &= ~EST_XY;
	if (~(self.sensorTimeout & SENSOR_GPS)       ...
		|| ~(self.sensorTimeout & SENSOR_FLOW)   ...
		|| ~(self.sensorTimeout & SENSOR_VISION) ...
		|| ~(self.sensorTimeout & SENSOR_MOCAP)  ...
		|| ~(self.sensorTimeout & SENSOR_LAND)   ...
		self.estimatorInitialized |= EST_XY;
z_stddev_ok = sqrt(self.P(X_z, X_z)) < self.z_pub_thresh;
if (self.estimatorInitialized & EST_Z)
if (~z_stddev_ok && (self.sensorTimeout & SENSOR_BARO))
	self.estimatorInitialized &= ~EST_Z;
	self.estimatorInitialized |= EST_Z;
tz_stddev_ok = sqrt(self.P(X_tz, X_tz)) < self.z_pub_thresh;
if (self.estimatorInitialized & EST_TZ)
	self.estimatorInitialized &= ~EST_TZ;
	self.estimatorInitialized |= EST_TZ;
if (~self.map_ref.init_done && (self.estimatorInitialized & EST_XY) && self.fake_origin)
map_projection_init(&self.map_ref, ...
			self.init_origin_lat, ...
			self.init_origin_lon);
self.time_origin = self.timeStamp;
				 double(self.init_origin_lat), double(self.init_origin_lon), double(self.altOrigin));
if (~isfinite(self.x(i)))
	self.x(i) = 0;
	if (~isfinite(self.P(i, j)))
		if (self.P(i, i) <= 0)
		self.P(j, i) = self.P(i, j);
if (self.sensorTimeout & SENSOR_GPS)
if (self.sensorTimeout & SENSOR_BARO)
if (self.sensorTimeout & SENSOR_LIDAR)
if (self.sensorTimeout & SENSOR_SONAR)
if (self.sensorTimeout & SENSOR_FLOW)
if (self.sensorTimeout & SENSOR_VISION)
if (self.sensorTimeout & SENSOR_MOCAP)
if (self.sensorTimeout & SENSOR_LAND)
if (self.altOriginInitialized)
self.pub_innov.update();
if ((self.estimatorInitialized & EST_XY) && (self.map_ref.init_done || self.fake_origin))
float dt_hist = 1.0e-6 * (self.timeStamp - self.time_last_hist);
if (self.time_last_hist == 0 || ...
self.tDelay.update(Scalar<uint64_t>(self.timeStamp));
self.xDelay.update(self.x);
self.time_last_hist = self.timeStamp;
if (~(self.fusion & FUSE_LAND))
disarmed_not_falling = (~self.sub_armed.armed) && (~self.sub_land.freefall);
output = self.sub_land.landed || disarmed_not_falling;
% self.pub_lpos.update(); ?
xLP = self.xLowPass.getState();
vxy_stddev = sqrt(self.P(X_vx, X_vx) + self.P(X_vy, X_vy));
epv = sqrt(self.P(X_z, X_z));
eph = sqrt(self.P(X_x, X_x) + self.P(X_y, X_y));
if (vxy_stddev < self.vxy_pub_thresh)
if (isfinite(self.x(X_x)) && isfinite(self.x(X_y)) && isfinite(self.x(X_z)) && ...
isfinite(self.x(X_vx)) && isfinite(self.x(X_vy)) ...
&& isfinite(self.x(X_vz)))
self.pub_lpos.timestamp = self.timeStamp;
self.pub_lpos.xy_valid = self.estimatorInitialized & EST_XY;
self.pub_lpos.z_valid = self.estimatorInitialized & EST_Z;
self.pub_lpos.v_xy_valid = self.estimatorInitialized & EST_XY;
self.pub_lpos.v_z_valid = self.estimatorInitialized & EST_Z;
self.pub_lpos.x = xLP(X_x); 	% north
self.pub_lpos.y = xLP(X_y);  	% east
if (self.fusion & FUSE_PUB_AGL_Z)
	self.pub_lpos.z = -self.aglLowPass.getState(); % agl
	self.pub_lpos.z = xLP(X_z); 	% down
self.pub_lpos.vx = xLP(X_vx); % north
self.pub_lpos.vy = xLP(X_vy); % east
self.pub_lpos.vz = xLP(X_vz); % down
self.pub_lpos.z_deriv = xLP(X_vz);
self.pub_lpos.yaw = self.eul(3);
self.pub_lpos.xy_global = self.estimatorInitialized & EST_XY;
self.pub_lpos.z_global = ~(self.sensorTimeout & SENSOR_BARO);
self.pub_lpos.ref_timestamp = self.time_origin;
self.pub_lpos.ref_lat = self.map_ref.lat_rad * 180 / pi;
self.pub_lpos.ref_lon = self.map_ref.lon_rad * 180 / pi;
self.pub_lpos.ref_alt = self.altOrigin;
self.pub_lpos.dist_bottom = self.aglLowPass.getState();
self.pub_lpos.dist_bottom_rate = - xLP(X_vz);
self.pub_lpos.surface_bottom_timestamp = self.timeStamp;
self.pub_lpos.dist_bottom_valid = self.estimatorInitialized & EST_Z;
self.pub_lpos.eph = eph;
self.pub_lpos.epv = epv;
self.pub_lpos.update();
self.pub_lpos.evh = 0.0;
self.pub_lpos.evv = 0.0;
% NEED TO TAKE CARE OF self.pub_est_status.update();
self.pub_est_status.timestamp = self.timeStamp;
self.pub_est_status.states(i) = self.x(i);
self.pub_est_status.covariances(i) = self.P(i, i);
self.pub_est_status.n_states = n_x;
self.pub_est_status.nan_flags = 0;
self.pub_est_status.health_flags = self.sensorFault;
self.pub_est_status.timeout_flags = self.sensorTimeout;
self.pub_est_status.pos_horiz_accuracy = self.pub_gpos.eph;
self.pub_est_status.pos_vert_accuracy = self.pub_gpos.epv;
self.pub_est_status.update();
% NEED TO TAKE CARE OF self.pub_gpos.update();
xLP = self.xLowPass.getState();
[ ~, lat, lon ] = map_projection_reproject(self.map_ref, xLP(X_x), xLP(X_y), lat, lon);
alt = -xLP(X_z) + self.altOrigin;
vxy_stddev = sqrt(self.P(X_vx, X_vx) + self.P(X_vy, X_vy));
epv = sqrt(self.P(X_z, X_z));
eph = sqrt(self.P(X_x, X_x) + self.P(X_y, X_y));
if (vxy_stddev < self.vxy_pub_thresh)
self.pub_gpos.timestamp = self.timeStamp;
self.pub_gpos.time_utc_usec = self.sub_gps.time_utc_usec;
self.pub_gpos.lat = lat;
self.pub_gpos.lon = lon;
self.pub_gpos.alt = alt;
self.pub_gpos.vel_n = xLP(X_vx);
self.pub_gpos.vel_e = xLP(X_vy);
self.pub_gpos.vel_d = xLP(X_vz);
self.pub_gpos.pos_d_deriv = xLP(X_vz);
self.pub_gpos.yaw = self.eul(3);
self.pub_gpos.eph = eph;
self.pub_gpos.epv = epv;
self.pub_gpos.terrain_alt = self.altOrigin - xLP(X_tz);
self.pub_gpos.terrain_alt_valid = self.estimatorInitialized & EST_TZ;
self.pub_gpos.dead_reckoning = ~(self.estimatorInitialized & EST_XY);
self.pub_gpos.pressure_alt = self.sub_sensor.baro_alt_meter;
self.pub_gpos.update();
self.pub_gpos.evh = 0.0;
self.pub_gpos.evv = 0.0;
self.P = zeros( n_x, n_x );
self.P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
self.P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
self.P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
self.P(X_vx, X_vx) = 2 * self.vxy_pub_thresh * self.vxy_pub_thresh;
self.P(X_vy, X_vy) = 2 * self.vxy_pub_thresh * self.vxy_pub_thresh;
self.P(X_vz, X_vz) = 2 * self.vxy_pub_thresh * self.vxy_pub_thresh;
self.P(X_bx, X_bx) = 1e-6;
self.P(X_by, X_by) = 1e-6;
self.P(X_bz, X_bz) = 1e-6;
self.P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
self.A = zeros(3, 3);
self.A(X_x, X_vx) = 1;
self.A(X_y, X_vy) = 1;
self.A(X_z, X_vz) = 1;
self.B = zeros(3, 3);
self.B(X_vx, U_ax) = 1;
self.B(X_vy, U_ay) = 1;
self.B(X_vz, U_az) = 1;
self.A(X_vx, X_bx) = -self.R_att(1, 1);
self.A(X_vx, X_by) = -self.R_att(1, 2);
self.A(X_vx, X_bz) = -self.R_att(1, 3);
self.A(X_vy, X_bx) = -self.R_att(2, 1);
self.A(X_vy, X_by) = -self.R_att(2, 2);
self.A(X_vy, X_bz) = -self.R_att(2, 3);
self.A(X_vz, X_bx) = -self.R_att(3, 1);
self.A(X_vz, X_by) = -self.R_att(3, 2);
self.A(X_vz, X_bz) = -self.R_att(3, 3);
self.R = zeros(3, 3); %.setZero();
self.R(1, 1) = self.accel_xy_stddev * self.accel_xy_stddev;
self.R(2, 2) = self.accel_xy_stddev * self.accel_xy_stddev;
self.R(3, 3) = self.accel_z_stddev * self.accel_z_stddev;
self.Q = zeros(n_x, n_x); %.setZero();
pn_p_sq = self.pn_p_noise_density * self.pn_p_noise_density;
pn_v_sq = self.pn_v_noise_density * self.pn_v_noise_density;
self.Q(X_x, X_x) = pn_p_sq;
self.Q(X_y, X_y) = pn_p_sq;
self.Q(X_z, X_z) = pn_p_sq;
self.Q(X_vx, X_vx) = pn_v_sq;
self.Q(X_vy, X_vy) = pn_v_sq;
self.Q(X_vz, X_vz) = pn_v_sq;
pn_b_sq = self.pn_b_noise_density * self.pn_b_noise_density;
self.Q(X_bx, X_bx) = pn_b_sq;
self.Q(X_by, X_by) = pn_b_sq;
self.Q(X_bz, X_bz) = pn_b_sq;
self.pn_t_noise_density + ...
(self.t_max_grade / 100.0) * ...
sqrt(self.x(X_vx) * self.x(X_vx) + self.x(X_vy) * self.x(X_vy));
self.Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
q = self.sub_att.q(0); % matrix::Quaternion<float> q(&self.sub_att.get().q(0));
self.eul = quat_to_euler( q ); %matrix::Euler<float>(q);
self.R_att = quat_to_dcm( q ); % matrix::Dcm<float>(q);
a = self.sub_sensor.accelerometer_m_s2; % Vector3f a(self.sub_sensor.get().accelerometer_m_s2);
self.u = self.R_att * a;
self.u(U_az) += 9.81; % add g
k1 = dynamics(0, self.x, self.u);
k2 = dynamics(h / 2, self.x + k1 * h / 2, self.u);
k3 = dynamics(h / 2, self.x + k2 * h / 2, self.u);
k4 = dynamics(h, self.x + k3 * h, self.u);
if (~(self.estimatorInitialized & EST_XY))
if (~(self.estimatorInitialized & EST_Z))
if (~(self.estimatorInitialized & EST_TZ))
bx = dx(X_bx) + self.x(X_bx);
by = dx(X_by) + self.x(X_by);
bz = dx(X_bz) + self.x(X_bz);
dx(X_bx) = bx - self.x(X_bx);
dx(X_by) = by - self.x(X_by);
dx(X_bz) = bz - self.x(X_bz);
self.x = self.x + dx;
Matrix<float, n_x, n_x> dP = (self.A * self.P + self.P * self.A.transpose() +
			  self.B * self.R * self.B.transpose() + self.Q) * getDt();
if (self.P(i, i) > P_MAX)
self.P = self.P + dP;
self.xLowPass.update( self.x );
self.aglLowPass.update( agl() );
t_delay = 1.0e-6 * (self.timeStamp - self.tDelay.get(i_hist)(0, 0));