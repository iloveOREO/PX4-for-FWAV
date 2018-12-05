#include "BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>
#include <cstdlib>

orb_advert_t mavlink_log_pub = nullptr;

// required standard deviation of estimate for estimator to publish data
static const uint32_t		EST_STDDEV_XY_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_Z_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_TZ_VALID = 2.0;	// 2.0 m

static const float P_MAX = 1.0e6f;	// max allowed value in state covariance
static const float LAND_RATE = 10.0f;	// rate of land detector correction

static const char *msg_label = "[lpe] ";	// rate of land detector correction

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	// this block has no parent, and has name LPE
	SuperBlock(nullptr, "LPE"),		//用于制定父类的名字和该block的名字，因为该block没有父类所以为空指针
	ModuleParams(nullptr),			//c++里面的标准类，用去其他类使用配置参数
	// subscriptions, set rate, add to list
	_sub_armed(ORB_ID(actuator_armed), 1000 / 2, 0, &getSubscriptions()),
	_sub_land(ORB_ID(vehicle_land_detected), 1000 / 2, 0, &getSubscriptions()),
	_sub_att(ORB_ID(vehicle_attitude), 1000 / 100, 0, &getSubscriptions()),
	// set flow max update rate higher than expected to we don't lose packets
	_sub_flow(ORB_ID(optical_flow), 1000 / 100, 0, &getSubscriptions()),	//光流
	// main prediction loop, 100 hz
	_sub_sensor(ORB_ID(sensor_combined), 1000 / 100, 0, &getSubscriptions()),	//sensor_combined 订阅
	// status updates 2 hz
	_sub_param_update(ORB_ID(parameter_update), 1000 / 2, 0, &getSubscriptions()),	//参数更新	
	// gps 10 hz
	_sub_gps(ORB_ID(vehicle_gps_position), 1000 / 10, 0, &getSubscriptions()),	//订阅GPS位置
	// vision 50 hz
	_sub_visual_odom(ORB_ID(vehicle_visual_odometry), 1000 / 50, 0, &getSubscriptions()),	//订阅视觉里程计
	// mocap 50 hz
	_sub_mocap_odom(ORB_ID(vehicle_mocap_odometry), 1000 / 50, 0, &getSubscriptions()),		//订阅动作捕捉里程计
	// all distance sensors, 10 hz
	// 订阅其他距离传感器，雷达等
	_sub_dist0(ORB_ID(distance_sensor), 1000 / 10, 0, &getSubscriptions()),
	_sub_dist1(ORB_ID(distance_sensor), 1000 / 10, 1, &getSubscriptions()),
	_sub_dist2(ORB_ID(distance_sensor), 1000 / 10, 2, &getSubscriptions()),
	_sub_dist3(ORB_ID(distance_sensor), 1000 / 10, 3, &getSubscriptions()),
	_dist_subs(),
	_sub_lidar(nullptr),
	_sub_sonar(nullptr),
	_sub_landing_target_pose(ORB_ID(landing_target_pose), 1000 / 40, 0, &getSubscriptions()),
	_sub_airdata(ORB_ID(vehicle_air_data), 0, 0, &getSubscriptions()),

	// publications
	// 配置发布信息
	_pub_lpos(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_pub_gpos(ORB_ID(vehicle_global_position), -1, &getPublications()),
	_pub_est_status(ORB_ID(estimator_status), -1, &getPublications()),
	_pub_innov(ORB_ID(ekf2_innovations), -1, &getPublications()),

	// map projection
	_map_ref(),		//地图规划，这里主要初始化经纬度

	// flow gyro
	_flow_gyro_x_high_pass(this, "FGYRO_HP"),		// 设置flow gyro filter
	_flow_gyro_y_high_pass(this, "FGYRO_HP"),

	// stats
	_baroStats(this, ""),		// 配置统计功能
	_sonarStats(this, ""),
	_lidarStats(this, ""),
	_flowQStats(this, ""),
	_visionStats(this, ""),
	_mocapStats(this, ""),
	_gpsStats(this, ""),

	// low pass
	_xLowPass(this, "X_LP"),		//配置低通滤波
	// use same lp constant for agl
	_aglLowPass(this, "X_LP"),

	// delay
	_xDelay(this, ""),		//配置延时
	_tDelay(this, ""),

	// misc
	// 设置时间为0或者绝对时间
	_polls(),
	_timeStamp(hrt_absolute_time()),
	_time_origin(0),
	_timeStampLastBaro(hrt_absolute_time()),
	_time_last_hist(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_init_sonar(0),
	_time_last_vision_p(0),
	_time_last_mocap(0),
	_time_last_land(0),
	_time_last_target(0),

	// reference altitudes
	// 配置参考点的海拔与其他初始化参数
	_altOrigin(0),
	_altOriginInitialized(false),
	_altOriginGlobal(false),
	_baroAltOrigin(0),
	_gpsAltOrigin(0),

	// status
	_receivedGps(false),		// 初始化gps false
	_lastArmedState(false),		// 初始化armed状态 false

	// masks
	_sensorTimeout(UINT16_MAX),		// 初始化传感器
	_sensorFault(0),
	_estimatorInitialized(0),

	// sensor update flags
	_flowUpdated(false),		// 传感器参数状态全部置false
	_gpsUpdated(false),
	_visionUpdated(false),
	_mocapUpdated(false),
	_lidarUpdated(false),
	_sonarUpdated(false),
	_landUpdated(false),
	_baroUpdated(false),

	// sensor validation flags
	_vision_xy_valid(false),		// 传感器有效标志置false
	_vision_z_valid(false),
	_mocap_xy_valid(false),
	_mocap_z_valid(false),

	// sensor std deviations
	_vision_eph(0.0),		// 传感器 std 微分置0
	_vision_epv(0.0),
	_mocap_eph(0.0),
	_mocap_epv(0.0),

	// local to global coversion related variables
	_is_global_cov_init(false),		// local 到 global转换参数初始化
	_global_ref_timestamp(0.0),
	_ref_lat(0.0),
	_ref_lon(0.0),
	_ref_alt(0.0)
{
	// assign distance subs to array
	_dist_subs[0] = &_sub_dist0;
	_dist_subs[1] = &_sub_dist1;
	_dist_subs[2] = &_sub_dist2;
	_dist_subs[3] = &_sub_dist3;

	// setup event triggering based on new flow messages to integrate		// 配置轮循光流、参数和传感器信息
	_polls[POLL_FLOW].fd = _sub_flow.getHandle();
	_polls[POLL_FLOW].events = POLLIN;

	_polls[POLL_PARAM].fd = _sub_param_update.getHandle();
	_polls[POLL_PARAM].events = POLLIN;

	_polls[POLL_SENSORS].fd = _sub_sensor.getHandle();
	_polls[POLL_SENSORS].events = POLLIN;

	// initialize A, B,  P, x, u
	_x.setZero();
	_u.setZero();
	initSS();

	// map
	_map_ref.init_done = false;

	// print fusion settings to console
	printf("[lpe] fuse gps: %d, flow: %d, vis_pos: %d, "
	       "landing_target: %d, land: %d, pub_agl_z: %d, flow_gyro: %d, "
	       "baro: %d\n",
	       (_fusion.get() & FUSE_GPS) != 0,
	       (_fusion.get() & FUSE_FLOW) != 0,
	       (_fusion.get() & FUSE_VIS_POS) != 0,
	       (_fusion.get() & FUSE_LAND_TARGET) != 0,
	       (_fusion.get() & FUSE_LAND) != 0,
	       (_fusion.get() & FUSE_PUB_AGL_Z) != 0,
	       (_fusion.get() & FUSE_FLOW_GYRO_COMP) != 0,
	       (_fusion.get() & FUSE_BARO) != 0);
}

// 动态方程， 本质为一个一阶微分方程
Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::dynamics(
	float t,
	const Vector<float, BlockLocalPositionEstimator::n_x> &x,
	const Vector<float, BlockLocalPositionEstimator::n_u> &u)
{
	return _A * x + _B * u;
}

// update 函数流程
// 1.先调用预测函数predict结合加速度计和飞机此时的姿态位置进行预测
// 2.调用了各个模块的修正函数，比如mocapCorrect （在调用前有一些判断，如是否更新，是否启用，是否超时等）
void BlockLocalPositionEstimator::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	int ret = px4_poll(_polls, 3, 100);		// poll函数轮循flow 、 param_update、sensor_combined， 轮循周期100ms

	if (ret < 0) {		// 轮循超时
		return;
	}

	uint64_t newTimeStamp = hrt_absolute_time();		// 获取绝对时间
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;		// 计算程序更新周期(us)
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);		// 设置dt应用与所有子模块

	// auto-detect connected rangefinders while not armed
	bool armedState = _sub_armed.get().armed;		// 检测飞机是否解锁， armed 解锁， disarmed 未解锁

	if (!armedState && (_sub_lidar == nullptr || _sub_sonar == nullptr)) {		// 未解锁并且雷达和超声订阅为空
		// detect distance sensors
		for (size_t i = 0; i < N_DIST_SUBS; i++) {		// N_DIST_SUBS = 4 , 检查各个传感器
			uORB::Subscription<distance_sensor_s> *s = _dist_subs[i];

			if (s == _sub_lidar || s == _sub_sonar) { continue; }	// 判断距离数据来自雷达或超声 则continue

			if (s->updated()) {		// 如果数据不是雷达或超声， 更新距离数据
				s->update();

				if (s->get().timestamp == 0) { continue; }	// 首次获取数据， continue

				if (s->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER &&
				    s->get().orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING &&
				    _sub_lidar == nullptr) {		// 数据来自激光传感器 && 传感器向下 && 尚未订阅雷达
					_sub_lidar = s;		// 激光传感器作为雷达订阅
					mavlink_and_console_log_info(&mavlink_log_pub, "%sDownward-facing Lidar detected with ID %i", msg_label, i);

				} else if (s->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND &&
					   s->get().orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING &&
					   _sub_sonar == nullptr) {		// 超声， 将超声作为_sub_sonar的数据订阅
					_sub_sonar = s;
					mavlink_and_console_log_info(&mavlink_log_pub, "%sDownward-facing Sonar detected with ID %i", msg_label, i);
				}
			}
		}
	}

	// reset pos, vel, and terrain on arming

	// XXX this will be re-enabled for indoor use cases using a
	// selection param, but is really not helping outdoors
	// right now.

	// if (!_lastArmedState && armedState) {

	//      // we just armed, we are at origin on the ground
	//      _x(X_x) = 0;
	//      _x(X_y) = 0;
	//      // reset Z or not? _x(X_z) = 0;

	//      // we aren't moving, all velocities are zero
	//      _x(X_vx) = 0;
	//      _x(X_vy) = 0;
	//      _x(X_vz) = 0;

	//      // assume we are on the ground, so terrain alt is local alt
	//      _x(X_tz) = _x(X_z);

	//      // reset lowpass filter as well
	//      _xLowPass.setState(_x);
	//      _aglLowPass.setState(0);
	// }

	_lastArmedState = armedState;

	// see which updates are available
	bool paramsUpdated = _sub_param_update.updated();
	_baroUpdated = false;

	if ((_fusion.get() & FUSE_BARO) && _sub_airdata.updated()) {		// 如果气压计数据有更新，更新气压计
		if (_sub_airdata.get().timestamp != _timeStampLastBaro) {
			_baroUpdated = true;
			_timeStampLastBaro = _sub_airdata.get().timestamp;
		}
	}

	// 获取各传感器更新状态
	_flowUpdated = (_fusion.get() & FUSE_FLOW) && _sub_flow.updated();
	_gpsUpdated = (_fusion.get() & FUSE_GPS) && _sub_gps.updated();
	_visionUpdated = (_fusion.get() & FUSE_VIS_POS) && _sub_visual_odom.updated();
	_mocapUpdated = _sub_mocap_odom.updated();
	_lidarUpdated = (_sub_lidar != nullptr) && _sub_lidar->updated();
	_sonarUpdated = (_sub_sonar != nullptr) && _sub_sonar->updated();
	_landUpdated = landed() && ((_timeStamp - _time_last_land) > 1.0e6f / LAND_RATE);// throttle rate
	bool targetPositionUpdated = _sub_landing_target_pose.updated();

	// get new data
	updateSubscriptions();		// 获取更新数据

	// update parameters
	if (paramsUpdated) {
		SuperBlock::updateParams();
		ModuleParams::updateParams();
		updateSSParams();
	}

	// is xy valid?
	bool vxy_stddev_ok = false;		// 判断X坐标和Y坐标数据的有效标志

	// 判断 _P 中关于x、y方向的速度方差是否小于设定阈值，是则有效
	if (math::max(_P(X_vx, X_vx), _P(X_vy, X_vy)) < _vxy_pub_thresh.get() * _vxy_pub_thresh.get()) {
		vxy_stddev_ok = true;
	}

	// 判断测量XY方向的各个传感器是否都初始化了
	if (_estimatorInitialized & EST_XY) {		// 如果都初始化了
		// if valid and gps has timed out, set to not valid
		if (!vxy_stddev_ok && (_sensorTimeout & SENSOR_GPS)) {		// 如果vxy无效，并且gps超时，重置为未初始化状态
			_estimatorInitialized &= ~EST_XY;
		}

	} else {
		if (vxy_stddev_ok) {		// 如果vxy_stddev有效 , 只要下面有一个传感器初始化了并且未超时的话，设置估计为初始化成功
			if (!(_sensorTimeout & SENSOR_GPS)
			    || !(_sensorTimeout & SENSOR_FLOW)
			    || !(_sensorTimeout & SENSOR_VISION)
			    || !(_sensorTimeout & SENSOR_MOCAP)
			    || !(_sensorTimeout & SENSOR_LAND)
			    || !(_sensorTimeout & SENSOR_LAND_TARGET)
			   ) {
				_estimatorInitialized |= EST_XY;
			}
		}
	}

	// is z valid?
	bool z_stddev_ok = sqrtf(_P(X_z, X_z)) < _z_pub_thresh.get();		// 判断z坐标数据有效性

	if (_estimatorInitialized & EST_Z) {
		// if valid and baro has timed out, set to not valid
		if (!z_stddev_ok && (_sensorTimeout & SENSOR_BARO)) {
			_estimatorInitialized &= ~EST_Z;
		}

	} else {
		if (z_stddev_ok) {
			_estimatorInitialized |= EST_Z;
		}
	}

	// is terrain valid?
	bool tz_stddev_ok = sqrtf(_P(X_tz, X_tz)) < _z_pub_thresh.get();		// 判断地形数据有效性， 地面的海拔高度反应地形

	if (_estimatorInitialized & EST_TZ) {
		if (!tz_stddev_ok) {
			_estimatorInitialized &= ~EST_TZ;
		}

	} else {
		if (tz_stddev_ok) {
			_estimatorInitialized |= EST_TZ;
		}
	}

	// check timeouts
	checkTimeouts();		// 检查超时情况

	// 开始更新GPS数据
	// 如果地图未初始化 并且 估计器初始化有效 将类的初始化列表中的经纬度(即前面的_map_ref)设置为初始化经纬度
	// if we have no lat, lon initialize projection to LPE_LAT, LPE_LON parameters
	if (!_map_ref.init_done && (_estimatorInitialized & EST_XY) && _fake_origin.get()) {
		map_projection_init(&_map_ref,
				    (double)_init_origin_lat.get(),
				    (double)_init_origin_lon.get());

		// set timestamp when origin was set to current time
		_time_origin = _timeStamp;

		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] global origin init (parameter) : lat %6.2f lon %6.2f alt %5.1f m",
					     double(_init_origin_lat.get()), double(_init_origin_lon.get()), double(_altOrigin));
	}

	// reinitialize x if necessary
	bool reinit_x = false;		//重新初始化x数据的标识符为false

	for (size_t i = 0; i < n_x; i++) {		// x数据总共有十个
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!PX4_ISFINITE(_x(i))) {		// 只要有一个x数据发散，就需要将重新初始化x数据的标识符设为true，然后跳出循环
			reinit_x = true;
			mavlink_and_console_log_info(&mavlink_log_pub, "%sreinit x, x(%d) not finite", msg_label, i);
			break;
		}
	}

	if (reinit_x) {		// 如果需要重新初始化x，将当前所有的x数据置0
		for (size_t i = 0; i < n_x; i++) {
			_x(i) = 0;
		}

		mavlink_and_console_log_info(&mavlink_log_pub, "%sreinit x", msg_label);
	}

	// force P symmetry and reinitialize P if necessary
	bool reinit_P = false;		// 同 reinit_x

	for (size_t i = 0; i < n_x; i++) {
		for (size_t j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {		// 任意一个元素发散则将reinit_P置true， 后面将跳出循环
				mavlink_and_console_log_info(&mavlink_log_pub,
							     "%sreinit P (%d, %d) not finite", msg_label, i, j);
				reinit_P = true;
			}

			if (i == j) {
				// make sure diagonal elements are positive
				if (_P(i, i) <= 0) {		// 对角线元素小于0则需要重新初始化P
					mavlink_and_console_log_info(&mavlink_log_pub,
								     "%sreinit P (%d, %d) negative", msg_label, i, j);
					reinit_P = true;
				}

			} else {
				// copy elememnt from upper triangle to force
				// symmetry
				_P(j, i) = _P(i, j);		// 将上三角复制到下三角，保持P对称
			}

			if (reinit_P) { break; }
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		initP();	// 重新初始化P
	}

	// do prediction
	predict();		// 预测，即状态估计

	// sensor corrections/ initializations		// 下面结合传感器以及其他模块对预测值进行补偿校正
	if (_gpsUpdated) {		// 如果gps有更新
		if (_sensorTimeout & SENSOR_GPS) {		// 超时初始化
			gpsInit();

		} else {
			gpsCorrect();		// gps校正
		}
	}

	if (_baroUpdated) {
		if (_sensorTimeout & SENSOR_BARO) {
			baroInit();

		} else {
			baroCorrect();
		}
	}

	if (_lidarUpdated) {
		if (_sensorTimeout & SENSOR_LIDAR) {
			lidarInit();

		} else {
			lidarCorrect();
		}
	}

	if (_sonarUpdated) {
		if (_sensorTimeout & SENSOR_SONAR) {
			sonarInit();

		} else {
			sonarCorrect();
		}
	}

	if (_flowUpdated) {
		if (_sensorTimeout & SENSOR_FLOW) {
			flowInit();

		} else {
			flowCorrect();
		}
	}

	if (_visionUpdated) {
		if (_sensorTimeout & SENSOR_VISION) {
			visionInit();

		} else {
			visionCorrect();
		}
	}

	if (_mocapUpdated) {
		if (_sensorTimeout & SENSOR_MOCAP) {
			mocapInit();

		} else {
			mocapCorrect();
		}
	}

	if (_landUpdated) {
		if (_sensorTimeout & SENSOR_LAND) {
			landInit();

		} else {
			landCorrect();
		}
	}

	if (targetPositionUpdated) {
		if (_sensorTimeout & SENSOR_LAND_TARGET) {
			landingTargetInit();

		} else {
			landingTargetCorrect();
		}
	}

/**********************发布信息**************************/
	if (_altOriginInitialized) {
		// update all publications if possible
		publishLocalPos();		// 发布local position
		publishEstimatorStatus();		// 发布十个状态量
		_pub_innov.get().timestamp = _timeStamp;		// 发布时间
		_pub_innov.update();		// 更新公告、订阅、发布的句柄

		if ((_estimatorInitialized & EST_XY) && (_map_ref.init_done || _fake_origin.get())) {
			publishGlobalPos();			// 发布global position
		}
	}

	// 更新延迟信息
	// 注意： 下面的dt_hist 是接收传感器信息的时间间隔，而前面在update 函数最开始定义的dt是执行update函数的时间周期，注意区分两者
	// propagate delayed state, no matter what
	// if state is frozen, delayed state still
	// needs to be propagated with frozen state
	float dt_hist = 1.0e-6f * (_timeStamp - _time_last_hist);	// 本次的时间减去上一次记录的时间 (us * 10^-6 = s)

	if (_time_last_hist == 0 ||
	    (dt_hist > HIST_STEP)) {	// 首次 或间隔大于 0.05s
		_tDelay.update(Scalar<uint64_t>(_timeStamp));		
		_xDelay.update(_x);
		_time_last_hist = _timeStamp;
	}
}

void BlockLocalPositionEstimator::checkTimeouts()		// 检查是否超时
{
	baroCheckTimeout();
	gpsCheckTimeout();
	lidarCheckTimeout();
	flowCheckTimeout();
	sonarCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
	landCheckTimeout();
	landingTargetCheckTimeout();
}

// 判断是否着陆
// 判断方法： 根据vehicle_land_detected 这个toptic 
// 这个topic 中有 bool landed、 bool freefall、 bool ground_contact、 bool maybe_landed四个布尔量
bool BlockLocalPositionEstimator::landed()
{
	if (!(_fusion.get() & FUSE_LAND)) {
		return false;
	}

	bool disarmed_not_falling = (!_sub_armed.get().armed) && (!_sub_land.get().freefall);

	return _sub_land.get().landed || disarmed_not_falling;
}

// 发布本地位置
void BlockLocalPositionEstimator::publishLocalPos()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(_P(X_vx, X_vx) + _P(X_vy, X_vy));		//求vx、vy的标准差
	float evv = sqrtf(_P(X_vz, X_vz));		// 计算vx的标准差
	float eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));		// 计算水平位置误差的标准差
	float epv = sqrtf(_P(X_z, X_z));		// 计算垂直位置误差的标准差

	float eph_thresh = 3.0f;		// eph 的阈值
	float epv_thresh = 3.0f;		// epv 的阈值

	if (evh < _vxy_pub_thresh.get()) {		// 根据两个阈值限定eph和epv
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	// publish local position
	if (PX4_ISFINITE(_x(X_x)) && PX4_ISFINITE(_x(X_y)) && PX4_ISFINITE(_x(X_z)) &&
	    PX4_ISFINITE(_x(X_vx)) && PX4_ISFINITE(_x(X_vy))
	    && PX4_ISFINITE(_x(X_vz))) {		// 如果所有数据有效
		_pub_lpos.get().timestamp = _timeStamp;

		_pub_lpos.get().xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().z_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().v_xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().v_z_valid = _estimatorInitialized & EST_Z;

		_pub_lpos.get().x = xLP(X_x);	// north 	// 发布x(北)方向的距离数据
		_pub_lpos.get().y = xLP(X_y);	// east		// 发布y(东)方向的距离数据

		if (_fusion.get() & FUSE_PUB_AGL_Z) {		// 判断是要发布相对高度还是绝对高度
			_pub_lpos.get().z = -_aglLowPass.getState();	// agl		// 发布相对地面的高度(agl) 因为坐标系为东北地，所以要加负号

		} else {
			_pub_lpos.get().z = xLP(X_z);	// down		// 发布绝对高度
		}

		_pub_gpos.get().yaw = matrix::Eulerf(matrix::Quatf(_sub_att.get().q)).psi();		// 发布偏航角速度

		_pub_lpos.get().vx = xLP(X_vx);		// north		// 发布 x(北)方向的速度
		_pub_lpos.get().vy = xLP(X_vy);		// east			// 发布 y(东)方向的速度
		_pub_lpos.get().vz = xLP(X_vz);		// down			// 发布 z(下)方向的速度

		// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		_pub_lpos.get().z_deriv = xLP(X_vz);		// 发布z方向的位置变化率，用X_vz代替

		_pub_lpos.get().ax = _u(U_ax);		// north
		_pub_lpos.get().ay = _u(U_ay);		// east
		_pub_lpos.get().az = _u(U_az);		// down

		_pub_lpos.get().xy_global = _estimatorInitialized & EST_XY;		// 发布xy方向的global position 信息的有效性
		_pub_lpos.get().z_global = !(_sensorTimeout & SENSOR_BARO) && _altOriginGlobal;		// 发布z方向的global position 信息的有效性
		_pub_lpos.get().ref_timestamp = _time_origin;		// 发布更新gps时的时间点
		_pub_lpos.get().ref_lat = _map_ref.lat_rad * 180 / M_PI;		// 将纬度信息转换为°的形式发布出去
		_pub_lpos.get().ref_lon = _map_ref.lon_rad * 180 / M_PI;		// 将经度信息转换为°的形式发布出去
		_pub_lpos.get().ref_alt = _altOrigin;		// 发布原点（参考点）的海拔高度
		_pub_lpos.get().dist_bottom = _aglLowPass.getState();		// 发布飞机下表面到地面的距离信息
		_pub_lpos.get().dist_bottom_rate = -xLP(X_vz);		// 发布向下的距离变化率
		// we estimate agl even when we don't have terrain info
		// if you are in terrain following mode this is important
		// so that if terrain estimation fails there isn't a
		// sudden altitude jump
		_pub_lpos.get().dist_bottom_valid = _estimatorInitialized & EST_Z;		// 发布向下距离信息的有效性
		_pub_lpos.get().eph = eph;			// 发布水平位置误差的标准差
		_pub_lpos.get().epv = epv;			// 发布垂直位置误差的标准差
		_pub_lpos.get().evh = evh;			// 发布水平速度误差的标准差
		_pub_lpos.get().evv = evv;			// 发布垂直速度误差的标准差
		_pub_lpos.get().vxy_max = INFINITY;
		_pub_lpos.get().vz_max = INFINITY;
		_pub_lpos.get().hagl_min = INFINITY;
		_pub_lpos.get().hagl_max = INFINITY;
		_pub_lpos.update();		// 更新发布
	}
}

// 发布估计的状态
void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	_pub_est_status.get().timestamp = _timeStamp;		// 发布时间点

	for (size_t i = 0; i < n_x; i++) {
		_pub_est_status.get().states[i] = _x(i);		// 发布十个状态量
	}

	// matching EKF2 covariances indexing
	// quaternion - not determined, as it is a position estimator		//四元数 位置估计未用到
	_pub_est_status.get().covariances[0] = NAN;
	_pub_est_status.get().covariances[1] = NAN;
	_pub_est_status.get().covariances[2] = NAN;
	_pub_est_status.get().covariances[3] = NAN;
	// linear velocity		// 线性速度
	_pub_est_status.get().covariances[4] = _P(X_vx, X_vx);
	_pub_est_status.get().covariances[5] = _P(X_vy, X_vy);
	_pub_est_status.get().covariances[6] = _P(X_vz, X_vz);
	// position		
	_pub_est_status.get().covariances[7] = _P(X_x, X_x);
	_pub_est_status.get().covariances[8] = _P(X_y, X_y);
	_pub_est_status.get().covariances[9] = _P(X_z, X_z);
	// gyro bias - not determined			// 陀螺仪偏差 未用到
	_pub_est_status.get().covariances[10] = NAN;
	_pub_est_status.get().covariances[11] = NAN;
	_pub_est_status.get().covariances[12] = NAN;
	// accel bias
	_pub_est_status.get().covariances[13] = _P(X_bx, X_bx);
	_pub_est_status.get().covariances[14] = _P(X_by, X_by);
	_pub_est_status.get().covariances[15] = _P(X_bz, X_bz);

	// mag - not determined
	for (size_t i = 16; i <= 21; i++) {
		_pub_est_status.get().covariances[i] = NAN;
	}

	// replacing the hor wind cov with terrain altitude covariance
	_pub_est_status.get().covariances[22] = _P(X_tz, X_tz);		// 用地形高度协方差代替水平风协方差
	_pub_est_status.get().covariances[23] = NAN;

	_pub_est_status.get().n_states = n_x;		// 发布状态量的个数
	_pub_est_status.get().health_flags = _sensorFault;		// 发布传感器的健康状态
	_pub_est_status.get().timeout_flags = _sensorTimeout;		// 发布超时flag
	_pub_est_status.get().pos_horiz_accuracy = _pub_gpos.get().eph;		// 相对于原点的水平位移
	_pub_est_status.get().pos_vert_accuracy = _pub_gpos.get().epv;		// 相对于原点的垂直位移

	_pub_est_status.update();		// 发布更新
}

// 发布 global position
void BlockLocalPositionEstimator::publishGlobalPos()
{
	// publish global position
	double lat = 0;
	double lon = 0;
	const Vector<float, n_x> &xLP = _xLowPass.getState();
	map_projection_reproject(&_map_ref, xLP(X_x), xLP(X_y), &lat, &lon);		// 将平面坐标转换为球坐标
	float alt = -xLP(X_z) + _altOrigin;		// 绝对高度= 地面海拔 + 相对高度， 因为xLP(X_z)为负值，所以加负号转为正

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(_P(X_vx, X_vx) + _P(X_vy, X_vy));		// 计算水平速度的标准偏差
	float eph = sqrtf(_P(X_x, X_x) + _P(X_y, X_y));		// 计算水平位置的标准偏差
	float epv = sqrtf(_P(X_z, X_z));		// 计算垂直位置的标准偏差

	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (evh < _vxy_pub_thresh.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) &&
	    PX4_ISFINITE(xLP(X_vx)) && PX4_ISFINITE(xLP(X_vy)) &&
	    PX4_ISFINITE(xLP(X_vz))) {
		_pub_gpos.get().timestamp = _timeStamp;
		_pub_gpos.get().lat = lat;
		_pub_gpos.get().lon = lon;
		_pub_gpos.get().alt = alt;
		_pub_gpos.get().vel_n = xLP(X_vx);
		_pub_gpos.get().vel_e = xLP(X_vy);
		_pub_gpos.get().vel_d = xLP(X_vz);
		_pub_gpos.get().yaw = matrix::Eulerf(matrix::Quatf(_sub_att.get().q)).psi();
		_pub_gpos.get().eph = eph;
		_pub_gpos.get().epv = epv;
		_pub_gpos.get().terrain_alt = _altOrigin - xLP(X_tz);
		_pub_gpos.get().terrain_alt_valid = _estimatorInitialized & EST_TZ;
		_pub_gpos.get().dead_reckoning = !(_estimatorInitialized & EST_XY);
		_pub_gpos.update();
	}
}

// 初始化协方差矩阵P
void BlockLocalPositionEstimator::initP()
{
	_P.setZero();		// 将P清零
	// initialize to twice valid condition
	_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;		// = 8
	_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	_P(X_vx, X_vx) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	_P(X_vy, X_vy) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	// use vxy thresh for vz init as well
	_P(X_vz, X_vz) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	// initialize bias uncertainty to small values to keep them stable
	_P(X_bx, X_bx) = 1e-6;
	_P(X_by, X_by) = 1e-6;
	_P(X_bz, X_bz) = 1e-6;
	_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
}

// 初始化 A、 B、 Q、 R
void BlockLocalPositionEstimator::initSS()
{
	initP();

	// dynamics matrix
	_A.setZero();
	// derivative of position is velocity
	_A(X_x, X_vx) = 1;
	_A(X_y, X_vy) = 1;
	_A(X_z, X_vz) = 1;

	// input matrix
	_B.setZero();
	_B(X_vx, U_ax) = 1;
	_B(X_vy, U_ay) = 1;
	_B(X_vz, U_az) = 1;

	// update components that depend on current state
	updateSSStates();
	updateSSParams();
}

// 设置A
void BlockLocalPositionEstimator::updateSSStates()
{
	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	_A(X_vx, X_bx) = -_R_att(0, 0);		// 速度的微分就是加速度计的加速度信息减去偏差
	_A(X_vx, X_by) = -_R_att(0, 1);
	_A(X_vx, X_bz) = -_R_att(0, 2);

	_A(X_vy, X_bx) = -_R_att(1, 0);
	_A(X_vy, X_by) = -_R_att(1, 1);
	_A(X_vy, X_bz) = -_R_att(1, 2);

	_A(X_vz, X_bx) = -_R_att(2, 0);
	_A(X_vz, X_by) = -_R_att(2, 1);
	_A(X_vz, X_bz) = -_R_att(2, 2);
}

// 设置R、Q
void BlockLocalPositionEstimator::updateSSParams()
{
	// input noise covariance matrix
	_R.setZero();
	_R(U_ax, U_ax) = _accel_xy_stddev.get() * _accel_xy_stddev.get();		// 平方
	_R(U_ay, U_ay) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
	_R(U_az, U_az) = _accel_z_stddev.get() * _accel_z_stddev.get();

	// process noise power matrix
	_Q.setZero();
	float pn_p_sq = _pn_p_noise_density.get() * _pn_p_noise_density.get();		// 平方
	float pn_v_sq = _pn_v_noise_density.get() * _pn_v_noise_density.get();
	_Q(X_x, X_x) = pn_p_sq;
	_Q(X_y, X_y) = pn_p_sq;
	_Q(X_z, X_z) = pn_p_sq;
	_Q(X_vx, X_vx) = pn_v_sq;
	_Q(X_vy, X_vy) = pn_v_sq;
	_Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	float pn_b_sq = _pn_b_noise_density.get() * _pn_b_noise_density.get();
	_Q(X_bx, X_bx) = pn_b_sq;
	_Q(X_by, X_by) = pn_b_sq;
	_Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	float pn_t_noise_density =
		_pn_t_noise_density.get() +
		(_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
	_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
}

// 预测
void BlockLocalPositionEstimator::predict()
{
	// get acceleration
	_R_att = matrix::Dcm<float>(matrix::Quatf(_sub_att.get().q));		// q来自vehicle_attitude这个topic，是经过传感器数据融合后修正的q
	Vector3f a(_sub_sensor.get().accelerometer_m_s2);		// a 向量里是加速度信息
	// note, bias is removed in dynamics function
	_u = _R_att * a;		// 转换到大地系
	_u(U_az) += CONSTANTS_ONE_G;	// add g		// 地理坐标系下的z轴加速度是有重力加速度的，因此补偿上去

	// update state space based on new states
	updateSSStates();		// 更新系统状态空间转移矩阵，即A矩阵

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = getDt();		// return _dt. 获取迭代步长
	Vector<float, n_x> k1, k2, k3, k4;		// 这里可以看出用到了四阶的龙格库塔，k1到k4表示取中间的4个点的斜率
	k1 = dynamics(0, _x, _u);		// dynamics 是现代控制理论中的动态方程，是一个一阶微分方程
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	// don't integrate position if no valid xy data
	if (!(_estimatorInitialized & EST_XY))  {
		dx(X_x) = 0;
		dx(X_vx) = 0;
		dx(X_y) = 0;
		dx(X_vy) = 0;
	}

	// don't integrate z if no valid z data
	if (!(_estimatorInitialized & EST_Z))  {
		dx(X_z) = 0;
	}

	// don't integrate tz if no valid tz data
	if (!(_estimatorInitialized & EST_TZ))  {
		dx(X_tz) = 0;
	}

	// saturate bias		// 计算偏差
	float bx = dx(X_bx) + _x(X_bx);
	float by = dx(X_by) + _x(X_by);
	float bz = dx(X_bz) + _x(X_bz);

	if (std::abs(bx) > BIAS_MAX) {		// 如果偏差大于最大，则以最大偏差 - 此次_x(X_bx)(以最大偏差剔除此次偏差作为偏差变化率)
		bx = BIAS_MAX * bx / std::abs(bx);
		dx(X_bx) = bx - _x(X_bx);
	}

	if (std::abs(by) > BIAS_MAX) {
		by = BIAS_MAX * by / std::abs(by);
		dx(X_by) = by - _x(X_by);
	}

	if (std::abs(bz) > BIAS_MAX) {
		bz = BIAS_MAX * bz / std::abs(bz);
		dx(X_bz) = bz - _x(X_bz);
	}

	// propagate
	_x += dx;		// _x就是下一时刻的预测值，接下来的任务就是对它进行校正
	// 下面是p的一阶微分方程，解这个方程的时候用的是最简单的欧拉法  transpose-- 转置
	Matrix<float, n_x, n_x> dP = (_A * _P + _P * _A.transpose() +
				      _B * _R * _B.transpose() + _Q) * getDt();

	// covariance propagation logic
	for (size_t i = 0; i < n_x; i++) {
		if (_P(i, i) > P_MAX) {		// 如果对角线元素大于P_MAX(10^6),则将该行和列的dp置0（不再累加）
			// if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for (size_t j = 0; j < n_x; j++) {
				dP(i, j) = 0;
				dP(j, i) = 0;
			}
		}
	}

	_P += dP;
	_xLowPass.update(_x);		// 将_x 低通处理后存在_xLowPass里的state中
	_aglLowPass.update(agl());		// 将agl低通处理后存在_aglLowPass的state中
}

int BlockLocalPositionEstimator::getDelayPeriods(float delay, uint8_t *periods)
{
	float t_delay = 0;
	uint8_t i_hist = 0;

	for (i_hist = 1; i_hist < HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > delay) {
			break;
		}
	}

	*periods = i_hist;

	if (t_delay > DELAY_MAX) {
		mavlink_and_console_log_info(&mavlink_log_pub, "%sdelayed data old: %8.4f", msg_label, double(t_delay));
		return -1;
	}

	return OK;
}
