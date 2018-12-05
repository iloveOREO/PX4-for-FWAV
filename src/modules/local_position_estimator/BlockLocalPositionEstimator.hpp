#pragma once

#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <px4_module_params.h>		// C++ 里的base class， 用于其他的参数使用配置参数
#include <controllib/blocks.hpp>
#include <mathlib/mathlib.h>		// 数学库
#include <lib/ecl/geo/geo.h>
#include <matrix/Matrix.hpp>		// 矩阵库

// uORB Subscriptions		// 下面是本程序的订阅和发布的信息流  一些订阅的topic
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>		// 飞机的状态
#include <uORB/topics/actuator_armed.h>		// 执行机构的armed状态
#include <uORB/topics/vehicle_land_detected.h>		// 检查是否着陆
#include <uORB/topics/vehicle_control_mode.h>		// 控制模式
#include <uORB/topics/vehicle_attitude.h>		// 飞机的姿态
#include <uORB/topics/vehicle_attitude_setpoint.h>		// 设置飞机的姿态
#include <uORB/topics/optical_flow.h>		// 光流
#include <uORB/topics/sensor_combined.h>		// 传感器数据
#include <uORB/topics/distance_sensor.h>		// 距离传感器
#include <uORB/topics/parameter_update.h>		// 参数更新
#include <uORB/topics/manual_control_setpoint.h>		// 手动控制下的一些设定值
#include <uORB/topics/vehicle_gps_position.h>		// 飞行器GPS的位置
#include <uORB/topics/landing_target_pose.h>		// 着陆点的相对位置
#include <uORB/topics/vehicle_air_data.h>		// 空速计数据
#include <uORB/topics/vehicle_odometry.h>		// odom 数据

// uORB Publications		// 发布的消息
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_status.h>		// 估计器的状态，这里的估计器包括之前用于位置姿态估计的各种传感器以及其他模块
#include <uORB/topics/ekf2_innovations.h>

using namespace matrix;
using namespace control;

static const float DELAY_MAX = 0.5f;	// seconds		// 最大延迟0.5s
static const float HIST_STEP = 0.05f;	// 20 hz		// 历史消息发布步长（频率）
static const float BIAS_MAX = 1e-1f;		// 偏差上限
static const size_t HIST_LEN = 10;	// DELAY_MAX / HIST_STEP;
static const size_t N_DIST_SUBS = 4;

// 统计学中的卡方检验：卡方检验就是统计样本的实际观测值与理论推断值之间的偏离程度，
// 实际观测值与理论推断值之间的偏离程度就决定卡方值的大小，卡方值越大，越不符合；卡方值越小，偏差越小，越趋于符合，
//　若两个值完全相等时，卡方值就为０，表示理论值完全符合。
// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
static const float BETA_TABLE[7] = {0,
				    8.82050518214,
				    12.094592431,
				    13.9876612368,
				    16.0875642296,
				    17.8797700658,
				    19.6465647819,
				   };

class BlockLocalPositionEstimator : public control::SuperBlock, public ModuleParams
{
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)		// 系统的状态方程
//	y_i = C_i*x		// 观测方程
//
// kalman filter
//
//	E[xx'] = P		// 估计量的误差的协方差
//	E[uu'] = W		// 系统噪声
//	E[y_iy_i'] = R_i		// 系统噪声协方差矩阵
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
//      ax, ay, az (acceleration NED)
//
// states:
//      px, py, pz , ( position NED, m)
//      vx, vy, vz ( vel NED, m/s),
//      bx, by, bz ( accel bias, m/s^2)
//      tz (terrain altitude, ASL, m)
//
// measurements:
//
//      sonar: pz (measured d*cos(phi)*cos(theta))
//
//      baro: pz
//
//      flow: vx, vy (flow is in body x, y frame)
//
//      gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
//      lidar: pz (actual measured d*cos(phi)*cos(theta))
//
//      vision: px, py, pz, vx, vy, vz
//
//      mocap: px, py, pz
//
//      land (detects when landed)): pz (always measures agl = 0)
//
public:

	// constants
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_vx = 0, Y_flow_vy, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {Y_land_vx = 0, Y_land_vy, Y_land_agl, n_y_land};
	enum {Y_target_x = 0, Y_target_y, n_y_target};
	enum {POLL_FLOW = 0, POLL_SENSORS, POLL_PARAM, n_poll};
	enum {
		FUSE_GPS = 1 << 0,
		FUSE_FLOW = 1 << 1,
		FUSE_VIS_POS = 1 << 2,
		FUSE_LAND_TARGET = 1 << 3,
		FUSE_LAND = 1 << 4,
		FUSE_PUB_AGL_Z = 1 << 5,
		FUSE_FLOW_GYRO_COMP = 1 << 6,
		FUSE_BARO = 1 << 7
	};

	enum sensor_t {
		SENSOR_BARO = 1 << 0,
		SENSOR_GPS = 1 << 1,
		SENSOR_LIDAR = 1 << 2,
		SENSOR_FLOW = 1 << 3,
		SENSOR_SONAR = 1 << 4,
		SENSOR_VISION = 1 << 5,
		SENSOR_MOCAP = 1 << 6,
		SENSOR_LAND = 1 << 7,
		SENSOR_LAND_TARGET = 1 << 8,
	};

	enum estimate_t {
		EST_XY = 1 << 0,
		EST_Z = 1 << 1,
		EST_TZ = 1 << 2,
	};

	// public methods
	BlockLocalPositionEstimator();		// BlockLocalPositionEsitimator 类的构造函数
	void update();		// 对飞机的状态先验估计值进行补偿校正
	virtual ~BlockLocalPositionEstimator() = default;		// 析构函数

private:
	BlockLocalPositionEstimator(const BlockLocalPositionEstimator &) = delete;
	BlockLocalPositionEstimator operator=(const BlockLocalPositionEstimator &) = delete;

	// methods
	// ----------------------------
	//
	Vector<float, n_x> dynamics(		// 动态方程，形式为：x' = _A * x + _B * u，这是一个一阶微分方程，也就是描述系统状态空间的状态方程
										//　区分kf中的 x_(k) = A*x_(k-1) + B*u_(k-1)
										// 本程序中的状态估计用的是这个一阶微分方程结合龙格库塔，而不是用的kf中的第一个方程，因为这是一个连续系统
		float t,
		const Vector<float, n_x> &x,
		const Vector<float, n_u> &u);
	void initP();		// 初始化状态协方差矩阵P
	void initSS();		// 这个函数包括了下面的两个函数，执行这个函数的同时也就执行下面的两个
	void updateSSStates();		// 设置A
	void updateSSParams();		// 设置R、Q

	// predict the next state
	void predict();		// 预测下一时刻的空间状态

	// lidar
	int  lidarMeasure(Vector<float, n_y_lidar> &y);		// 数据测量
	void lidarCorrect();		// 将之前用predict()预测的状态结合雷达数据进行校正
	void lidarInit();		// 雷达的初始化
	void lidarCheckTimeout();		// 检查超时

	// sonar
	int  sonarMeasure(Vector<float, n_y_sonar> &y);
	void sonarCorrect();
	void sonarInit();
	void sonarCheckTimeout();

	// baro
	int  baroMeasure(Vector<float, n_y_baro> &y);		// 气压计数据测量
	void baroCorrect();
	void baroInit();
	void baroCheckTimeout();

	// gps
	int  gpsMeasure(Vector<double, n_y_gps> &y);		// 数据测量
	void gpsCorrect();		// 将之前用predict()预测的状态结合GPS数据进行校正
	void gpsInit();		// gps初始化
	void gpsCheckTimeout();		// GPS检查超时

	// flow
	int  flowMeasure(Vector<float, n_y_flow> &y);
	void flowCorrect();
	void flowInit();
	void flowCheckTimeout();

	// vision
	int  visionMeasure(Vector<float, n_y_vision> &y);
	void visionCorrect();
	void visionInit();
	void visionCheckTimeout();

	// mocap
	int  mocapMeasure(Vector<float, n_y_mocap> &y);
	void mocapCorrect();
	void mocapInit();
	void mocapCheckTimeout();

	// land
	int  landMeasure(Vector<float, n_y_land> &y);
	void landCorrect();
	void landInit();
	void landCheckTimeout();

	// landing target
	int  landingTargetMeasure(Vector<float, n_y_target> &y);
	void landingTargetCorrect();
	void landingTargetInit();
	void landingTargetCheckTimeout();

	// timeouts
	void checkTimeouts();		// 检查超时

	// misc
	inline float agl()		// 用于检测是否着陆，着陆时agl=0
	{
		return _x(X_tz) - _x(X_z);		// _x(X_tz)是地面到原点的z，_x(X_z)是飞行器到原点的z
	}
	bool landed();
	int getDelayPeriods(float delay, uint8_t *periods);		// 获取延迟时间长度

	// publications		// 发布信息函数
	void publishLocalPos();
	void publishGlobalPos();
	void publishEstimatorStatus();

	// attributes
	// ----------------------------

	// subscriptions
	uORB::Subscription<actuator_armed_s> _sub_armed;
	uORB::Subscription<vehicle_land_detected_s> _sub_land;
	uORB::Subscription<vehicle_attitude_s> _sub_att;
	uORB::Subscription<optical_flow_s> _sub_flow;
	uORB::Subscription<sensor_combined_s> _sub_sensor;
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<vehicle_gps_position_s> _sub_gps;
	uORB::Subscription<vehicle_odometry_s> _sub_visual_odom;
	uORB::Subscription<vehicle_odometry_s> _sub_mocap_odom;
	uORB::Subscription<distance_sensor_s> _sub_dist0;
	uORB::Subscription<distance_sensor_s> _sub_dist1;
	uORB::Subscription<distance_sensor_s> _sub_dist2;
	uORB::Subscription<distance_sensor_s> _sub_dist3;
	uORB::Subscription<distance_sensor_s> *_dist_subs[N_DIST_SUBS];
	uORB::Subscription<distance_sensor_s> *_sub_lidar;
	uORB::Subscription<distance_sensor_s> *_sub_sonar;
	uORB::Subscription<landing_target_pose_s> _sub_landing_target_pose;
	uORB::Subscription<vehicle_air_data_s> _sub_airdata;

	// publications
	uORB::Publication<vehicle_local_position_s> _pub_lpos;
	uORB::Publication<vehicle_global_position_s> _pub_gpos;
	uORB::Publication<estimator_status_s> _pub_est_status;
	uORB::Publication<ekf2_innovations_s> _pub_innov;

	// map projection
	struct map_projection_reference_s _map_ref;		// 地图构建的参考。这个结构体在基于GPS的位置控制中有用到，
													// 用于map_projection_project函数和map_projection_global函数，
													// 前者将地理学坐标系(geographic coordinate system)中的点(球)投影到本地方位等距平面(XOY)中
													// 后者是将本地方位等距平面中的点投影到地理学坐标系，也就是球面坐标和平面坐标之间的转换


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   /**< example parameter */

		// general parameters
		(ParamInt<px4::params::LPE_FUSION>) _fusion,
		(ParamFloat<px4::params::LPE_VXY_PUB>) _vxy_pub_thresh,
		(ParamFloat<px4::params::LPE_Z_PUB>) _z_pub_thresh,

		// sonar parameters
		(ParamFloat<px4::params::LPE_SNR_Z>) _sonar_z_stddev,
		(ParamFloat<px4::params::LPE_SNR_OFF_Z>) _sonar_z_offset,

		// lidar parameters
		(ParamFloat<px4::params::LPE_LDR_Z>) _lidar_z_stddev,
		(ParamFloat<px4::params::LPE_LDR_OFF_Z>) _lidar_z_offset,

		// accel parameters
		(ParamFloat<px4::params::LPE_ACC_XY>) _accel_xy_stddev,
		(ParamFloat<px4::params::LPE_ACC_Z>) _accel_z_stddev,

		// baro parameters
		(ParamFloat<px4::params::LPE_BAR_Z>) _baro_stddev,

		// gps parameters
		(ParamFloat<px4::params::LPE_GPS_DELAY>) _gps_delay,
		(ParamFloat<px4::params::LPE_GPS_XY>) _gps_xy_stddev,
		(ParamFloat<px4::params::LPE_GPS_Z>) _gps_z_stddev,
		(ParamFloat<px4::params::LPE_GPS_VXY>) _gps_vxy_stddev,
		(ParamFloat<px4::params::LPE_GPS_VZ>) _gps_vz_stddev,
		(ParamFloat<px4::params::LPE_EPH_MAX>) _gps_eph_max,
		(ParamFloat<px4::params::LPE_EPV_MAX>) _gps_epv_max,

		// vision parameters
		(ParamFloat<px4::params::LPE_VIS_XY>) _vision_xy_stddev,
		(ParamFloat<px4::params::LPE_VIS_Z>) _vision_z_stddev,
		(ParamFloat<px4::params::LPE_VIS_DELAY>) _vision_delay,

		// mocap parameters
		(ParamFloat<px4::params::LPE_VIC_P>) _mocap_p_stddev,

		// flow parameters
		(ParamFloat<px4::params::LPE_FLW_OFF_Z>) _flow_z_offset,
		(ParamFloat<px4::params::LPE_FLW_SCALE>) _flow_scale,
		(ParamInt<px4::params::LPE_FLW_QMIN>) _flow_min_q,
		(ParamFloat<px4::params::LPE_FLW_R>) _flow_r,
		(ParamFloat<px4::params::LPE_FLW_RR>) _flow_rr,

		// land parameters
		(ParamFloat<px4::params::LPE_LAND_Z>) _land_z_stddev,
		(ParamFloat<px4::params::LPE_LAND_VXY>) _land_vxy_stddev,

		// process noise
		(ParamFloat<px4::params::LPE_PN_P>) _pn_p_noise_density,
		(ParamFloat<px4::params::LPE_PN_V>) _pn_v_noise_density,
		(ParamFloat<px4::params::LPE_PN_B>) _pn_b_noise_density,
		(ParamFloat<px4::params::LPE_PN_T>) _pn_t_noise_density,
		(ParamFloat<px4::params::LPE_T_MAX_GRADE>) _t_max_grade,

		(ParamFloat<px4::params::LPE_LT_COV>) _target_min_cov,
		(ParamInt<px4::params::LTEST_MODE>) _target_mode,

		// init origin
		(ParamInt<px4::params::LPE_FAKE_ORIGIN>) _fake_origin,
		(ParamFloat<px4::params::LPE_LAT>) _init_origin_lat,
		(ParamFloat<px4::params::LPE_LON>) _init_origin_lon
	)

	// target mode paramters from landing_target_estimator module
	// 与地面目标有关的参数，比如着陆点是固定还是移动的
	enum TargetMode {
		Target_Moving = 0,		// 目标移动状态
		Target_Stationary = 1		// 目标为静止状态
	};

	// flow gyro filter		// 滤波器 在block类里
	BlockHighPass _flow_gyro_x_high_pass;
	BlockHighPass _flow_gyro_y_high_pass;

	// stats		// 传感器或者其他模块的统计量
	BlockStats<float, n_y_baro> _baroStats;
	BlockStats<float, n_y_sonar> _sonarStats;
	BlockStats<float, n_y_lidar> _lidarStats;
	BlockStats<float, 1> _flowQStats;
	BlockStats<float, n_y_vision> _visionStats;
	BlockStats<float, n_y_mocap> _mocapStats;
	BlockStats<double, n_y_gps> _gpsStats;
	uint16_t _landCount;

	// low pass
	BlockLowPassVector<float, n_x> _xLowPass;
	BlockLowPass _aglLowPass;

	// delay blocks
	BlockDelay<float, n_x, 1, HIST_LEN> _xDelay;
	BlockDelay<uint64_t, 1, 1, HIST_LEN> _tDelay;

	// misc		// 一些时间变量
	px4_pollfd_struct_t _polls[3];
	uint64_t _timeStamp;
	uint64_t _time_origin;
	uint64_t _timeStampLastBaro;
	uint64_t _time_last_hist;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_init_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_mocap;
	uint64_t _time_last_land;
	uint64_t _time_last_target;

	// reference altitudes
	float _altOrigin;		// 原点的海拔
	bool _altOriginInitialized;		// 原点海拔初始化
	bool _altOriginGlobal; // true when the altitude of the origin is defined wrt a global reference frame
	float _baroAltOrigin;		// 原点的气压高度
	float _gpsAltOrigin;		// 原点的gps高度

	// status
	bool _receivedGps;
	bool _lastArmedState;

	// masks
	uint16_t _sensorTimeout;		// 传感器超时时间
	uint16_t _sensorFault;		// 传感器故障
	uint8_t _estimatorInitialized;

	// sensor update flags
	bool _flowUpdated;
	bool _gpsUpdated;
	bool _visionUpdated;
	bool _mocapUpdated;
	bool _lidarUpdated;
	bool _sonarUpdated;
	bool _landUpdated;
	bool _baroUpdated;

	// sensor validation flags
	bool _vision_xy_valid;
	bool _vision_z_valid;
	bool _mocap_xy_valid;
	bool _mocap_z_valid;

	// sensor std deviations
	float _vision_eph;
	float _vision_epv;
	float _mocap_eph;
	float _mocap_epv;

	// local to global coversion related variables
	bool _is_global_cov_init;
	uint64_t _global_ref_timestamp;
	double _ref_lat;
	double _ref_lon;
	float _ref_alt;

	// state space
	Vector<float, n_x>  _x;	// state vector		// 状态向量
	Vector<float, n_u>  _u;	// input vector		// 系统输入量
	Matrix<float, n_x, n_x>  _P;	// state covariance matrix		// 状态协方差矩阵

	matrix::Dcm<float> _R_att;		// 旋转矩阵，为了与下面的输入协方差矩阵_R区别，于是加了_att

	Matrix<float, n_x, n_x>  _A;	// dynamics matrix		// 动态矩阵， 也叫系统矩阵
	Matrix<float, n_x, n_u>  _B;	// input matrix			// 输入矩阵
	Matrix<float, n_u, n_u>  _R;	// input covariance		// 输入的噪声协方差矩阵
	Matrix<float, n_x, n_x>  _Q;	// process noise covariance		// 过程噪声的协方差矩阵
};
