/*
PID controller to calculate:
# aim_steer, in deg
# aim_throttle
# aim_brake
for the vehicle dynamics model in carla, or in Carsim
according to the input of tiev-plus:
# aim_acc in m/s^2
# aim_steer of steering wheel, in deg

2020.7.1 by leoherz_liu@163.com
*/
#pragma once

#include <list>
#include <string>
#include <cmath>

#include "rapidjson/filereadstream.h"
#include "rapidjson/document.h"

struct PIDParaSet
{
	double kp;
	double ki;
	double kd;
};

struct PIDDeadZone
{
	double dzone_acc_error_throttle;
	double dzone_acc_error_brake;
	double dzone_steer_error;
};

struct PIDLimits
{
	double min_throttle;
	double min_brake;
	double min_steer;
	double max_throttle;
	double max_brake;
	double max_steer;
};

struct Control
{
	double steer;
	double throttle;
	double brake;
};

class PIDController
{
public:
	PIDController(const std::string &filename);
	void tick(double aim_acc, double aim_steer, double veh_acc, double veh_steer, bool direct_steer = true);

public:
	Control control;

private:
	int _integr_update_period;
	double _time_step;
	static int _step_count;
	PIDParaSet _para_steer;
	PIDParaSet _para_throttle;
	PIDParaSet _para_brake;
	PIDDeadZone _para_dzone;
	PIDLimits _para_limits;
	std::list<double> _error_acc;
	std::list<double> _error_steer;
	double _integr_error_acc;
	double _integr_error_steer;

private:
	void pull_parameters(const std::string &filename);
	void integral(double &res, double val);
	double differential(const std::list<double> &vals);
	void longitudinal_control(double aim_acc, double veh_acc);
	void lateral_control(double aim_steer, double veh_steer, bool direct_steer);
};