#include "PIDController.h"

int PIDController::_step_count;

PIDController::PIDController(const std::string &filename)
{
	pull_parameters(filename);
	control.brake = _para_limits.max_brake;
	control.throttle = 0;
	control.steer = 0;
}

void PIDController::tick(double aim_acc, double aim_steer, double veh_acc, double veh_steer, bool direct_steer)
{
	longitudinal_control(aim_acc, veh_acc);
	lateral_control(aim_steer, veh_steer, direct_steer);
	_step_count += 1;
}

void PIDController::pull_parameters(const std::string &filename)
{
	FILE *fp = fopen(filename.c_str(), "r");
	if (fp == 0)
	{
		printf("cannot open pid_parameters.json.\n");
		return;
	}
	printf("controller use pid_parameters.json.\n");
	char readBuffer[1024];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	rapidjson::Document d;
	d.ParseStream(is);
	int freq = d["frequency"].GetInt();
	freq = freq <= 20 ? 20 : freq;
	_time_step = 1.0 / (double)freq;
	_step_count = 0;
	_integr_error_acc = 0;
	_integr_error_steer = 0;
	_integr_update_period = d["integration_update_period"].GetInt();
	_para_brake.kp = d["kp_brake"].GetDouble();
	_para_brake.ki = d["ki_brake"].GetDouble();
	_para_brake.kd = d["kd_brake"].GetDouble();
	_para_steer.kp = d["kp_steer"].GetDouble();
	_para_steer.ki = d["ki_steer"].GetDouble();
	_para_steer.kd = d["kd_steer"].GetDouble();
	_para_throttle.kp = d["kp_throttle"].GetDouble();
	_para_throttle.ki = d["ki_throttle"].GetDouble();
	_para_throttle.kd = d["kd_throttle"].GetDouble();
	_para_dzone.dzone_acc_error_throttle = d["deadzone_acc_error_throttle"].GetDouble();
	_para_dzone.dzone_acc_error_brake = d["deadzone_acc_error_brake"].GetDouble();
	_para_dzone.dzone_steer_error = d["deadzone_steer_error"].GetDouble();
	_para_limits.min_brake = d["min_brake"].GetDouble();
	_para_limits.min_throttle = d["min_throttle"].GetDouble();
	_para_limits.min_steer = d["min_steer"].GetDouble();
	_para_limits.max_brake = d["max_brake"].GetDouble();
	_para_limits.max_throttle = d["max_throttle"].GetDouble();
	_para_limits.max_steer = d["max_steer"].GetDouble();
	fclose(fp);
}

void PIDController::longitudinal_control(double aim_acc, double veh_acc)
{
	double error = aim_acc - veh_acc;
	_error_acc.push_back(error);
	while (_error_acc.size() > 3)
	{
		_error_acc.pop_front();
	}
	integral(_integr_error_acc, error);
	double diff = differential(_error_acc);

	if (error > _para_dzone.dzone_acc_error_throttle)
	{
		control.brake = 0;
		control.throttle = _para_throttle.kp * error + _para_throttle.ki * _integr_error_acc + _para_throttle.kd * diff;
		control.throttle = control.throttle > _para_limits.min_throttle ? control.throttle : 0;
		control.throttle = control.throttle < _para_limits.max_throttle ? control.throttle : _para_limits.max_throttle;
	}
	else if (error < _para_dzone.dzone_acc_error_brake)
	{
		control.throttle = 0;
		control.brake = -_para_brake.kp * error - _para_brake.ki * _integr_error_acc - _para_brake.kd * diff;
		control.brake = control.brake > _para_limits.min_brake ? control.brake : 0;
		control.brake = control.brake < _para_limits.max_brake ? control.brake : _para_limits.max_brake;
	}
}

void PIDController::lateral_control(double aim_steer, double veh_steer, bool direct_steer)
{
	double error = aim_steer - veh_steer;
	_error_steer.push_back(error);
	while (_error_steer.size() > 3)
	{
		_error_steer.pop_front();
	}
	integral(_integr_error_steer, error);
	double diff = differential(_error_steer);

	if (direct_steer)
	{
		control.steer = _para_steer.kp * aim_steer;
		control.steer = control.steer > _para_limits.min_steer ? control.steer : 0;
		if (fabs(control.steer) > _para_limits.max_steer)
		{
			control.steer = control.steer > 0 ? _para_limits.max_steer : -_para_limits.max_steer;
		}
		return;
	}

	if (fabs(error) > _para_dzone.dzone_steer_error)
	{
		control.steer = _para_steer.kp * error + _para_steer.ki * _integr_error_steer + _para_steer.kd * diff;
		control.steer = fabs(control.steer) > _para_limits.min_steer ? control.steer : 0;
		if (fabs(control.steer) > _para_limits.max_steer)
		{
			control.steer = control.steer > 0 ? _para_limits.max_steer : -_para_limits.max_steer;
		}
	}
}

void PIDController::integral(double &res, double val)
{
	if (_step_count % static_cast<int>(_integr_update_period / _time_step) == 0)
	{
		res = 0;
	}
	else
	{
		res += val * _time_step;
	}
	return;
}

double PIDController::differential(const std::list<double> &vals)
{
	double result = 0;
	int len = vals.size();
	double vals_array[3];
	std::list<double>::const_iterator it = vals.begin();
	std::list<double>::const_iterator end_it = vals.end();
	for (int i = 0; it != end_it; ++it, ++i)
	{
		vals_array[i] = *it;
	}
	if (len >= 3)
	{
		// 3-point backwards
		result = (vals_array[len - 3] - 4 * vals_array[len - 2] + 3 * vals_array[len - 1]) / (2 * _time_step);
	}
	else if (len >= 2)
	{
		// 2-point backwards
		result = (vals_array[len - 1] - vals_array[len - 2]) / _time_step;
	}
	else
	{
		// no enough points
		result = 0;
	}
	return result;
}