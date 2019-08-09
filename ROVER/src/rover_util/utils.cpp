#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iterator>
#include <sstream>
#include <iostream>
#include <math.h>
#include <cmath> 
#include <float.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <algorithm>
#include "utils.h"
#include "unistd.h"

bool Debug::makeLogFolder()
{
	int try_count = 3;

	std::string str_timestamp = Time::getTimeStamp();

	for (int i = 0; i < try_count; i++)
	{
		const int dir_err = mkdir(LOG_FOLDER, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
		if (dir_err == 0)
		{
			Debug::print(LOG_SUMMARY, "Failed make log folder\r\n");
		}
		else
		{
			Debug::print(LOG_SUMMARY, "Success make log folder\r\n");
			break;
		}

		if (i == try_count - 1)return false;
	}

	std::string log_time_folder = LOG_FOLDER;
	log_time_folder += "/" + str_timestamp;

	for (int i = 0; i < try_count; i++)
	{
		const int dir_time_err = mkdir(log_time_folder.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
		if (dir_time_err == 0)
		{
			Debug::print(LOG_SUMMARY, "Failed make log sub folder\r\n");
		}
		else
		{
			Debug::print(LOG_SUMMARY, "Success make log sub folder\r\n");
			break;
		}

		if (i == try_count - 1)return false;
	}

	return true;
}

void Debug::print(LOG_LEVEL level, const char* fmt, ...)
{
	std::string str_timestamp = Time::getTimeStamp();
	std::string str_log_dir = std::string(LOG_FOLDER);

	static std::string mFilename;
	if (mFilename.length() == 0)
	{
		Filename filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log", ".txt");
		filename.get(mFilename);
	}
#ifndef _LOG_DETAIL
	if (level == LOG_DETAIL)return; //デバッグモードでなければログ出力しない
#endif

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	//画面に出力
	printf(buf);
	//ログファイルに出力
	if (level != LOG_PRINT)
	{
		auto str = std::string(fmt);
		str = "[" + str_timestamp + "] " + str;
		fmt = str.c_str();
		va_list argp;
		va_start(argp, fmt);
		vsprintf(buf, fmt, argp);

		std::ofstream of(mFilename.c_str(), std::ios::out | std::ios::app);
		of << buf;
	}
}

void Debug::typo_print(LOG_LEVEL level, const char * fmt, ...)
{
	std::string str_timestamp = Time::getTimeStamp();
	std::string str_log_dir = std::string(LOG_FOLDER);

	static std::string mFilename;
	if (mFilename.length() == 0)
	{
		Filename filename(str_log_dir + "/" + str_timestamp + "/" + str_timestamp + "_" + "log", ".txt");
		filename.get(mFilename);
	}
#ifndef _LOG_DETAIL
	if (level == LOG_DETAIL)return; //デバッグモードでなければログ出力しない
#endif

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	//画面に出力
	printf(buf);
	//ログファイルに出力
	if (level != LOG_PRINT)
	{
		std::ofstream of(mFilename.c_str(), std::ios::out | std::ios::app);
		of << buf;
	}
}

Debug::Debug()
{
}


void Filename::get(std::string& name)
{
	std::stringstream filename;
	filename << mPrefix << ++mIndex << mSuffix;
	name.assign(filename.str());
}
void Filename::getNow(std::string& name)
{
	std::stringstream filename;
	filename << mPrefix << mIndex << mSuffix;
	name.assign(filename.str());
}
void Filename::getNoIndex(std::string& name)
{
	std::stringstream filename;
	filename << mPrefix << mSuffix;
	name.assign(filename.str());
}
Filename::Filename(const std::string& prefix, const std::string& suffix) : mPrefix(prefix), mSuffix(suffix), mIndex(0)
{
	//撮影インデックスを既存のファイルに上書きしないように変更
	std::string filename;
	struct stat st;
	do
	{
		get(filename);
	} while (stat(filename.c_str(), &st) == 0);
	--mIndex;
}
double Time::dt(const struct timespec& now, const struct timespec& last)
{
	return ((double)(now.tv_sec - last.tv_sec) * 1000000000 + now.tv_nsec - last.tv_nsec) / 1000000000.0;
}
void Time::get(struct timespec& time)
{
	if (clock_gettime(CLOCK_MONOTONIC_RAW, &time) != 0)
	{
		Debug::print(LOG_DETAIL, "FAILED to getAngle time!\r\n");
	}
}
std::string Time::getTimeStamp(bool detail)
{
	struct timeval myTime;    // time_t構造体を定義．1970年1月1日からの秒数を格納するもの
	struct tm *time_st;       // tm構造体を定義．年月日時分秒をメンバ変数に持つ構造体
	gettimeofday(&myTime, NULL);    // 現在時刻を取得してmyTimeに格納．通常のtime_t構造体とsuseconds_tに値が代入される
	time_st = localtime(&myTime.tv_sec);    // time_t構造体を現地時間でのtm構造体に変換

	auto year = time_st->tm_year + 1900;
	auto month = time_st->tm_mon + 1;
	auto day = time_st->tm_mday;
	auto hour = time_st->tm_hour;
	auto min = time_st->tm_min;
	auto sec = time_st->tm_sec;
	auto usec = myTime.tv_usec;
	/*
		time_t now;
		struct tm *ts;
		now = time(NULL);
		gettimeofday(&now, NULL);
		ts = localtime(&now.tv_sec);

		auto year = ts->tm_year + 1900;
		auto month = ts->tm_mon + 1;
		auto day = ts->tm_mday;
		auto hour = ts->tm_hour;
		auto min = ts->tm_min;
		auto sec = ts->tm_sec;
		auto usec = now.tv_usec;*/

	std::string str_timestamp = std::to_string(year) + "-" + std::to_string(month) + "-" + std::to_string(day) + "-" + std::to_string(hour) + "-" + std::to_string(min) + "-" + std::to_string(sec);
	if (detail) str_timestamp += "-" + std::to_string(usec);

	return str_timestamp;
}
void Time::showNowTime()
{
	time_t now;
	struct tm *ts;
	now = time(NULL);
	ts = localtime(&now);

	Debug::print(LOG_SUMMARY, "Time --> %d:%d:%d:%d:%d:%d\r\n", ts->tm_year + 1900, ts->tm_mon + 1, ts->tm_mday, ts->tm_hour, ts->tm_min, ts->tm_sec);
}
void String::split(const std::string& input, std::vector<std::string>& outputs)
{
	//文字列を空白文字で分割してvectorに格納
	outputs.clear();
	std::istringstream iss(input);
	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(outputs));
}
bool String::check_int(const std::string& str){
	if (std::all_of(str.cbegin(), str.cend(), isdigit))
	{
		return true;
	}
	return false;
}


void ConstantManager::add(unsigned int index, const char* name, double value)
{
	if (mData.count(index) != 0)
	{
		Debug::print(LOG_SUMMARY, "Constant %d  is already exist!\r\n", index);
		return;
	}
	struct CONSTANT constant = { name, value };
	mData[index] = constant;
}
double& ConstantManager::operator[](int index)
{
	static double error = DBL_MIN;
	if (mData.count(index) == 0)
	{
		Debug::print(LOG_SUMMARY, "Constant %d not found!\r\n", index);
		return error;
	}
	std::map<unsigned int, struct CONSTANT>::iterator it = mData.find(index);
	return it->second.value;
}
double& ConstantManager::operator[](const char* name)
{
	static double error = DBL_MIN;
	std::map<unsigned int, struct CONSTANT>::iterator it = mData.begin();
	while (it != mData.end())
	{
		if (it->second.name.compare(name) == 0)
		{
			return it->second.value;
		}
		++it;
	}
	Debug::print(LOG_SUMMARY, "Constant %s not found!\r\n", name);
	return error;
}

void ConstantManager::save(const char* filename)
{
	if (filename == NULL)
	{
		Debug::print(LOG_SUMMARY, "Constant: null po\r\n");
		return;
	}
	std::ofstream of(filename, std::ios::out);
	std::map<unsigned int, struct CONSTANT>::iterator it = mData.begin();
	while (it != mData.end())
	{
		of << it->first << " " << it->second.name << " " << it->second.value << std::endl;
		++it;
	}
}
void ConstantManager::load(const char* filename)
{
	if (filename == NULL)
	{
		Debug::print(LOG_SUMMARY, "Constant: null po\r\n");
		return;
	}
	std::ifstream ifs(filename, std::ios::in);
	std::string str;
	if (ifs.good())
	{
		Debug::print(LOG_SUMMARY, "Reading %s\r\n", filename);

		//行ごとに読み込んで設定を読み込む
		while (!ifs.eof() && !ifs.fail() && !ifs.bad())
		{
			std::getline(ifs, str);
			std::vector<std::string> str_constant;
			String::split(str, str_constant);

			if (str_constant.size() != 3)
			{
				Debug::print(LOG_SUMMARY, "Constant: parse error\r\n");
				continue;
			}

			add(atoi(str_constant[0].c_str()), str_constant[1].c_str(), atof(str_constant[2].c_str()));
		}
	}
}
ConstantManager& ConstantManager::get()
{
	static ConstantManager instance;
	return instance;
}
ConstantManager::ConstantManager() : mData()
{
}
ConstantManager::~ConstantManager()
{
}

double VECTOR3::calcAngleXY(const VECTOR3& current, const VECTOR3& target)//度数表示の角度
{
	double phi1 = current.x * M_PI / 180;
	double lam1 = current.y * M_PI / 180;
	double phi2 = target.x * M_PI / 180;
	double lam2 = target.y * M_PI / 180;

	return atan2(sin(lam2 - lam1)*cos(phi2), cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(lam2 - lam1)) * 180 / M_PI;

	//return atan2(target.y - current.y, target.x - current.x) / M_PI * 180;
}
double VECTOR3::calcDistanceXY(const VECTOR3& current, const VECTOR3& target)
{
	double lat1 = current.x;
	double lon1 = current.y;
	double lat2 = target.x;
	double lon2 = target.y;

	//double a = 6378137, b = 6356752.314245, f = 1 / 298.257223563;
	double a = 6378137, b = 6356752, f = 1 / 298.0;
	double L = toRadians(lon2 - lon1);


	double U1 = atan((1 - f) * tan(toRadians(lat1)));
	double U2 = atan((1 - f) * tan(toRadians(lat2)));
	double sinU1 = sin(U1), cosU1 = cos(U1);
	double sinU2 = sin(U2), cosU2 = cos(U2);
	double cosSqAlpha;
	double sinSigma;
	double cos2SigmaM;
	double cosSigma;
	double sigma;

	double lambda = L, lambdaP, iterLimit = 100;
	do
	{
		double sinLambda = sin(lambda), cosLambda = cos(lambda);
		sinSigma = sqrt((cosU2 * sinLambda)
			* (cosU2 * sinLambda)
			+ (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
			* (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
		);
		if (sinSigma == 0)
		{
			return 0;
		}

		cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
		sigma = atan2(sinSigma, cosSigma);
		double sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
		cosSqAlpha = 1 - sinAlpha * sinAlpha;
		cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;

		double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
		lambdaP = lambda;
		lambda = L + (1 - C) * f * sinAlpha
			* 	(sigma + C * sinSigma
				* 	(cos2SigmaM + C * cosSigma
					* 	(-1 + 2 * cos2SigmaM * cos2SigmaM)
					)
				);

	} while (abs(lambda - lambdaP) > 1e-5 && --iterLimit > 0);
	//} while (abs(lambda - lambdaP) > 1e-12 && --iterLimit > 0);

	if (iterLimit == 0)
	{
		return 99999;
	}

	double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
	double A = 1 + uSq / 16384
		* (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
	double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
	double deltaSigma =
		B * sinSigma
		* (cos2SigmaM + B / 4
			* (cosSigma
				* (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM
				* (-3 + 4 * sinSigma * sinSigma)
				* (-3 + 4 * cos2SigmaM * cos2SigmaM)));

	double s = b * A * (sigma - deltaSigma);

	return abs(s);
}

double VECTOR3::toRadians(double degree)
{
	double r = degree * M_PI / 180;
	return r;
}

VECTOR3 VECTOR3::operator+() const
{
	return *this;
}
VECTOR3 VECTOR3::operator-() const
{
	return VECTOR3(-x, -y, -z);
}
VECTOR3& VECTOR3::operator+=(const VECTOR3& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}
VECTOR3& VECTOR3::operator-=(const VECTOR3& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}
VECTOR3 VECTOR3::operator+(const VECTOR3& v) const
{
	return VECTOR3(x + v.x, y + v.y, z + v.z);
}
VECTOR3 VECTOR3::operator-(const VECTOR3& v) const
{
	return VECTOR3(x - v.x, y - v.y, z - v.z);
}
VECTOR3 VECTOR3::operator+(const double v) const
{
	return VECTOR3(x + v, y + v, z + v);
}
VECTOR3 VECTOR3::operator-(const double v) const
{
	return VECTOR3(x - v, y - v, z - v);
}
VECTOR3& VECTOR3::operator*=(const double v)
{
	x *= v;
	y *= v;
	z *= v;
	return *this;
}
VECTOR3& VECTOR3::operator/=(const double v)
{
	x /= v;
	y /= v;
	z /= v;
	return *this;
}
VECTOR3 VECTOR3::operator*(const double v) const
{
	return VECTOR3(x * v, y * v, z * v);
}
VECTOR3 VECTOR3::operator/(const double v) const
{
	return VECTOR3(x / v, y / v, z / v);
}
bool VECTOR3::operator==(const VECTOR3& v) const
{
	return (x == v.x) && (y == v.y) && (z == v.z);
}
bool VECTOR3::operator!=(const VECTOR3& v) const
{
	return (x != v.x) || (y != v.y) || (z != v.z);
}
double VECTOR3::norm()
{
	return sqrt(x*x + y * y + z * z);
}
VECTOR3 VECTOR3::normalize() const
{
	double length = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	if (length == 0)
	{
		return VECTOR3(0, 0, 1);
	}
	else
	{
		return *this / length;
	}
}

VECTOR3::VECTOR3() : x(0), y(0), z(0) {}
VECTOR3::VECTOR3(double tx, double ty, double tz) : x(tx), y(ty), z(tz) {}

unsigned int wiringPiI2CReadReg32LE(int fd, int address)
{
	return (unsigned int)((unsigned long)wiringPiI2CReadReg8(fd, address + 3) << 24 | (unsigned int)wiringPiI2CReadReg8(fd, address + 2) << 16 | (unsigned int)wiringPiI2CReadReg8(fd, address + 1) << 8 | (unsigned int)wiringPiI2CReadReg8(fd, address));
}
unsigned short wiringPiI2CReadReg16BE(int fd, int address)
{
	return (unsigned short)((unsigned short)wiringPiI2CReadReg8(fd, address) << 8 | (unsigned short)wiringPiI2CReadReg8(fd, address + 1));
}
unsigned short wiringPiI2CReadReg16LE(int fd, int address)
{
	return (unsigned short)((unsigned short)wiringPiI2CReadReg8(fd, address) | (unsigned short)wiringPiI2CReadReg8(fd, address + 1) << 8);
}


PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki) :
	_dt(dt),
	_max(max),
	_min(min),
	_Kp(Kp),
	_Kd(Kd),
	_Ki(Ki),
	_pre_error(0),
	_integral(0)
{
}

double PIDImpl::calculate(double setpoint, double pv)
{
	// Calculate error
	double error = setpoint - pv;

	// Proportional term
	double Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	double Iout = _Ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	double Dout = _Kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout;

	// Restrict to max/min
	if (output > _max)
		output = _max;
	else if (output < _min)
		output = _min;

	// Save error to previous error
	_pre_error = error;

	return output;
}

PIDImpl::~PIDImpl()
{
}
PID::PID()
{
}

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki)
{
	pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki);
}
double PID::calculate(double setpoint, double pv)
{
	return pimpl->calculate(setpoint, pv);
}
PID::~PID()
{
	//delete pimpl;
	pimpl = NULL;
}