/*
	その他関数など

	デバッグ用マクロやprint関数を用意してあります
	・print関数は画面とファイル両方に出力します
	・ログレベルは重要ではないログで画面が埋め尽くされないように設定します
	・staticクラスのため単純にDebug::print()のように呼び出してください
	*/

#pragma once
#include <vector>
#include <string>
#include <map>
#include <time.h>
#include "../constants.h"

#ifdef _DEBUG
#include <assert.h>
	//xが0ならabort
#define ASSERT(x) assert(x);
//xが非0ならabort
#define VERIFY(x) assert(!(x));
#else
#define ASSERT(x)
#define VERIFY(x)
#endif

#define earthRadiusKm 6371.0

typedef enum
{
	LOG_DETAIL = 0,	//デバッグログ(バグが出たときの状況確認用)
	LOG_SUMMARY,	//ファイル保存、画面表示共にするもの
	LOG_PRINT		//画面にのみ表示するもの
}LOG_LEVEL;			//ログレベル(Apacheとかと似た感じで)

const static unsigned int MAX_STRING_LENGTH = 1024;//Print用のバッファサイズ

const static char LOG_FOLDER[] = "log";

class Debug
{
private:
public:
	static bool makeLogFolder();
	static void print(LOG_LEVEL level, const char* fmt, ...);//ストリーム面倒だからprintfタイプでいいよね
	static void typo_print(LOG_LEVEL level, const char* fmt, ...);
	Debug();
};

class String
{
public:
	//文字列を空白で分割
	static void split(const std::string& input, std::vector<std::string>& outputs);
};

class Filename
{
	std::string mPrefix, mSuffix;
	unsigned int mIndex;
public:
	void get(std::string& name);
	void getNow(std::string& name);
	void getNoIndex(std::string& name);
	Filename(const std::string& prefix, const std::string& suffix);
};

//定数マネージャ
class ConstantManager
{
	ConstantManager();
	struct CONSTANT { std::string name; double value; };
	std::map<unsigned int, struct CONSTANT> mData;
public:
	static ConstantManager& get();

	void add(unsigned int index, const char* name, double value = 0);

	double& operator[](int index);
	double& operator[](const char* name);

	void save(const char* filename);
	void load(const char* filename);

	~ConstantManager();
};

/*
 * Timespec
 */
#ifndef __timespec_defined
#define __timespec_defined
struct timespec
{
	time_t  tv_sec;   // Seconds
	long    tv_nsec;  // Nanoseconds
};
#endif

class Time
{
public:
	//時間の変化量を計算(秒)
	static double dt(const struct timespec& now, const struct timespec& last);
	//現在時刻を取得
	static void get(struct timespec& time);
	static std::string getTimeStamp(bool detail = false);
	//現在時刻をログに出力する
	static void showNowTime();
};

class VECTOR3
{
public:
	double x, y, z;

	VECTOR3 operator+() const;
	VECTOR3 operator-() const;
	VECTOR3& operator+=(const VECTOR3& v);
	VECTOR3& operator-=(const VECTOR3& v);
	VECTOR3 operator+(const VECTOR3& u) const;
	VECTOR3 operator-(const VECTOR3& u) const;
	VECTOR3 operator+(const double v) const;
	VECTOR3 operator-(const double v) const;
	VECTOR3& operator*=(const double v);
	VECTOR3& operator/=(const double v);
	VECTOR3 operator*(const double v) const;
	VECTOR3 operator/(const double v) const;
	bool operator==(const VECTOR3& v)const;
	bool operator!=(const VECTOR3& v)const;

	VECTOR3();
	VECTOR3(double tx, double ty, double tz);

	static double calcAngleXY(const VECTOR3& current, const VECTOR3& target);
	//2点間の距離を計算
	static double calcDistanceXY(const VECTOR3& current, const VECTOR3& target);
	static double toRadians(double degree);
	//normalize length to 1
	VECTOR3 normalize() const;

	double norm();
};

class PIDImpl
{
public:
	PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);
	~PIDImpl();
	double calculate(double setpoint, double pv);

private:
	double _dt;
	double _max;
	double _min;
	double _Kp;
	double _Kd;
	double _Ki;
	double _pre_error;
	double _integral;
};

class PID
{
public:
	// Kp -  proportional gain
	// Ki -  Integral gain
	// Kd -  derivative gain
	// dt -  loop interval time
	// max - maximum value of manipulated variable
	// min - minimum value of manipulated variable
	PID();
	PID(double dt, double max, double min, double Kp, double Kd, double Ki);

	// Returns the manipulated variable given a setpoint and current process value
	double calculate(double setpoint, double pv);
	~PID();

private:
	PIDImpl * pimpl;
};



unsigned int wiringPiI2CReadReg32LE(int fd, int address);
unsigned short wiringPiI2CReadReg16BE(int fd, int address);
unsigned short wiringPiI2CReadReg16LE(int fd, int address);
