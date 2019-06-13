/*
センサ制御プログラム

モータ以外の実世界から情報を取得するモジュールを操作します
task.hも参照
*/
#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include "pressure_address.h"



class PressureSensor : public TaskBase
{
private:
	double mPressure, mTemperature, mHumidity;
	int mFileHandle;//winringPi i2c　のファイルハンドラ
	struct timespec mLastUpdateTime;
	void readCalibrationData(int fd, bme280_calib_data *cal);
	int32_t getTemperatureCalibration(bme280_calib_data *cal, int32_t adc_T);
	double compensateTemperature(int32_t t_fine);
	double compensatePressure(int32_t adc_P, bme280_calib_data *cal, int32_t t_fine);
	double compensateHumidity(int32_t adc_H, bme280_calib_data *cal, int32_t t_fine);
	void getRawData(int fd, bme280_raw_data *raw);
	
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args); 
public:
	double getPressure();
	double getTemperature();
	double getHumidity();
	double getAltitude();
	double getAltitude(double p);
	bool isAlive();


	PressureSensor();
	~PressureSensor();
};

extern PressureSensor gPressureSensor;



