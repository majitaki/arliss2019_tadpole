/*
センサ制御プログラム

モータ以外の実世界から情報を取得するモジュールを操作します
task.hも参照
*/
#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"

//Cdsからデータを取得するクラス
class LightSensor : public TaskBase
{
private:
	int mPin;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	bool get() const;
	void showData() const;
	LightSensor();
	~LightSensor();
};

extern LightSensor gLightSensor;



