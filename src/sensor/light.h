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
	//初期化
	virtual bool onInit(const struct timespec& time);
	//センサの使用を終了する
	virtual void onClean();
	//コマンドを処理する
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	//現在の明るさを取得する
	bool get() const;

	LightSensor();
	~LightSensor();
};

extern LightSensor gLightSensor;



