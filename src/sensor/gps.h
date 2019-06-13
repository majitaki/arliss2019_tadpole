#pragma once
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include <pthread.h>
#include <list>
#include <libgpsmm.h>

//Navigatron v2からデータを取得するクラス
class GPSSensor : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//前回のチェック時刻
	struct timespec mLastGetNewDataTime;
	struct timespec mLastGetRemoveTime;
	int mFileHandle;//winringPi i2c　のファイルハンドラ
	VECTOR3 mPos;//座標(経度、緯度、高度)
	VECTOR3 mAveragePos;
	int mSatelites;//補足した衛星の数
	int mGpsTime;
	float mGpsSpeed;
	float mGpsCourse;
	bool mIsNewData;//新しい座標データがあれば真
	bool mIsLogger;//真なら1秒ごとにgpsコマンドを実行
	bool mErrorFlag;
	gpsmm gps_rec;
	struct gps_data_t *newdata;
	std::list<VECTOR3> mLastPos;
	void showState();//補足した衛星数と座標を表示
	bool mRemoveErrorFlag;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	//現在の座標を取得する(falseを返した場合は場所が不明)
	//disableNewFlagをfalseにすると座標が新しいという情報を削除
	bool get(VECTOR3& pos, bool disableNewFlag = false);

	//前回の座標取得以降にデータが更新された場合は真
	bool isNewPos() const;
	int getTime() const;
	double getPosx() const;
	double getPosy() const;
	double getPosz() const;
	float getCourse() const;
	float getSpeed() const;
	void clearSample();
	bool removeErrorSample();
	bool removeAveNewErrorSample();
	bool getAvePos(VECTOR3& pos);
	bool getDirectionAngle(double & angle);
	void clearSamples();
	bool isStuckGPS();
	bool isStuckGPS(double& distance);
	bool isAlive();

	GPSSensor();
	~GPSSensor();
};

extern GPSSensor gGPSSensor;