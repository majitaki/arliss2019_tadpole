#pragma once
#include "../rover_util/task.h"
#include "../rover_util/utils.h"
#include <pthread.h>
#include <list>
#include <libgpsmm.h>

class GPSSensor : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//�O��̃`�F�b�N����
	struct timespec mLastGetNewDataTime;
	struct timespec mLastGetRemoveTime;
	int mFileHandle;//winringPi i2c�@�̃t�@�C���n���h��
	VECTOR3 mPos;//���W(�o�x�A�ܓx�A���x)
	VECTOR3 mAveragePos;
	int mSatelites;//�⑫�����q���̐�
	int mGpsTime;
	float mGpsSpeed;
	float mGpsCourse;
	bool mIsNewData;//�V�������W�f�[�^������ΐ^
	bool mIsLogger;//�^�Ȃ�1�b���Ƃ�gps�R�}���h�����s
	bool mErrorFlag;
	gpsmm gps_rec;
	struct gps_data_t *newdata;
	std::list<VECTOR3> mLastPos;
	void showState();//�⑫�����q�����ƍ��W��\��
	bool mRemoveErrorFlag;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	bool get(VECTOR3& pos, bool disableNewFlag = false);

	//�O��̍��W�擾�ȍ~�Ƀf�[�^���X�V���ꂽ�ꍇ�͐^
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

	GPSSensor();
	~GPSSensor();
};

extern GPSSensor gGPSSensor;