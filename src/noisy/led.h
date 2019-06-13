#pragma once
#include <pthread.h>
#include <list>
#include "../rover_util/task.h"


const static int PIN_LED_R = 25;// LED
const static int PIN_LED_G = 27;// LED
const static int PIN_LED_B = 29;// LED

class Color 
{
private:
	int r, g, b;
public:
	int getR() { return r; }
	int getG() { return g; }
	int getB() { return b; }

};

class LED :public TaskBase
{
	struct timespec mLastUpdateTime1, mLastUpdateTime2;
private:
	int r, g, b, t;
	double s, u, v, p;
	float d;
	bool rbw, bnk, hf;
	std::list<Color>  fails;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
public:
	void reflect();
	void turnOff();
	void setColor(int);
	void setColor(int, int, int);
	void rainbow(double);
	void stopRainbow();
	void brink(double);
	void brink(double, double);
	void stopBrink();
	void hsv(float);
	void stopHSV();
	void startHSV(double);
	void clearLED();

	void error_brink();
	void add_error(int, int, int);

	LED();
	~LED();
};
extern LED gLED;
