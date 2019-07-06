#pragma once
#include <string.h>

const static double SERVO_UPDATE_INTERVAL_TIME = 1.0; //second
const static int ENIN_ID = 25;
//basic
const static int MAX_RAW_VALUE = 11500;
const static int CENTER_RAW_VALUE = 7500;
const static int MIN_RAW_VALUE = 3500;

//neck
const static int NECK_ID = 3;
const static std::string NECK_NAME = "neck";
const static int NECK_INNER = 8000;
const static int NECK_CENTER = 5700;
const static int NECK_OUTER = 5700;
//direct
const static int DIRECT_ID = 0;
const static std::string DIRECT_NAME = "direct";
const static int DIRECT_RIGHT = 6000;
const static int DIRECT_CENTER = 7500;
const static int DIRECT_LEFT = 9000;
//waist
const static int WAIST_ID = 2;
const static std::string WAIST_NAME = "waist";
const static int WAIST_INNER = 5000;
const static int WAIST_CENTER = 7500;
const static int WAIST_OUTER = 9500;
//stabi
const static int STABI_ID = 1;
const static std::string STABI_NAME = "stabi";
const static int STABI_INNER = 7500;
const static int STABI_CENTER = 5000;
const static int STABI_OUTER = 5000;

