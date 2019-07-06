#pragma once
#include "../sensor/nineaxis_constant.h"

const static double ACCELMANAGER_UPDATE_INTERVAL_TIME = NINEAXIS_UPDATE_INTERVAL_TIME;
const static double DECLINATION_FOR_YAW = 7.32; //Tokyo

const static double MAGNET_POS_OFFSET_X = 0;
const static double MAGNET_POS_OFFSET_Y = 0;
const static double MAGNET_POS_OFFSET_Z = 0;
const static double MAGNET_SCALE_OFFSET_X = 1;
const static double MAGNET_SCALE_OFFSET_Y = 1;
const static double MAGNET_SCALE_OFFSET_Z = 1;
const static int MAGNET_CALIBRATION_SAMPLES = 1000;

const static double TURN_SIDE_THRESHOLD = 30;
const static double TURN_BACK_THRESHOLD = 40;
