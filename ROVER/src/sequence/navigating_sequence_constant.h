#pragma once

const static double NAVIGATING_UPDATE_INTERVAL_TIME = 0.1; //second
const static double NAVIGATING_ABORT_UPDATE_INTERVAL_TIME = 1; //second
const static double NAVIGATING_INITIAL_RUN_WHILE_TIME = 10; //second
const static unsigned int NAVIGATING_ABORT_TIME = 3600 * 3; // for initial abort time
const static unsigned int NAVIGATING_NEAR_ABORT_TIME = 3600; //near navi abort time for each
const static unsigned int NAVIGATING_START_ABORT_TIME = 1800; //secnd for reboot 
const static unsigned int NAVIGATING_START_ABORT_DISTANCE = 100; //meter
const static double NAVIGATING_GOAL_FAR_DISTANCE_THRESHOLD = 2;//meter
const static double NAVIGATING_MIDDLE_DISTANCE_RATE = 2.0 / 4.0; // (from goal to middle point) / (from goal to start)
const static int NAVIGATING_STUCK_COUNT = 3;
const static int NAVIGATING_NEAR_MODE_LIMIT = 20;
const static double NAVIGATING_STUCK_CHECK_INTERVAL_TIME = 3; //second
const static double NAVIGATING_FREEZE_TIME = 5;
const static double NAVIGATING_TURN_SLOPE = 0.25;

const static double NAVIGATING_DIRECTION_UPDATE_INTERVAL = 1;//�i�s������ύX����Ԋu(�b) 2016/08/31 5->1
const static double NAVIGATING_MAX_DELTA_ANGLE = 90;//���̑���ŕ����]������ő�̊p�x
