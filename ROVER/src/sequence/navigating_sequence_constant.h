#pragma once

const static double NAVIGATING_UPDATE_INTERVAL_TIME = 0.1; //second
const static double NAVIGATING_INITIAL_RUN_WHILE_TIME = 5; //second
const static double NAVIGATING_GOAL_FAR_DISTANCE_THRESHOLD = 5;//meter
const static double NAVIGATING_MIDDLE_DISTANCE_RATE = 3.0 / 4.0; // (from goal to middle point) / (from goal to start)
const static int NAVIGATING_STUCK_COUNT = 3;
const static int NAVIGATING_NEAR_MODE_LIMIT = 3;
const static double NAVIGATING_STUCK_CHECK_INTERVAL_TIME = 3; //second
const static double NAVIGATING_FREEZE_TIME = 10;
const static double NAVIGATING_TURN_SLOPE = 0.2;

const static double NAVIGATING_DIRECTION_UPDATE_INTERVAL = 1;//�i�s������ύX����Ԋu(�b) 2016/08/31 5->1
const static double NAVIGATING_MAX_DELTA_ANGLE = 90;//���̑���ŕ����]������ő�̊p�x
