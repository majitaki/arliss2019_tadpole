#pragma once

const static double NAVIGATING_UPDATE_INTERVAL_TIME = 0.3; //second
const static double NAVIGATING_INITIAL_RUN_WHILE_TIME = 5; //second
const static double NAVIGATING_GOAL_NEAR_DISTANCE_THRESHOLD = 10;//meter
const static double NAVIGATING_GOAL_FAR_DISTANCE_THRESHOLD = 10;
const static int NAVIGATING_STUCK_COUNT = 3;
const static int NAVIGATING_NEAR_MODE_LIMIT = 3;
const static double NAVIGATING_STUCK_CHECK_INTERVAL_TIME = 3; //second
const static double NAVIGATING_FREEZE_TIME = 10;

const static double NAVIGATING_DIRECTION_UPDATE_INTERVAL = 1;//進行方向を変更する間隔(秒) 2016/08/31 5->1
const static double NAVIGATING_MAX_DELTA_ANGLE = 90;//一回の操作で方向転換する最大の角度
