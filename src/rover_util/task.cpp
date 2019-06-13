#include <algorithm>
#include <fstream>
#include <iostream>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include "utils.h"
#include "task.h"

TaskBase::TaskBase() : mpName(0), mPriority(UINT_MAX), mInterval(UINT_MAX), mSlept(0), mIsRunning(false), mNewRunningState(false), mInitializeRetryCount(0)
{
	getManager()->add(this);
}
TaskBase::~TaskBase()
{
	getManager()->del(this);
}

void TaskBase::setRunMode(bool running)
{
	//新しい状態を設定する(TaskManagerのupdate時に実際に変更される)
	mNewRunningState = running;
	mInitializeRetryCount = 0;
}
void TaskBase::setName(const char* name)
{
	if (name == NULL)
	{
		Debug::print(LOG_SUMMARY, "Task name is NOT specified!\r\n");
		return;
	}

	if (mpName != 0)delete mpName;
	int name_length = strlen(name) + 1;
	mpName = new char[name_length];
	for (int i = 0; i < name_length; ++i)mpName[i] = tolower(name[i]);
}
std::string TaskBase::getName()
{
	return std::string(mpName);
}
void TaskBase::setPriority(unsigned int pri, unsigned int interval)
{
	mPriority = pri;
	mInterval = interval;

	getManager()->sortByPriority();
}
bool TaskBase::isActive() const
{
	return mIsRunning;
}
TaskManager* TaskBase::getManager()
{
	return TaskManager::getInstance();
}
bool TaskBase::onInit(const struct timespec& time)
{
	return true;
}

void TaskBase::onClean()
{
}

bool TaskBase::onCommand(const std::vector<std::string>& args)
{
	return false;
}

void TaskBase::onUpdate(const struct timespec& time)
{
}

TaskManager::TaskManager()
{
}
TaskManager::~TaskManager()
{
	clean();
}
TaskManager* TaskManager::getInstance()
{
	//static TaskManager signleton;
	static TaskManager __attribute__((section("AHBSRAM0"))) signleton;// use mbed ethernet area
	return &signleton;
}

bool TaskManager::init()
{
	mTasks.clear();
	return true;
}
void TaskManager::clean()
{
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while (it != mTasks.end())
	{
		if (*it != NULL)if ((**it).mIsRunning)
		{
			(*it)->onClean();
			(*it)->mIsRunning = false;
		}
		++it;
	}
	mTasks.clear();
}

template <typename T> T toLower(T c) { return tolower(c); }

bool TaskManager::command(const std::string& arg)
{
	std::vector<std::string> args;
	std::string arg_mod(arg);

	//文字列の下処理
	//std::transform(arg_mod.begin(), arg_mod.end(), arg_mod.begin(), toLower<char>);//全て小文字に変換
#ifdef USE_ALIAS
	applyAlias(arg_mod);//Aliasを適用
#endif
	String::split(arg_mod, args);//スペース区切りで分割

	if (args.size() != 0)
	{
		Debug::print(LOG_SUMMARY, "> %s\r\n", arg_mod.c_str());
		TaskBase* pTask = get(args[0]);
		if (pTask != NULL)
		{
			//コマンド実行対象のタスクが見つかったらコマンドを実行
			if (args[0].compare(pTask->mpName) == 0)
			{
				if (pTask->onCommand(args))return true;
				else
				{
					Debug::print(LOG_SUMMARY, "Failed to exec command! (Error on task)\r\n");
					return false;
				}
			}
		}
		if (onCommand(args))return true;

		Debug::print(LOG_SUMMARY, "\r\nFailed to exec command!\r\n");
	}
	else
	{
		Debug::print(LOG_PRINT, "\r\n***     Command Usage   ***\r\n");
		Debug::print(LOG_PRINT, " start [task name]      : start task\r\n");
		Debug::print(LOG_PRINT, " stop [task name]       : stop task\r\n");
		Debug::print(LOG_PRINT, " list                   : enumerate tasks\r\n");
#ifdef USE_ALIAS
		Debug::print(LOG_PRINT, " alias                  : enumerate aliases\r\n");
		Debug::print(LOG_PRINT, " alias [alias] [command]: add new alias\r\n");
		Debug::print(LOG_PRINT, " unalias [alias]        : delete alias\r\n");
#endif
#ifdef USE_EXEC_SCRIPT
		Debug::print(LOG_PRINT, " exec [path]            : execute script\r\n");
#endif
		Debug::print(LOG_PRINT, " [task name] [args]     : execute command\r\n");
		return true;
	}
	return false;
}

void TaskManager::enumTasks()
{
	//すべてのタスクとその状態を列挙して表示
	Debug::print(LOG_PRINT, "\r\n Active Priority Interval Name\r\n");
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while (it != mTasks.end())
	{
		TaskBase* pTask = *it;
		if (pTask != NULL)
		{
			Debug::print(LOG_PRINT, " %s %8d %8d %s\r\n", pTask->mIsRunning ? "Yes   " : "No    ", pTask->mPriority, pTask->mInterval, pTask->mpName);
		}
		++it;
	}
}

struct timespec TaskManager::alertGlitch(const std::string& message, const struct timespec& lastTime)
{
	struct timespec newTime;

	Time::get(newTime);
	double dt = Time::dt(newTime, lastTime);
	if(dt * 1000 > TASK_GLITCH_DETECTION_THRESHOLD_MSEC)
	{
		Debug::print(LOG_DETAIL, "Glitch Detected: %s (%f sec.)\r\n", message.c_str(), dt);
	}

	return newTime;
}

void TaskManager::update()
{
	struct timespec newTime;
	Time::get(newTime);

	//タスクのupdate処理を実行
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while (it != mTasks.end())
	{
		TaskBase* pTask = *it;
		if (pTask != NULL)
		{
			if (pTask->mInterval != UINT_MAX && pTask->mIsRunning && (pTask->mInterval <= pTask->mSlept++))
			{
				//実行するタイミングであれば処理を行う(mIntervalがUINT_MAXならupdate不要なタスク)
				pTask->onUpdate(newTime);
				pTask->mSlept = 0;
				newTime = alertGlitch(std::string("While Updating ") + pTask->mpName, newTime);
			}
		}
		++it;
	}

	//タスクの実行状態を切り替え
	it = mTasks.begin();
	while (it != mTasks.end())
	{
		TaskBase* pTask = *it;
		if (pTask != NULL)
		{
			if (pTask->mIsRunning != pTask->mNewRunningState && pTask->mInitializeRetryCount < TASK_MAX_INITIALIZE_RETRY_COUNT)
			{
				++pTask->mInitializeRetryCount;
				//実行状態を変更する必要がある場合変更する
				if (pTask->mIsRunning == false)
				{
					//実行開始する場合：onInitを呼び出し、成功した場合は実行中状態に設定
					if (pTask->onInit(newTime))pTask->mIsRunning = pTask->mNewRunningState;
					else Debug::print(LOG_SUMMARY, "FAILED to initialize task %s (%d/%d)\r\n", pTask->mpName, pTask->mInitializeRetryCount, TASK_MAX_INITIALIZE_RETRY_COUNT);//失敗した場合はログ出力
					newTime = alertGlitch(std::string("While Initializing ") + pTask->mpName, newTime);
				}
				else
				{
					//実行停止する場合：onCleanを呼び出し
					pTask->onClean();
					pTask->mIsRunning = pTask->mNewRunningState;
					newTime = alertGlitch(std::string("While Cleaning ") + pTask->mpName, newTime);
				}
				pTask->mSlept = 0;
			}
		}
		++it;
	}
}

TaskBase* TaskManager::get(const std::string& name)
{
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while (it != mTasks.end())
	{
		TaskBase* pTask = *it;
		if (pTask != NULL)
		{
			if (name.compare(pTask->mpName) == 0)
			{
				return pTask;
			}
		}
		++it;
	}
	return NULL;
}
void TaskManager::setRunMode(bool running)
{
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while (it != mTasks.end())
	{
		TaskBase* pTask = *it;
		if (pTask != NULL)
		{
			pTask->setRunMode(running);
		}
		++it;
	}
}
bool TaskManager::executeFile(const char* path)
{
	Debug::print(LOG_SUMMARY, "Reading %s...", path);
	std::ifstream ifs(path);
	if (ifs.good())
	{
		Debug::print(LOG_SUMMARY, "OK\r\nExecuting Commands...\r\n");
		//コマンドをすべて実行する
		while (!ifs.eof() && !ifs.fail() && !ifs.bad())
		{
			std::string str;
			std::getline(ifs, str);
			command(str);
		}
		return true;
	}
	Debug::print(LOG_SUMMARY, "FAILED\r\n");
	return false;
}
void TaskManager::add(TaskBase* pTask)
{
	if (pTask == NULL)
	{
		Debug::print(LOG_DETAIL, "TaskManager(add): No Task Specified!\r\n");
		return;
	}
	if (std::find(mTasks.begin(), mTasks.end(), pTask) == mTasks.end())
	{
		//すでに追加されていなければタスクを追加する
		mTasks.push_back(pTask);
		sortByPriority();
	}
}
void TaskManager::del(TaskBase* pTask)
{
	if (pTask == NULL)
	{
		Debug::print(LOG_SUMMARY, "TaskManager(del): No Task Specified!\r\n");
		return;
	}
	std::vector<TaskBase*>::iterator it = mTasks.begin();
	while (it != mTasks.end())
	{
		if (pTask == *it)
		{
			*it = NULL;
			Debug::print(LOG_DETAIL, "TaskManager(del): Succeeded!\r\n");
		}
		++it;
	}
	//Debug::print(LOG_DETAIL ,"TaskManager(del): Task Not Found!\r\n");
}
void  TaskManager::sortByPriority()
{
	std::sort(mTasks.begin(), mTasks.end(), TaskSorter());
}
bool TaskManager::setRunModeByCommand(const std::string& name, bool state)
{
	TaskBase* pTask = get(name);
	if (pTask != NULL)
	{
		Debug::print(LOG_SUMMARY, "%s %s (%s)\r\n", state ? "Start" : "Stop", name.c_str(), pTask->isActive() ? "run" : "stopped");
		pTask->setRunMode(state);
		return true;
	}
	else Debug::print(LOG_SUMMARY, "%s Not Found\r\n", name.c_str());
	return false;
}
bool TaskManager::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 1)
	{
		if (args[0].compare("list") == 0)
		{
			enumTasks();
			return true;
		}
#ifdef USE_ALIAS
		else if (args[0].compare("alias") == 0)
		{
			enumAliases();
			return true;
		}
#endif
	}
	else if (args.size() == 2)
	{
#ifdef USE_ALIAS
		if (args[0].compare("unalias") == 0 && args[1].compare("unalias") != 0)
		{
			if (removeAlias(args[1]))Debug::print(LOG_SUMMARY, "Unalias %s\r\n", args[1].c_str());
			else Debug::print(LOG_SUMMARY, "Failed unalias\r\n");
			return true;
		}
#endif
#ifdef USE_EXEC_SCRIPT
		if (args[0].compare("exec") == 0)
		{
			return executeFile(args[1].c_str());
		}
#endif
	}

	if (args.size() >= 2)
	{
		if (args[0].compare("start") == 0)
		{
			bool result = false;
			for (int i = 1; i < (int)args.size(); ++i)
				if (setRunModeByCommand(args[i], true))result = true;
			return result;
		}
		else if (args[0].compare("stop") == 0)
		{
			bool result = false;
			for (int i = 1; i < (int)args.size(); ++i)
				if (setRunModeByCommand(args[i], false))result = true;
			return result;
		}
	}
	if (args.size() >= 3)
	{
#ifdef USE_ALIAS
		if (args[0].compare("alias") == 0 && args[1].compare("alias") != 0)
		{
			std::string cmd;
			for (int i = 2; i < (int)args.size(); ++i)
			{
				cmd.append(args[i]);
				cmd.append(" ");
			}

			if (addAlias(args[1], cmd))Debug::print(LOG_SUMMARY, "alias %s='%s'\r\n", args[1].c_str(), cmd.c_str());
			else Debug::print(LOG_SUMMARY, "Failed alias\r\n");
			return true;
		}
#endif
	}

	return false;
}
