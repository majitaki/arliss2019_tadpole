#include <stdlib.h>
#include <algorithm>
#include <memory>
#include <limits.h>
#include "delayed_execution.h"

DelayedExecutor gDelayedExecutor;

const unsigned int DelayedExecutable::COUNT_INFINITY = UINT_MAX;

void DelayedExecutable::addTime(unsigned int ms)
{
	mExecuteTime.tv_sec += ms / 1000;
	mExecuteTime.tv_nsec += (ms % 1000) * 1000000l;
}

const struct timespec& DelayedExecutable::getExecuteTime() const
{
	return mExecuteTime;
}

bool DelayedExecutable::isRequired() const
{
	return mExecutableCount > 0;
}

void DelayedExecutable::execute()
{
	if(!isRequired())return;
	if(mExecutableCount != COUNT_INFINITY)--mExecutableCount;
	addTime(mDelayInMs);

	onExecute();
}

void DelayedExecutable::print(std::string& msg) const
{
}

DelayedExecutable::DelayedExecutable(unsigned int delayInMs, unsigned int count) : mDelayInMs(delayInMs), mExecutableCount(count)
{
	Time::get(mExecuteTime);
	addTime(delayInMs);
}

DelayedExecutable::~DelayedExecutable()
{
}

void DelayedExecutableCommand::onExecute()
{
	Debug::print(LOG_PRINT, "Triggered Delayed Execution\r\n");
	TaskManager::getInstance()->command(mExecuteCommand);
}

void DelayedExecutableCommand::print(std::string& msg) const
{
	msg.append(mExecuteCommand);
}

DelayedExecutableCommand::DelayedExecutableCommand(const std::string& cmd, unsigned int delayInMs, unsigned int count) : DelayedExecutable(delayInMs, count), mExecuteCommand(cmd)
{
}

DelayedExecutableCommand::~DelayedExecutableCommand()
{
}

void DelayedExecutableFunction::onExecute()
{
	Debug::print(LOG_PRINT, "Triggered Delayed Execution\r\n");
	mExecuteFunction();
}

void DelayedExecutableFunction::print(std::string& msg) const
{
	msg.append("<Function Object>");
}

DelayedExecutableFunction::DelayedExecutableFunction(std::function<void()>& func, unsigned int delayInMs, unsigned int count) : DelayedExecutable(delayInMs, count), mExecuteFunction(func)
{
}

DelayedExecutableFunction::~DelayedExecutableFunction()
{
}

bool DelayedExecutor::onInit(const struct timespec& time)
{
	return true;
}

bool DelayedExecutor::onCommand(const std::vector<std::string>& args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("test") == 0)
		{
			//you can use lambda for future execution
			std::function<void()> f = [&]()
			{
				Debug::print(LOG_SUMMARY, "This is a test message.\r\n");
			};
			auto pExecutable = new DelayedExecutableFunction(f, 1000);
			add(std::shared_ptr<DelayedExecutable>(pExecutable));

			return true;
		}
	}
	if(args.size() == 3)
	{
		if(args[1].compare("del") == 0)
		{
			unsigned index = atoi(args[2].c_str());
			del(index);

			return true;
		}
	}
	if(args.size() >= 4)
	{
		if(args[1].compare("add") == 0)
		{
			unsigned int delayInMs = atoi(args[2].c_str());

            std::string cmd;
            for(int i = 3; i < (int)args.size();++i)
            {
                cmd.append(args[i]);
                cmd.append(" ");
            }
			auto pExecutable = new DelayedExecutableCommand(cmd, delayInMs);
			add(std::shared_ptr<DelayedExecutable>(pExecutable));

			Debug::print(LOG_PRINT, "Future execution is scheduled!(%u ms)\r\n", delayInMs);

			return true;
		}
	}
	if(args.size() >= 5)
	{
		if(args[1].compare("int") == 0)
		{
			unsigned int delayInMs = atoi(args[2].c_str());
			unsigned int count = atoi(args[3].c_str());

            std::string cmd;
            for(int i = 4; i < (int)args.size();++i)
            {
                cmd.append(args[i]);
                cmd.append(" ");
            }
			auto pExecutable = new DelayedExecutableCommand(cmd, delayInMs, count);
			add(std::shared_ptr<DelayedExecutable>(pExecutable));

			Debug::print(LOG_PRINT, "Future execution is scheduled!(%u ms, %u)\r\n", delayInMs, count);

			return true;
		}
	}

	struct timespec time;
	Time::get(time);
	Debug::print(LOG_PRINT, "Usage:\r\n");
	Debug::print(LOG_PRINT, " %s add [milliseconds] [cmd]         : Append Delayed Task\r\n", args[0].c_str());
	Debug::print(LOG_PRINT, " %s int [milliseconds] [count] [cmd] : Append Delayed Task (interval)\r\n", args[0].c_str());
	Debug::print(LOG_PRINT, " %s del [id]                         : Delete Delayed Task\r\n", args[0].c_str());
	Debug::print(LOG_PRINT, " %s test                             : Test behavior of function\r\n", args[0].c_str());

	if(args.size() != 1)return false;

	Debug::print(LOG_PRINT, "*** Delayed Executables ***\r\n");
	Debug::print(LOG_PRINT, "Id Time  Detail\r\n");
	unsigned int id = 0;
	for(auto it = mDelayedList.begin(); it != mDelayedList.end(); ++it)
	{
		Debug::print(LOG_PRINT, "%2d %1.3f ", id++, Time::dt((**it).getExecuteTime(), time));
		std::string msg;
		(**it).print(msg);
		Debug::print(LOG_PRINT, "%s\r\n", msg.c_str());
	}

	return true;
}

void DelayedExecutor::onUpdate(const struct timespec& time)
{
	auto result = mDelayedList.begin();
	for(auto it = mDelayedList.begin(); it != mDelayedList.end(); ++it)
	{
		if(Time::dt((**it).getExecuteTime(), time) <= 0)
		{
			(**it).execute();
		}

		if((**it).isRequired())
		{
			*result = *it;
			++result;
		}
	}

	mDelayedList.erase(result, mDelayedList.end());
}

bool DelayedExecutor::add(std::shared_ptr<DelayedExecutable> pExecutable)
{
	if(pExecutable == NULL)return false;

	auto it = std::remove(mDelayedList.begin(), mDelayedList.end(), pExecutable);
	if(it != mDelayedList.end())mDelayedList.erase(it, mDelayedList.end());

	mDelayedList.push_back(pExecutable);
	return true;
}

void DelayedExecutor::del(unsigned int index)
{
	if(index >= mDelayedList.size())
	{
		Debug::print(LOG_PRINT, "Out of Index Error");
		return;
	}

	del(mDelayedList[index]);
}

void DelayedExecutor::del(std::shared_ptr<DelayedExecutable> pExecutable)
{
	if(pExecutable.get() == NULL)return;
	auto it = std::remove(mDelayedList.begin(), mDelayedList.end(), pExecutable);
	if(it == mDelayedList.end())return;

	mDelayedList.erase(it, mDelayedList.end());
}

void DelayedExecutor::clear()
{
	mDelayedList.clear();
}

DelayedExecutor::DelayedExecutor()
{
	setName("delay");
	setPriority(0, TASK_INTERVAL_SEQUENCE);
}

DelayedExecutor::~DelayedExecutor()
{
}
