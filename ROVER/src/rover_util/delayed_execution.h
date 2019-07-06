#pragma once
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include "task.h"
#include "utils.h"

class DelayedExecutable
{
private:
	struct timespec mExecuteTime;
	unsigned int mDelayInMs;
	unsigned int mExecutableCount;

	void addTime(unsigned int ms);
protected:
	virtual void onExecute() = 0;
public:
	const static unsigned int COUNT_INFINITY;

	const struct timespec& getExecuteTime() const;
	bool isRequired() const;
	void execute();
	virtual void print(std::string& msg) const;

	DelayedExecutable(unsigned int delayInMs, unsigned int count = 1);
	virtual ~DelayedExecutable();
};

class DelayedExecutableCommand : public DelayedExecutable
{
private:
	std::string mExecuteCommand;
protected:
	virtual void onExecute();
public:
	virtual void print(std::string& msg) const;

	DelayedExecutableCommand(const std::string& cmd, unsigned int delayInMs, unsigned int count = 1);
	~DelayedExecutableCommand();
};

class DelayedExecutableFunction : public DelayedExecutable
{
private:
	std::function<void()> mExecuteFunction;
protected:
	virtual void onExecute();
public:
	virtual void print(std::string& msg) const;

	DelayedExecutableFunction(std::function<void()>& func, unsigned int delayInMs, unsigned int count = 1);
	~DelayedExecutableFunction();

};

class DelayedExecutor : public TaskBase
{
private:
	std::vector<std::shared_ptr<DelayedExecutable>> mDelayedList;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
public:
	//DelayedExecutableを追加する
	bool add(std::shared_ptr<DelayedExecutable> pExecutable);

	//DelayedExecutableを削除する
	void del(unsigned int index);
	void del(std::shared_ptr<DelayedExecutable> pExecutable);

	//DelayedExecutableを全部削除します
	void clear();

	DelayedExecutor();
	~DelayedExecutor();
};

extern DelayedExecutor gDelayedExecutor;

