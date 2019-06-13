/*
	シリアルから入力されたコマンドを処理するクラス

	このクラスがTaskManagerのcommandメソッドを呼び出します
	task.hも参照
	*/
#pragma once
#include <string>
#include <vector>
#include <termios.h>
#include <list>
#include "task.h"

class SerialCommand : public TaskBase
{
private:
	std::string mCommandBuffer;
	std::list<std::string> mHistory;
	std::list<std::string>::iterator mHistoryIterator;
	int mCursorPos;
	int mEscapeBeginPos;//エスケープシーケンスの開始位置(-1：エスケープなし　-2：エスケープ完了)
	struct termios mOldTermios, mNewTermios;
public:
	virtual void onUpdate(const struct timespec& time);//シリアルポートに到着したコマンドを確認する

	SerialCommand();
	~SerialCommand();
};
extern SerialCommand gSerialCommand;
