/*
	タスククラス

	基本的にいじる必要はありません
	役割
	・コマンドを実行
	・初期化/開放を制御
	・一定間隔ごとに行う必要のある処理を実行
	→簡易タスクシステム

	使い方
	・TaskBaseを継承したクラスがインスタンス化されるとTaskManagerに自動で登録される
	・TaskBaseのsetRunModeメソッドを呼び出すとタスクが有効になり、初期化される
	・TaskManagerの各メソッドを呼び出すことで、TaskBaseの各メソッドが適切に呼び出される
	*/
#pragma once
#include <map>
#include <string>
#include <limits.h>
#include <vector>
#include <time.h>
#include "alias.h"

#define USE_ALIAS
#define USE_EXEC_SCRIPT

class TaskManager;

//タスク基底クラス
class TaskBase
{
private:
	friend class TaskManager;
	TaskBase(const TaskBase& obj);

	//タスクの状態を表す変数
	char* mpName;//タスク名
	unsigned int mPriority, mInterval;//タスク実行設定(優先度、実行間隔)
	unsigned int mSlept;//実行がスキップされた回数
	bool mIsRunning;//実行中
	bool mNewRunningState;//新しい実行状態
	unsigned int mInitializeRetryCount;//初期化失敗回数
protected:
	//このタスクに名前を設定することでコマンドを受け付けるようにする
	void setName(const char* name);
	std::string getName();

	//このタスクに優先度(小さいほど先に実行される)と実行間隔(小さいほどたくさん実行する)を設定する
	void setPriority(unsigned int pri, unsigned int interval);

	//このタスクを管理するTaskManagerのインスタンスを返す
	virtual TaskManager* getManager();

	///////////////////////////////////////////////////
	//各タスクが実装する関数
	///////////////////////////////////////////////////
	/* このタスクを初期化する
	  呼び出しタイミングはTaskManagerのupdateメソッド内で実行中タスクのonUpdate処理が終わった後
	  falseを返した場合、TaskManagerのupdateメソッドが呼ばれるたびに再度呼び出される
	  */
	virtual bool onInit(const struct timespec& time);
	/* このタスクを開放する
	  不要な電力消費を抑えるために、極力最小限の状態に変更すること
	  */
	virtual void onClean();

	/* 指定されたコマンドを実行する
	  このメソッドは実行状態にかかわらず呼び出されるため注意
	  falseを返すと、コマンドの実行に失敗した旨が表示される
	  */
	virtual bool onCommand(const std::vector<std::string>& args);

	/* ある程度の時間ごとに呼び出される関数
	  数ms以内に処理を返すこと！！
	  実行中状態の場合にのみ呼び出される
	  */
	virtual void onUpdate(const struct timespec& time);
	///////////////////////////////////////////////////

public:
	//このタスクの実行状態を返す
	bool isActive() const;

	//このタスクの実行状態を変更する(必要に応じてinit/cleanが呼ばれる)
	void setRunMode(bool running);

	TaskBase();
	~TaskBase();
};

//タスクマネージャクラス(シングルトン)
class TaskManager : public Alias
{
private:
	TaskManager(const TaskManager& obj);
	//管理下のタスク
	std::vector<TaskBase*> mTasks;

	class TaskSorter {
	public:
		bool operator()(const TaskBase* riLeft, const TaskBase* riRight) const {
			if (riLeft == NULL)return false;
			if (riRight == NULL)return true;
			return riLeft->mPriority < riRight->mPriority;
		}
	};
	TaskManager();

	bool onCommand(const std::vector<std::string>& args);
	bool setRunModeByCommand(const std::string& name, bool state);
	void enumTasks();
protected:
	struct timespec alertGlitch(const std::string& message, const struct timespec& lastTime); 
public:
	const static unsigned int TASK_MAX_INITIALIZE_RETRY_COUNT = 5;
	const static unsigned int TASK_GLITCH_DETECTION_THRESHOLD_MSEC = 100;

	//インスタンスを取得
	static TaskManager* getInstance();

	//初期化
	bool init();
	//開放
	void clean();
	//指定されたコマンドを実行する(空白文字区切り)
	bool command(const std::string& arg);
	//ある程度の時間ごとに呼び出すこと
	void update();
	//指定されたタスクへのポインタを返す(NULLはエラー)
	TaskBase* get(const std::string& name);

	//全タスクの実行状態を変更する
	void setRunMode(bool running);

	//設定ファイルを実行する
	bool executeFile(const char* path);

	//指定されたタスクを登録/削除(基本的に呼び出す必要なし)
	void add(TaskBase* pTask);
	void del(TaskBase* pTask);

	//タスクリストを優先度順に並び替える(基本的に呼び出す必要なし)
	void sortByPriority();

	virtual ~TaskManager();
};
