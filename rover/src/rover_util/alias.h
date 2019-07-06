#pragma once
#include <string>
#include <vector>

class Alias
{
	struct ALIAS_PAIR
	{
		char* pAlias;
		char* pCommand;
	};
	std::vector<struct ALIAS_PAIR> mAliases;
protected:
	bool applyAlias(std::string& cmd);
	void enumAliases();
public:
	//Alias設定(コマンド実行時の別名)
	bool addAlias(const std::string& alias, const std::string& cmd);
	bool removeAlias(const std::string& alias);

	Alias();
	virtual ~Alias();
};
