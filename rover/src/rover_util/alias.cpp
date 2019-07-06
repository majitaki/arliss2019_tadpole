#include "alias.h"
#include "utils.h"
#include <string.h>

bool Alias::addAlias(const std::string& alias, const std::string& cmd)
{
	// Look up old aliases
	for (std::vector<struct ALIAS_PAIR>::iterator it = mAliases.begin(); it != mAliases.end(); ++it)
		if (alias.compare(it->pAlias) == 0)return false;

	//apply setting
	struct ALIAS_PAIR pair;

	//copy alias
	int len = alias.length();
	pair.pAlias = new char[len + 1];
	memcpy(pair.pAlias, alias.c_str(), len);
	pair.pAlias[len] = '\0';

	//copy command
	len = cmd.length();
	pair.pCommand = new char[len + 1];
	memcpy(pair.pCommand, cmd.c_str(), len);
	pair.pCommand[len] = '\0';

	mAliases.push_back(pair);
	return true;
}

bool Alias::removeAlias(const std::string& alias)
{
	// Look up old aliases
	for (std::vector<struct ALIAS_PAIR>::iterator it = mAliases.begin(); it != mAliases.end(); ++it)
		if (alias.compare(it->pAlias) == 0)
		{
			delete it->pAlias;
			delete it->pCommand;
			mAliases.erase(it);
			return true;
		}
	return false;
}

bool Alias::applyAlias(std::string& cmd)
{
	size_t cmd_length = cmd.find(' ');
	if (cmd_length == std::string::npos)cmd_length = cmd.length();

	for (std::vector<struct ALIAS_PAIR>::iterator it = mAliases.begin(); it != mAliases.end(); ++it)
	{
		if (strlen(it->pAlias) == cmd_length && cmd.compare(0, cmd_length, it->pAlias, cmd_length) == 0)
		{
			cmd = it->pCommand + cmd.substr(cmd_length);
			Debug::print(LOG_PRINT, "\r\nalias %s->%s", it->pAlias, cmd.c_str());
			return true;
		}
	}

	return false;
}

void Alias::enumAliases()
{
	Debug::print(LOG_PRINT, "***        Aliases        ***\r\n");
	for (std::vector<struct ALIAS_PAIR>::iterator it = mAliases.begin(); it != mAliases.end(); ++it)
	{
		Debug::print(LOG_PRINT, " %6s=\'%s\'\r\n", it->pAlias, it->pCommand);
	}
}

Alias::Alias(){}
Alias::~Alias(){}
