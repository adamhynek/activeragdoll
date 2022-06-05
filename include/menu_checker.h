#pragma once

#include "skse64/GameReferences.h"

#include "skse64/PapyrusVM.h"
#include <unordered_map>
#include "skse64/GameMenus.h"


// Big thanks to Shizof for this method of checking what menus are open
namespace MenuChecker
{
	const std::string MOD_VERSION = "1.0.0";

	extern std::vector<std::string> gameStoppingMenus;

	extern std::unordered_map<std::string, bool> menuTypes;

	bool isGameStopped();
	
	class AllMenuEventHandler : public BSTEventSink <MenuOpenCloseEvent> {
	public:
		virtual EventResult	ReceiveEvent(MenuOpenCloseEvent * evn, EventDispatcher<MenuOpenCloseEvent> * dispatcher);
	};

	extern AllMenuEventHandler menuEvent;
}
