#include "menu_checker.h"

// Big thanks to Shizof for this method of checking what menus are open
namespace MenuChecker
{
	std::vector<std::string> iniChangeMenus{
		"Console",
		"Journal Menu"
	};

	std::vector<std::string> gameStoppingMenus{
		"BarterMenu",
		"Book Menu",
		"Console",
		"Native UI Menu",
		"ContainerMenu",
		"Dialogue Menu",
		"Crafting Menu",
		"Credits Menu",
		"Cursor Menu",
		"Debug Text Menu",
		"FavoritesMenu",
		"GiftMenu",
		"InventoryMenu",
		"Journal Menu",
		"Kinect Menu",
		"Loading Menu",
		"Lockpicking Menu",
		"MagicMenu",
		"Main Menu",
		"MapMarkerText3D",
		"MapMenu",
		"MessageBoxMenu",
		"Mist Menu",
		"Quantity Menu",
		"RaceSex Menu",
		"Sleep/Wait Menu",
		"StatsMenuSkillRing",
		"StatsMenuPerks",
		"Training Menu",
		"Tutorial Menu",
		"TweenMenu"
	};

	std::unordered_map<std::string, bool> menuTypes({
		{ "BarterMenu", false },
		{ "Book Menu", false },
		{ "Console", false },
		{ "Native UI Menu", false },
		{ "ContainerMenu", false },
		{ "Dialogue Menu", false },
		{ "Crafting Menu", false },
		{ "Credits Menu", false },
		{ "Cursor Menu", false },
		{ "Debug Text Menu", false },
		{ "Fader Menu", false },
		{ "FavoritesMenu", false },
		{ "GiftMenu", false },
		{ "HUD Menu", false },
		{ "InventoryMenu", false },
		{ "Journal Menu", false },
		{ "Kinect Menu", false },
		{ "Loading Menu", false },
		{ "Lockpicking Menu", false },
		{ "MagicMenu", false },
		{ "Main Menu", false },
		{ "MapMarkerText3D", false },
		{ "MapMenu", false },
		{ "MessageBoxMenu", false },
		{ "Mist Menu", false },
		{ "Overlay Interaction Menu", false },
		{ "Overlay Menu", false },
		{ "Quantity Menu", false },
		{ "RaceSex Menu", false },
		{ "Sleep/Wait Menu", false },
		{ "StatsMenu", false },
		{ "StatsMenuPerks", false },
		{ "StatsMenuSkillRing", false },
		{ "TitleSequence Menu", false },
		{ "Top Menu", false },
		{ "Training Menu", false },
		{ "Tutorial Menu", false },
		{ "TweenMenu", false },
		{ "WSEnemyMeters", false },
		{ "WSDebugOverlay", false },
		{ "WSActivateRollover", false },
		{ "LoadWaitSpinner", false }
	});

	// Menu open event functions
	AllMenuEventHandler menuEvent;

	EventResult AllMenuEventHandler::ReceiveEvent(MenuOpenCloseEvent * evn, EventDispatcher<MenuOpenCloseEvent> * dispatcher)
	{
		const char * menuName = evn->menuName.data;
	
		if (evn->opening) { // Menu opened
			if (menuTypes.find(menuName) != menuTypes.end()) {
				menuTypes[menuName] = true;
			}
		}
		else { // Menu closed
			if (menuTypes.find(menuName) != menuTypes.end()) {
				menuTypes[menuName] = false;
			}
		}
		return EventResult::kEvent_Continue;
	}

	bool isGameStopped()
	{
		for (int i = 0; i < gameStoppingMenus.size(); i++) {
			if (menuTypes[gameStoppingMenus[i]] == true) {
				return true;
			}
		}
		return false;
	}
}
