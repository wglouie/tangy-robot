#include <BingoGameFullServer/BingoGameFullServer.h>

/*Robot Gui ActionClient
Handles all calls for speech and displaying text on robot screen*/
void BingoGameFullServer::gui_handler(robot_gui::Robot_guiGoal interface_goal, int delay) {

	robotGuiClient.sendGoalAndWait(interface_goal);
	sleep(delay);

}
