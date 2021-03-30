#include <tangy_move/navigationServer.h>

void navigationServer::pause() {
	debug_print("Navigation Client is paused all previous requests have been cancelled. w-forward s- backward a-turn left d-turn right q-stop");
	paused = true;
}

void navigationServer::unpause() {
	debug_print("Navigation Client has resumed you can no longer control it and it will recommence performing navigation goals");
	paused = false;
}
