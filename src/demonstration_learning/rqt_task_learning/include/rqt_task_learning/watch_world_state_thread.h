#include <rqt_task_learning/watch_world_state_thread.h>

namespace rqt_task_learning{
WatchWorldStateThread::WatchWorldStateThread(WorldStateIdentifierClient *input_client):
    world_state_identifier_client(input_client){
}

void WatchWorldStateThread::run(){
    while(world_state_identifier_client->is_goal_active()){
    }
    emit identification_complete_signal();
}
}
