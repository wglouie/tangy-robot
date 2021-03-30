#include <rqt_task_learning/task_learning.h>
namespace rqt_task_learning{

WatchActionExecutorThread::WatchActionExecutorThread(ActionExecutorClient *input_client):
    action_executor_client(input_client){
}

void WatchActionExecutorThread::run(){
    while(action_executor_client->is_goal_active()){
    }
    emit action_complete_signal();
}

}
