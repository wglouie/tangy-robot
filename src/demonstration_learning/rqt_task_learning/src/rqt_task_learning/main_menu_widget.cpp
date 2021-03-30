#include <rqt_task_learning/task_learning.h>
namespace rqt_task_learning{
/********************************** Main Menu UI **********************************/

void TaskLearning::onNewGUINewGame(){
    std_msgs::String learning_state_msg;
    learning_state_msg.data = "new game";
    learning_state_pub.publish(learning_state_msg);
    updateWorldStateStatus();
    create_new_training_set();
    stacked_widget->setCurrentWidget(new_gui_widget_);
    get_current_trajectory();
    updateActionList();
}

void TaskLearning::onNewGUIContinueGame(){
    std_msgs::String learning_state_msg;
    learning_state_msg.data = "continue game";
    world_state_identifier_client.load_saved_state();
    learning_state_pub.publish(learning_state_msg);
    stacked_widget->setCurrentWidget(new_gui_widget_);
    get_current_trajectory();
    updateActionList();
		updateWorldStateStatus();
    

}

void TaskLearning::onNewGUIDebug(){
    std_msgs::String learning_state_msg;
    learning_state_msg.data = "new game";
    learning_state_pub.publish(learning_state_msg);
    updateWorldStateStatus();
    create_new_training_set();
    stacked_widget->setCurrentWidget(new_gui_widget_);
    get_current_trajectory();
    updateActionList();
    debug = true;
}

void TaskLearning::onGesture(){
    stacked_widget->setCurrentWidget(gesture_widget_);
    updateActionList();
}

}

