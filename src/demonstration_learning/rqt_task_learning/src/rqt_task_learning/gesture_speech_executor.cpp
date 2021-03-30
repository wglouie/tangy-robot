#include <rqt_task_learning/task_learning.h>
namespace rqt_task_learning{

GestureSpeechExecutor::GestureSpeechExecutor(ActionExecutorClient *input_client, QObject *parent):
    QObject(parent)
    , action_executor_client(input_client){
    speech = "";
    gesture_containing_package = "";
    gesture_relative_file_path = "";
		turn_off_display = false;
}
void GestureSpeechExecutor::set_spch_n_gest(std::string speech, std::string gesture_containing_package, std::string gesture_relative_file_path, bool turn_off_display){
    this->speech = speech;
    this->gesture_containing_package = gesture_containing_package;
    this->gesture_relative_file_path = gesture_relative_file_path;
		this->turn_off_display = turn_off_display;
}
void GestureSpeechExecutor::run_spch_n_gest(){
    ROS_INFO("Running gesture and speech exeuction thread");
    if (this->speech.find_first_not_of(' ') == std::string::npos) { ROS_WARN("Speech was not set.");}
    if (this->gesture_containing_package.find_first_not_of(' ') == std::string::npos) { ROS_WARN("Gesture Package not set");}
    if (this->gesture_relative_file_path.find_first_not_of(' ') == std::string::npos) { ROS_WARN("Gesture file path not set");}
    action_executor_client->exe_spch_and_gest(this->speech, this->gesture_containing_package, this->gesture_relative_file_path, this->turn_off_display);
    this->speech = "";
    this->gesture_containing_package = "";
    this->gesture_relative_file_path = "";
    emit done();
}

}
