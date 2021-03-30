#include <rqt_task_learning/task_learning.h>
#include <random>

namespace rqt_task_learning{
/********************************** Previous UI **********************************/
void TaskLearning::onProbPredict(){

    activity_learner::probabilistic_predict_action srv;
    srv.request.current_world_state = world_state_identifier_client.get_world_state();
    if(prob_predict_action_client.call(srv)){
        ROS_INFO("States sent for probabilistic prediction: Activity State [%s], Name [%s], Card State [%s], Assistance Request [%d]"
                                       ,srv.request.current_world_state.activity_state.c_str()
                                       ,srv.request.current_world_state.name.c_str()
                                       ,srv.request.current_world_state.user_activity_state.c_str()
                                       ,srv.request.current_world_state.user_assistance_requests.size());

        QString prediction_text;

        for(int i=0; i<srv.response.class_list.size(); i++){
            if(i%5 == 0 && i != 0){
             prediction_text += "\n";
            }
            float reduced_precision_probability = floor(srv.response.probabilities[i] * 100.0) / 100.0;
            prediction_text += QString::fromStdString(srv.response.class_list[i])
                            + " = "
                            + QString::number(reduced_precision_probability)
                            + "\t\t";
        }
        //std::vector<std::string> class_list = srv.response.class_list;
        //std::vector<float> probabilities = srv.response.probabilities;

        std::string sampled_class = sample_class_probs(srv.response.class_list,srv.response.probabilities);
        //home/tangy/tangy-robot/src/demonstration_learning/activity_learner/train/Actual Removed Testing Actions (Tab).txt
        prediction_text += "Sampled class = "
                        + QString::fromStdString(sampled_class);
        new_gui_ui_.predicted_action_text_box->setText(prediction_text);
        if(stacked_widget->currentWidget() == new_gui_widget_){
            if(sampled_class.compare("FINISH_HELP") == 0){
                helping_user = false;
                std_msgs::String notify_help_msg;
                notify_help_msg.data = "stop help";
                notify_help_stop_pub.publish(notify_help_msg);
                new_gui_ui_.image_view_tabs->setCurrentIndex(0);

                if(debug) {
                    handle_world_state_executions(false,true,false,false);
                } else {
                    if(helping_user == true){
                        handle_world_state_executions(true,true,false,true);
                    } else {
                        handle_world_state_executions(false,true,false,true);
                    }
                }
            } else {
                if(new_gui_ui_.show_gestures_check_box->isChecked() && debug == false){
                    bool success = handle_action_executions(sampled_class, true, true);
                } else {
                    bool success = handle_action_executions(sampled_class,false, true);
                }
            }
						last_selected_action = sampled_class;
        }
    } else {
        new_gui_ui_.predicted_action_text_box->setText("Bad service call for prob_predict_action service");
        ROS_ERROR("Bad service call for prob_predict_action service");
    }

}

void TaskLearning::onPredict(){
    activity_learner::predict_action srv;
    srv.request.current_world_state = world_state_identifier_client.get_world_state();
    if(predict_action_client.call(srv)){
        ROS_INFO("States sent for prediction: Activity State [%s], Name [%s], Card State [%s], Assistance Request [%d]"
                                       ,srv.request.current_world_state.activity_state.c_str()
                                       ,srv.request.current_world_state.name.c_str()
                                       ,srv.request.current_world_state.user_activity_state.c_str()
                                       ,srv.request.current_world_state.user_assistance_requests.size());
        ROS_INFO("Predicted action: [%s]", srv.response.action.c_str());

        if(stacked_widget->currentWidget() == new_gui_widget_){
            bool success = handle_action_executions(srv.response.action, false, true);
        }
        new_gui_ui_.predicted_action_text_box->setText(QString::fromStdString(srv.response.action));
    } else {
        ROS_ERROR("Bad service call for predict_action service");
        new_gui_ui_.predicted_action_text_box->setText("Bad service call for predict_action service");
    }
}

void TaskLearning::onActionChanged(int index)
{
    if(index != 0){
       //action_buttons_timer->start(100);
    }
}


void TaskLearning::onTrain(){

    QString text;
    do{
        //QInputDialog::getText()
        text = QInputDialog::getText(stacked_widget, tr("QInputDialog::getText()"),
                                               tr("Path to training set:"), QLineEdit::Normal);

        if(!boost::filesystem::exists(text.toStdString())){
            QMessageBox msg_box;
            msg_box.setIcon(QMessageBox::Warning);
            msg_box.setText("Training file path does not exist. Please input an appropriate file path." );
            msg_box.exec();
        }
    }while(!boost::filesystem::exists(text.toStdString()));
    activity_learner::train_policy srv;

    srv.request.training_set_path = text.toStdString();
    if(train_policy_client.call(srv)){
        ROS_INFO("Policy has been trained");
        new_gui_ui_.predicted_action_text_box->setText("The policy has been trained and activity learner is ready to predict");
    } else {
        new_gui_ui_.predicted_action_text_box->setText("Bad service call for train_policy_service");
        ROS_ERROR("Bad service call for predict_action service");
    }
}

std::string TaskLearning::sample_class_probs(std::vector<std::string> class_list, std::vector<float> probabilities){
    std::vector<int> weights;
    for(int i=0; i<probabilities.size(); i++){
        float reduced_precision_probability = floor(probabilities[i] * 100.0) / 100.0;
        weights.push_back((int)(reduced_precision_probability*100));
    }
    std::default_random_engine generator(std::random_device{}());
    std::discrete_distribution<int> distribution(weights.begin(),weights.end());

    /*
    std::vector<int> class_counts(class_list.size(),0);

    for(int i=0; i<100000; i++){
       int sampled_class = distribution(generator);
       class_counts[sampled_class]++;
    }

    QString prediction_text;

    for(int i=0; i<class_list.size(); i++){
        if(i%5 == 0 && i != 0){
         prediction_text += "\n";
        }
        prediction_text += QString::fromStdString(class_list[i])
                        + " = "
                        + QString::number(class_counts[i])
                        + "\t\t";
    }
    new_gui_ui_.predicted_action_text_box->setText(prediction_text);
    */
    int sampled_class = distribution(generator);
    return class_list[sampled_class];
}

}
