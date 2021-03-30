#include <rqt_task_learning/task_learning.h>
namespace rqt_task_learning{

void TaskLearning::onStartRecordGesture(){
    if(customization_state == NONE){
        ROS_ERROR("A customization state was not selected. Cannot record gesture.");
    }

    if(stacked_widget->currentWidget() == gesture_widget_
            && (customization_state == GESTURE || customization_state == SPEECH_GESTURE)
            && gesture_process_started == true){
        if(number_of_skeletons != 0 && recording_gesture == false){
            std_msgs::String record_msg;
            record_msg.data = "start";
            record_traj_pub.publish(record_msg);
            recording_gesture = true;
            gesture_ui_.recording_status_box->setStyleSheet("background: rgb(0, 255, 0);");
            change_text("Gesture recording started");
            //is_tracking_skeletons = false;
            std::string speech = "Gesture recording has started. Please move slowly so I can follow your movements.";
            gest_spch_exe->set_spch_n_gest(speech,"","");
            emit run_gest_spch_signal();
  				  stacked_widget->setEnabled(false);
						waitDialog->push("Gesture recording has started. Please move slowly so I can follow your movements.");

						
        } else if(recording_gesture == true) {
            std::string speech = "You are already recording a gesture.";
            gest_spch_exe->set_spch_n_gest(speech,"","");
            emit run_gest_spch_signal();
						stacked_widget->setEnabled(false);
						waitDialog->push("You are already recording a gesture.");
            change_text("You are already recording a gesture.");
        } else {
            std::string speech = "Gesture recording cannot start until I start tracking you. Please perform the hands in the air pose.";
            gest_spch_exe->set_spch_n_gest(speech,"","");
						emit run_gest_spch_signal();
						stacked_widget->setEnabled(false);
						waitDialog->push("We cannot start recording until Tangy starts tracking you. Please perform Tangy's instructed pose");
                        
						change_text("We cannot start recording until Tangy starts tracking you. Please perform Tangy's instructed pose");
        }
    } else {
        ROS_ERROR("Gesture process has not started. Cannot start recording a gesture.");
    }
}

void TaskLearning::onStopRecordGesture(){
    if(customization_state == NONE){
        ROS_ERROR("A customization state was not selected. Cannot record gesture.");
    }

    std_msgs::String gesture_file_name;
    if(stacked_widget->currentWidget() == gesture_widget_
            && (customization_state == GESTURE || customization_state == SPEECH_GESTURE)
            && gesture_process_started == true) {
        if(recording_gesture == true){
            QString gui_text = gesture_ui_.actions_combo_box->currentText();
            std::string action_name = gui_text_action_map.at(gui_text.toStdString());
            QString topic = QString::fromStdString(action_name);
            std_msgs::String record_msg;
            record_msg.data = "stop";
            record_traj_pub.publish(record_msg);
            recording_gesture = false;
            gesture_ui_.recording_status_box->setStyleSheet("background: rgb(255, 0, 0);" );
            gesture_file_name.data = gesture_directory + "/" + topic.toStdString();
            save_traj_pub.publish(gesture_file_name);
            change_text("Gesture recording finished. Are you satisfied with the gesture?");
            gesture_ui_.yes_button->setVisible(true);
            gesture_ui_.no_button->setVisible(true);
            instruct_stop_record();
        } else {
            ROS_ERROR("Cannot stop recording because we have not started recording. Gesture file not saved.");
        }
    } else {
        ROS_ERROR("Gesture process has not started. Cannot start recording a gesture.");
    }
}

void TaskLearning::onFinishedSpeech(){
    QString speech = gesture_ui_.speech_text_box->text();
    gesture_ui_.info_stacked_widget->setCurrentIndex(3);
    finish_customization(speech);
}

void TaskLearning::onYesDoneGesture(){
    gesture_ui_.yes_button->setVisible(false);
    gesture_ui_.no_button->setVisible(false);
    if(customization_state == SPEECH_GESTURE){
        gesture_ui_.info_stacked_widget->setCurrentIndex(2);
    } else {
        finish_customization(" ");
        gesture_ui_.info_stacked_widget->setCurrentIndex(3);
    }
    gesture_process_started = false;
}

void TaskLearning::onNoDoneGesture(){
    gesture_ui_.cus_state_stacked_widget->setCurrentIndex(1);
    gesture_ui_.info_stacked_widget->setCurrentIndex(1);
    change_text("Please get into Tangy's view and follow Tangy's instructions to record a gesture");
    gesture_ui_.yes_button->setVisible(false);
    gesture_ui_.no_button->setVisible(false);
    gesture_ui_.action_being_changed_label->setText(gesture_ui_.actions_combo_box->currentText());
    gesture_ui_.action_selection_stacked_widget->setCurrentIndex(1);
    gesture_process_started = true;
    is_tracking_skeletons = false;
    instruct_psi_pose();

}

void TaskLearning::onRedoCustomization(){
    init_gesture_ui();
}

void TaskLearning::onPreviewCustomization(){
    QString gui_text = gesture_ui_.actions_combo_box->currentText();
    stacked_widget->setEnabled(false);
    std::string action_name = gui_text_action_map.at(gui_text.toStdString());
    waitDialog->push("Tangy is performing the action for you to preview");
    action_executor_client.preview_customization(action_name,
                                                 final_speech.toStdString(),
                                                 final_gesture_file_path.toStdString());
    waitDialog->pop();
    stacked_widget->setEnabled(true);
}

void TaskLearning::onCustomGesture(){
    gesture_ui_.cus_state_stacked_widget->setCurrentIndex(1);
    gesture_ui_.info_stacked_widget->setCurrentIndex(1);
    customization_state = GESTURE;
    gesture_ui_.cus_state_label->setText("Changing gesture");
    change_text("Please get into Tangy's view and follow Tangy's instructions to record a gesture");
    gesture_ui_.yes_button->setVisible(false);
    gesture_ui_.no_button->setVisible(false);
    gesture_ui_.action_being_changed_label->setText(gesture_ui_.actions_combo_box->currentText());
    gesture_ui_.action_selection_stacked_widget->setCurrentIndex(1);
    gesture_process_started = true;
    is_tracking_skeletons = false;
    instruct_psi_pose();
}

void TaskLearning::onCustomSpeech(){
    gesture_ui_.cus_state_stacked_widget->setCurrentIndex(1);
    gesture_ui_.info_stacked_widget->setCurrentIndex(2);
    customization_state = SPEECH;
    gesture_ui_.cus_state_label->setText("Changing speech");
    gesture_ui_.action_being_changed_label->setText(gesture_ui_.actions_combo_box->currentText());
    gesture_ui_.action_selection_stacked_widget->setCurrentIndex(1);
}

void TaskLearning::onCustomSpeechGesture(){
    gesture_ui_.cus_state_stacked_widget->setCurrentIndex(1);
    gesture_ui_.info_stacked_widget->setCurrentIndex(1);
    customization_state = SPEECH_GESTURE;
    gesture_ui_.cus_state_label->setText("Changing both speech and gesture");
    change_text("Please get into Tangy's view and follow Tangy's instructions to record a gesture");
    gesture_ui_.yes_button->setVisible(false);
    gesture_ui_.no_button->setVisible(false);
    gesture_ui_.action_being_changed_label->setText(gesture_ui_.actions_combo_box->currentText());
    gesture_ui_.action_selection_stacked_widget->setCurrentIndex(1);
    gesture_process_started = true;
    is_tracking_skeletons = false;
    instruct_psi_pose();    
}

void TaskLearning::instruct_psi_pose(){
    std::string speech = "Please make the following gesture so I can track your movements. I will let you know when I am ready to start recording your movements";
    std::string package_name = "action_executor";
    std::string relative_path = "/database/demonstration_gestures/psi_pose";
    gest_spch_exe->set_spch_n_gest(speech,package_name,relative_path);
    emit run_gest_spch_signal();
    stacked_widget->setEnabled(false);
    waitDialog->push("Tangy is providing instructions on how to record gestures. Please wait and listen");
    //gesture_widget_->setEnabled(false);
}

void TaskLearning::spch_gest_done(){

    waitDialog->pop();
		stacked_widget->setEnabled(true);
    stacked_widget->installEventFilter(wireless_device_receiver);
    stacked_widget->activateWindow();
		stacked_widget->setFocus();    
    if(!is_tracking_skeletons){
        emit watch_for_skeletons();
    }
}

void TaskLearning::instruct_start_record(){
    is_tracking_skeletons = true;
    waitDialog->pop();
    stacked_widget->setEnabled(true);
    std::string speech = "You can now put your arms down. Start gesture recording by pressing the left button on the wireless clicker. When you are done press the right button to stop the recording.";
    std::string package_name = "action_executor";
    std::string relative_path = "/database/demonstration_gestures/neutral_pose";
    gest_spch_exe->set_spch_n_gest(speech,package_name,relative_path,true);
    emit run_gest_spch_signal();
    stacked_widget->setEnabled(false);
    waitDialog->push("Tangy is providing instructions on how to record gestures. Please wait and listen");

}

void TaskLearning::instruct_stop_record(){
    is_tracking_skeletons = true;
    std::string speech = "You have stopped recording the gesture. Please return to the computer.";
    std::string package_name = "";
    std::string relative_path = "";
    gest_spch_exe->set_spch_n_gest(speech,package_name,relative_path,true);
    emit run_gest_spch_signal();
    stacked_widget->setEnabled(false);
    waitDialog->push("Recording has stopped. Please wait until Tangy has finished speaking.");
}


void TaskLearning::skel_status_cb(const std_msgs::Int8::ConstPtr& msg){
    number_of_skeletons = msg->data;
    skele_watcher->update_num_skeletons(number_of_skeletons);
    gesture_ui_.skeleton_tracking_status->setText(QString::number(number_of_skeletons));
    if(recording_gesture == true){
        if(number_of_skeletons == 0){
            std::string speech = "Stopping recording and saved gesture. We have lost track of you. If this was unintended redo customization in step 4.";
            action_executor_client.exe_spch_and_gest(speech, " ", " ");
            onStopRecordGesture();
            QString error_msg = "Stopped recording and saved file because we lost track of the user";
            emit msgbox_signal(0, error_msg);
        }
    }
}


void TaskLearning::onCancelCustomization(){
    stacked_widget->setCurrentWidget(new_gui_widget_);

    if(stacked_widget->currentWidget() == gesture_widget_){
        stacked_widget->setCurrentWidget(new_gui_widget_);
        demonstration_learning_msgs::world_state world_state = world_state_identifier_client.get_world_state();
        std::string robot_state = world_state.robot_state.data();
        if(robot_state.compare("At front of room") != 0){
            std_msgs::Float32 goal;
            goal.data = 0.35;
            move_straight_cmd_pub.publish(goal);
        }
    }
}

void TaskLearning::onSaveCustomization(){
    if(customization_state == NONE){
        ROS_ERROR("A customization state was not selected. Cannot finish customization.");
    }

    if(stacked_widget->currentWidget() == gesture_widget_){        
        // Send msg to action_executor to create new action
        QString gui_text = gesture_ui_.actions_combo_box->currentText();
        std::string action_name = gui_text_action_map.at(gui_text.toStdString());
        QString topic = QString::fromStdString(action_name);
        demonstration_learning_msgs::customize_action msg;
        msg.action_name = topic.toStdString();
        msg.gesture_file_name = final_gesture_file_path.toStdString();
        msg.speech = final_speech.toStdString();
        customize_action_pub.publish(msg);

        //Move back to table
        demonstration_learning_msgs::world_state world_state = world_state_identifier_client.get_world_state();
        std::string robot_state = world_state.robot_state.data();
        if(robot_state.compare("At front of room") != 0){
            std_msgs::Float32 goal;
            goal.data = 0.35;
            move_straight_cmd_pub.publish(goal);
        }

        //Return to New GUI
        stacked_widget->setCurrentWidget(new_gui_widget_);

    }
}

void TaskLearning::finish_customization(QString input_speech){
    if(customization_state == NONE){
        ROS_ERROR("A customization state was not selected. Cannot save customization.");
    } else {
        ROS_INFO("Saving your customizations for the action");
    }

    QString gui_text = gesture_ui_.actions_combo_box->currentText();
    std::string action_name = gui_text_action_map.at(gui_text.toStdString());
    QString topic = QString::fromStdString(action_name);
    final_gesture_file_path = " ";
    if(customization_state == GESTURE || customization_state == SPEECH_GESTURE){
        std::string file_path = gesture_directory + "/" + topic.toStdString();
        final_gesture_file_path = QString::fromStdString(file_path);
        ROS_INFO("Final Customization path: [%s]", final_gesture_file_path.toStdString().c_str());
    }

    final_speech = " ";
    if(customization_state == SPEECH || customization_state == SPEECH_GESTURE){
        final_speech = input_speech;
        ROS_INFO("Final speech: [%s]", final_speech.toStdString().c_str());
    }

    gesture_ui_.redo_button->setVisible(true);
    gesture_ui_.save_button->setVisible(true);
    gesture_ui_.preview_button->setVisible(true);

}

void TaskLearning::change_text(QString text){
    //Set the text for the first label (should be the trivia question)
    int font_size = max_font_size(text, gesture_ui_.textEdit);
    QFont font = gesture_ui_.textEdit->font();
    font.setPointSize(font_size);
    gesture_ui_.textEdit->setFont(font);
    gesture_ui_.textEdit->setText(text);
    gesture_ui_.textEdit->setAlignment(Qt::AlignCenter);
}


void TaskLearning::create_message_box(int icon, QString msg){
    QMessageBox msg_box;
    if(icon == 0){
        msg_box.setIcon(QMessageBox::Critical);
    } else {
        msg_box.setIcon(QMessageBox::Warning);
    }
    msg_box.setText(msg);
    int ret = msg_box.exec();
}

void TaskLearning::init_gesture_ui(){
    //Gesture UI initialization
    customization_state = NONE;
    gesture_ui_.speech_text_box->setText(" ");
    gesture_ui_.action_selection_stacked_widget->setCurrentIndex(0);
    gesture_ui_.cus_state_stacked_widget->setCurrentIndex(0);
    gesture_ui_.info_stacked_widget->setCurrentIndex(0);
    change_text("");
    final_speech = " ";
    final_gesture_file_path = " ";
    gesture_ui_.yes_button->setVisible(false);
    gesture_ui_.no_button->setVisible(false);
    gesture_ui_.redo_button->setVisible(false);
    gesture_ui_.save_button->setVisible(false);
    gesture_ui_.preview_button->setVisible(false);
    recording_gesture = false;
    gesture_process_started = false;
    number_of_skeletons = 0;
    skele_watcher->update_num_skeletons(number_of_skeletons);
}

}
