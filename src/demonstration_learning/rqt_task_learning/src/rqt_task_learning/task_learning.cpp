/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rqt_task_learning/task_learning.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QMessageBox>
#include <QPainter>
#include <iostream>
#include <fstream>

namespace rqt_task_learning{

TaskLearning::TaskLearning()
    : rqt_gui_cpp::Plugin()
    , start_menu_widget_(0)
    , new_gui_widget_(0)
    , gesture_widget_(0)
    , stacked_widget(0)
    , watch_action_executor_thread_(0)
    , action_executor_client("action_executor_client")
    , world_state_identifier_client("world_state_identifier_client")
    , helping_user(false)
    , demo_started(false)
    , debug(false)
    , green_buttons(false)
    , red_buttons(false)
    , recording_gesture(false)
    , customization_state(NONE)
{
    setObjectName("TaskLearning");
}

TaskLearning::~TaskLearning(){
    delete start_menu_widget_;
    delete new_gui_widget_;
    delete gesture_widget_;
    delete stacked_widget;
    delete watch_action_executor_thread_;
    delete watch_world_state_thread_;
    delete waitDialog;
    delete notify_card_state_timer;
    delete notify_help_timer;
    delete skele_watcher;
    delete bingo_number_data;
}

void TaskLearning::initPlugin(qt_gui_cpp::PluginContext& context)
{

    start_menu_widget_ = new QWidget();
    new_gui_widget_ = new QWidget();
    gesture_widget_ = new QWidget();
    stacked_widget= new QStackedWidget();

    start_menu_ui_.setupUi(start_menu_widget_);
    new_gui_ui_.setupUi(new_gui_widget_);
    gesture_ui_.setupUi(gesture_widget_);

    if (context.serialNumber() > 1){
        start_menu_widget_->setWindowTitle(start_menu_widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        new_gui_widget_->setWindowTitle(new_gui_widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        gesture_widget_->setWindowTitle(gesture_widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
     }

    // General
    connect(this,SIGNAL(msgbox_signal(int,QString)),this, SLOT(create_message_box(int,QString)));

    // Action Execution
    updateActionList();
    watch_action_executor_thread_ = new WatchActionExecutorThread(&action_executor_client);
    connect(watch_action_executor_thread_, SIGNAL(action_complete_signal()), this, SLOT(updateActionExecutorStatus()), Qt::QueuedConnection);

    // World State Identification
    watch_world_state_thread_ = new WatchWorldStateThread(&world_state_identifier_client);
    connect(watch_world_state_thread_, SIGNAL(identification_complete_signal()), this, SLOT(updateWorldStateStatus()), Qt::QueuedConnection);

    // Start Menu
    connect(start_menu_ui_.new_gui_continue_game_button, SIGNAL(released()), this, SLOT(onNewGUIContinueGame()));
    connect(start_menu_ui_.new_gui_new_game_button, SIGNAL(released()), this, SLOT(onNewGUINewGame()));
    connect(start_menu_ui_.new_gui_debug_button, SIGNAL(released()), this, SLOT(onNewGUIDebug()));
    connect(start_menu_ui_.gestures_button, SIGNAL(released()), this, SLOT(onGesture()));

    // New Gui
    waitDialog = new WaitDialog(tr("Loading data... please wait."), this->stacked_widget);
    notify_card_state_timer = new QTimer(this);
    notify_help_timer = new QTimer(this);
    connect(new_gui_ui_.perform_action_button, SIGNAL(released()), this, SLOT(onPreviewAction()));
    connect(new_gui_ui_.undo_action_button, SIGNAL(released()), this, SLOT(onUndoAction()));
    connect(new_gui_ui_.end_task_button, SIGNAL(released()), this, SLOT(onEndTask()));
    connect(new_gui_ui_.detect_card_button, SIGNAL(released()), this, SLOT(onDetectCard()));
    connect(notify_card_state_timer, SIGNAL(timeout()), this, SLOT(blink_card_state_warning_box()));
    connect(notify_help_timer, SIGNAL(timeout()), this, SLOT(blink_help_state_warning_box()));
    connect(new_gui_ui_.customize_action_button, SIGNAL(released()), this, SLOT(onCustomizeAction()));
    connect(new_gui_ui_.predict_button, SIGNAL(released()), this, SLOT(onPredict()));
    connect(new_gui_ui_.prob_predict_button, SIGNAL(released()), this, SLOT(onProbPredict()));
    connect(this, SIGNAL(select_new_action()),this, SLOT(onProbPredict()));
    connect(new_gui_ui_.train_button, SIGNAL(released()), this, SLOT(onTrain()));

    // Gesture GUI
    wireless_device_receiver = new WirelessDeviceReceiver();
    connect(gesture_ui_.redo_button, SIGNAL(released()), this, SLOT(onRedoCustomization()));
    connect(gesture_ui_.preview_button, SIGNAL(released()), this, SLOT(onPreviewCustomization()));
    connect(gesture_ui_.save_button, SIGNAL(released()), this, SLOT(onSaveCustomization()));
    connect(gesture_ui_.cus_gesture_button, SIGNAL(released()), this, SLOT(onCustomGesture()));
    connect(gesture_ui_.cus_speech_button, SIGNAL(released()), this, SLOT(onCustomSpeech()));
    connect(gesture_ui_.cus_speech_gesture_button, SIGNAL(released()), this, SLOT(onCustomSpeechGesture()));
    connect(gesture_ui_.cancel_button, SIGNAL(released()), this, SLOT(onCancelCustomization()));
    connect(wireless_device_receiver, SIGNAL(start_record_signal()), this, SLOT(onStartRecordGesture()));
    connect(wireless_device_receiver, SIGNAL(stop_record_signal()), this, SLOT(onStopRecordGesture()));
    connect(gesture_ui_.yes_button, SIGNAL(released()), this, SLOT(onYesDoneGesture()));
    connect(gesture_ui_.no_button, SIGNAL(released()), this, SLOT(onNoDoneGesture()));
    connect(gesture_ui_.speech_text_box, SIGNAL(returnPressed()), this, SLOT(onFinishedSpeech()));
    skele_watcher = new SkeletonWatcher();
    watch_skeleton_thread_ = new QThread;
    skele_watcher->moveToThread(watch_skeleton_thread_);
    connect(this, SIGNAL(watch_for_skeletons()), skele_watcher, SLOT(track_skeletons()));
    connect(skele_watcher, SIGNAL(done()), this, SLOT(instruct_start_record()), Qt::QueuedConnection);
    watch_skeleton_thread_->start();

    gest_spch_exe = new GestureSpeechExecutor(&action_executor_client);
    watch_gest_spch_exe_thread_ = new QThread;
    gest_spch_exe->moveToThread(watch_gest_spch_exe_thread_);
    connect(this, SIGNAL(run_gest_spch_signal()), gest_spch_exe, SLOT(run_spch_n_gest()),Qt::QueuedConnection);
    connect(gest_spch_exe, SIGNAL(done()), this, SLOT(spch_gest_done()), Qt::QueuedConnection);
    watch_gest_spch_exe_thread_->start();

    //Maybe connect done() signal for gest_spch_exe?

    //Bingo number tabl e header
    bingo_number_data = new QStandardItemModel(15,5);
    bingo_number_data->setHorizontalHeaderItem(0, new QStandardItem(QString("B")));
    bingo_number_data->setHorizontalHeaderItem(1, new QStandardItem(QString("I")));
    bingo_number_data->setHorizontalHeaderItem(2, new QStandardItem(QString("N")));
    bingo_number_data->setHorizontalHeaderItem(3, new QStandardItem(QString("G")));
    bingo_number_data->setHorizontalHeaderItem(4, new QStandardItem(QString("O")));
    new_gui_ui_.bingo_numbers_table->setModel(bingo_number_data);
    new_gui_ui_.bingo_numbers_table->resizeColumnsToContents();
    new_gui_ui_.bingo_numbers_table->resizeRowsToContents();
    new_gui_ui_.bingo_numbers_table->horizontalHeader()->setStretchLastSection(true);
    new_gui_ui_.bingo_numbers_table->horizontalHeader()->setResizeMode(QHeaderView::Stretch);

    //Trajectory Table
    new_gui_ui_.trajectory_table->setSelectionBehavior(QAbstractItemView::SelectRows);

    // Publishers and Subscribers
    ros::NodeHandle nh_;
    state_action_notifier = nh_.advertise<std_msgs::String>("rqt_task_learning/state_action_notifier",1000);
    notify_help_stop_pub = nh_.advertise<std_msgs::String>("rqt_task_learning/notify_help_stop",1000);
    notify_end_task_pub = nh_.advertise<std_msgs::String>("rqt_task_learning/notify_end_task", 1000);
    notify_undo_action_pub = nh_.advertise<std_msgs::String>("rqt_task_learning/notify_undo_action", 1000);
    learning_state_pub = nh_.advertise<std_msgs::String>("rqt_task_learning/learning_state",1000);
    record_traj_pub = nh_.advertise<std_msgs::String>("robot_ik/record",1000);
    save_traj_pub = nh_.advertise<std_msgs::String>("robot_ik/save_to_file",1000);
    exec_traj_pub = nh_.advertise<std_msgs::String>("robot_ik/execute_file",1000);
    save_action_database_pub = nh_.advertise<std_msgs::String>("action_executor/save_database", 1000);
    customize_action_pub = nh_.advertise<demonstration_learning_msgs::customize_action>("action_executor/customize_action", 1000);
    move_straight_cmd_pub = nh_.advertise<std_msgs::Float32>("action_executor/move_straight_cmd", 1000);
    set_arms_neutral_pub = nh_.advertise<std_msgs::String>("trajectory_executor/go_to_neutral", 1000);
    skel_status_sub = nh_.subscribe("skeletontracker_nu/status", 1000, &TaskLearning::skel_status_cb, this);
    notify_help_start_sub = nh_.subscribe("action_executor/notify_help_start", 1000, &TaskLearning::help_start, this);
    called_numbers_sub = nh_.subscribe("action_executor/called_numbers", 1000, &TaskLearning::called_numbers_callback, this);
    notify_new_tri_sub = nh_.subscribe("/help_indicators/notify_new_triangle", 1000, &TaskLearning::new_triangle_callback, this);
    new_training_set_client = nh_.serviceClient<activity_learner::create_new_training_set>("activity_learner/create_new_training_set");
    train_policy_client = nh_.serviceClient<activity_learner::train_policy>("activity_learner/train_policy");
    predict_action_client = nh_.serviceClient<activity_learner::predict_action>("activity_learner/predict_action");
    prob_predict_action_client = nh_.serviceClient<activity_learner::probabilistic_predict_action>("activity_learner/probabilistic_predict_action");
    save_trajectory_client = nh_.serviceClient<activity_learner::save_trajectory>("activity_learner/save_trajectory");
    get_trajectory_client = nh_.serviceClient<activity_learner::get_current_trajectory>("activity_learner/get_current_trajectory");
    init_action_database_client = nh_.serviceClient<demonstration_learning_msgs::init_action_database>("action_executor/init_database");
    create_empty_save_folder();

    // Images
    new_gui_ui_.help_image_frame->installEventFilter(this);
    new_gui_ui_.card_image_frame->installEventFilter(this);
    new_gui_ui_.eye_image_frame->installEventFilter(this);
    gesture_ui_.gesture_img_frame->installEventFilter(this);
    stacked_widget->installEventFilter(wireless_device_receiver);
    waitDialog->installEventFilter(this);
    help_qimage_ = QImage();
    card_qimage_ = QImage();
    eye_qimage_ = QImage();
    gesture_qimage_ = QImage();
    new_gui_ui_.help_image_frame->update();
    new_gui_ui_.card_image_frame->update();
    new_gui_ui_.eye_image_frame->update();
    gesture_ui_.gesture_img_frame->update();
    image_transport::ImageTransport it(nh_);
    try {
        image_transport::TransportHints hints("theora");
        help_subscriber_ = it.subscribe("help_indicators/detected_triangles", 1, &TaskLearning::help_image_cb, this,hints);
        card_subscriber_ = it.subscribe("/bingo_detection/analyzed_image", 1, &TaskLearning::card_image_cb, this,hints);
        eye_subscriber_ = it.subscribe("/Right_axis_camera/image_raw",1, &TaskLearning::eye_image_cb, this,hints);
        gesture_img_subscriber_ = it.subscribe("/camera/rgb/image_raw",1, &TaskLearning::gesture_image_cb, this,hints);
    } catch (image_transport::TransportLoadException& e) {
        QMessageBox::warning(new_gui_widget_, tr("Loading image transport plugin failed"), e.what());
    }

    //Create stacked widget to flip screens
    context.addWidget(stacked_widget);

    stacked_widget->addWidget(start_menu_widget_);
    stacked_widget->addWidget(new_gui_widget_);
    stacked_widget->addWidget(gesture_widget_);
    stacked_widget->setCurrentWidget(start_menu_widget_);
}

/********************************** General **********************************/
void TaskLearning::called_numbers_callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
    std::vector<int> called_numbers;
    called_numbers = msg->data;

    //Actions UI
    QString numbers_string;
    for(int i=0; i<called_numbers.size(); i++){
        if(i != 0){
            numbers_string = numbers_string + ", ";
        }
        numbers_string = numbers_string + QString::number(called_numbers[i]);
    }

    if(called_numbers.empty()){
        return;
    }
    if(demo_started){
        add_number_to_table(called_numbers.back());
    } else {
        for(int i=0; i<called_numbers.size(); i++){
            add_number_to_table(called_numbers[i]);
        }
        demo_started = true;
    }

}

bool TaskLearning::eventFilter(QObject* watched, QEvent* event)
{
    // New GUI Image Frames
    if (watched == new_gui_ui_.help_image_frame && event->type() == QEvent::Paint) {
        if (!help_qimage_.isNull()){
            drawImageFrame(new_gui_ui_.help_image_frame, help_qimage_);
        } else {
            greyOutImageFrame(new_gui_ui_.help_image_frame);
        }
        return false;
    } else if (watched == new_gui_ui_.card_image_frame && event->type() == QEvent::Paint) {
        if (!card_qimage_.isNull()) {
            drawImageFrame(new_gui_ui_.card_image_frame, card_qimage_);
        } else {
            greyOutImageFrame(new_gui_ui_.card_image_frame);
        }
        return false;
    } else if (watched == new_gui_ui_.eye_image_frame && event->type() == QEvent::Paint) {
        if (!eye_qimage_.isNull()) {
            drawImageFrame(new_gui_ui_.eye_image_frame, eye_qimage_);
        } else {
            greyOutImageFrame(new_gui_ui_.eye_image_frame);
        }
        return false;
    }

    // Gesture Image Frames
    if (watched == gesture_ui_.gesture_img_frame && event->type() == QEvent::Paint) {
        if (!gesture_qimage_.isNull()) {
            drawImageFrame(gesture_ui_.gesture_img_frame, gesture_qimage_);
        } else {
            greyOutImageFrame(gesture_ui_.gesture_img_frame);
        }
        return false;
    }


  return QObject::eventFilter(watched, event);

}

void TaskLearning::greyOutImageFrame(rqt_task_learning::RatioLayoutedFrame *frame){
    // default image with gradient
    QPainter painter(frame);
    QLinearGradient gradient(0, 0, frame->frameRect().width(), frame->frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frame->frameRect().width() + 1, frame->frameRect().height() + 1);
}

void TaskLearning::drawImageFrame(rqt_task_learning::RatioLayoutedFrame *frame, QImage qimage_){
    QMutex qimage_mutex_;
    QPainter painter(frame);
    frame->resizeToFitAspectRatio();
    qimage_mutex_.lock();
    painter.drawImage(frame->contentsRect(), qimage_);
    qimage_mutex_.unlock();

}

void TaskLearning::shutdownPlugin()
{
    help_subscriber_.shutdown();
    card_subscriber_.shutdown();
    eye_subscriber_.shutdown();
    gesture_img_subscriber_.shutdown();
}

void TaskLearning::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void TaskLearning::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    bool zoom1_checked = instance_settings.value("zoom1", false).toBool();

    QString topic = instance_settings.value("topic", "").toString();
    //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
    selectAction(topic);
}

void TaskLearning::updateActionList()
{
    std::map<std::string,std::string> temp_gui_text_action_map = action_executor_client.get_action_list();
    if(stacked_widget->currentWidget() == new_gui_widget_){
        QStringList q_action_list;
        for (auto const& element : temp_gui_text_action_map) {
            QString action = element.first.c_str();
            q_action_list << action;
        }
        QStringListModel *model = new QStringListModel(q_action_list);
        new_gui_ui_.action_list->setModel(model);
    } else if(stacked_widget->currentWidget() == gesture_widget_){
        gesture_ui_.actions_combo_box->clear();
        gesture_ui_.actions_combo_box->addItem("Please choose an action");
        for (auto const& element : temp_gui_text_action_map) {
            QString action = element.first.c_str();
            gesture_ui_.actions_combo_box->addItem(action);
        }
        selectAction("Please choose an action");
    }

    gui_text_action_map = temp_gui_text_action_map;
}

void TaskLearning::selectAction(const QString& action)
{
    if(stacked_widget->currentWidget() == new_gui_widget_){
    } else if(stacked_widget->currentWidget() == gesture_widget_){
        int index = gesture_ui_.actions_combo_box->findText(action);
        if (index == -1)
        {
            index = gesture_ui_.actions_combo_box->findText("");
        }
        gesture_ui_.actions_combo_box->setCurrentIndex(index);
    }


}

int TaskLearning::max_font_size(QString text, QTextEdit *text_browser){
  //text_browser->setWordWrap(true);
  QRect rect=text_browser->contentsRect();
  int height=rect.height()/1.25;
  int width=rect.width()/1.25;
  QFont ft=text_browser->font();
  int fontsz=5;

  while(fontsz<300){
    QFont f(ft);
    f.setPointSize( fontsz );
    QRect r = QFontMetrics(f).boundingRect(0,0,width,height,Qt::TextWordWrap, text );
    if (r.height() <= height && r.width() <= width){
          fontsz=fontsz+5;
    }else{
          break;
    }
  }

  return fontsz;
}

/********************************** New UI **********************************/
void TaskLearning::add_number_to_table(int called_number){
    bool row_empty = false;
    ROS_INFO("Called Number: [%d]", called_number);
    int col = 0;
    if(called_number >= 1 && called_number <= 15){
       col = 0;
    } else if (called_number >= 16 && called_number <= 30) {
       col = 1;
    } else if (called_number >= 31 && called_number <= 45) {
       col = 2;
    } else if (called_number >= 46 && called_number <= 60) {
       col = 3;
    } else if (called_number >= 61 && called_number <= 75) {
       col = 4;
    }
    int row = 0;

    while(!row_empty){

        QStandardItem* item = bingo_number_data->item(row,col);

        if(!item || item->text().isEmpty()){
            bingo_number_data->setItem(row,col,new QStandardItem(QString::number(called_number)));
            new_gui_ui_.bingo_numbers_table->setModel(bingo_number_data);
            QModelIndex next_index = new_gui_ui_.bingo_numbers_table->model()->index(row, col);
            new_gui_ui_.bingo_numbers_table->setCurrentIndex(next_index);

            row_empty = true;
        }
        row++;
    }
}

void TaskLearning::moveEvent(QMoveEvent* event)
{
    const QPoint global = stacked_widget->mapToGlobal(stacked_widget->rect().center());
    waitDialog->move(global.x() - waitDialog->width() / 2, global.y() - waitDialog->height() / 2);
}



QString TaskLearning::get_selected_action(){

    QStringList list;
    foreach(const QModelIndex &index, new_gui_ui_.action_list->selectionModel()->selectedIndexes()){
        list.append(index.data(Qt::DisplayRole ).toString());
    }

    if(list.size() == 1){
        return list[0];
    } else {
        ROS_ERROR("Too many actions selected or no action selected");
        return "";
    }

}

void TaskLearning::blink_action_buttons(){
    /*
    QSize size = new_gui_ui_.execute_action_push_button->iconSize();

    if(green_buttons){
        new_gui_ui_.execute_action_push_button->setIcon(QIcon(":/images/execute.png"));
        new_gui_ui_.execute_action_push_button->setIconSize(size);
        new_gui_ui_.preview_action_button->setIcon(QIcon(":/images/preview.png"));
        new_gui_ui_.preview_action_button->setIconSize(size);
        green_buttons = false;
    } else {
        new_gui_ui_.execute_action_push_button->setIcon(QIcon(":/images/execute_green.png"));
        new_gui_ui_.execute_action_push_button->setIconSize(size);
        new_gui_ui_.preview_action_button->setIcon(QIcon(":/images/preview_green.png"));
        new_gui_ui_.preview_action_button->setIconSize(size);
        green_buttons = true;
    }
    */
}

void TaskLearning::blink_card_state_warning_box(){
    if(green_buttons){
        new_gui_ui_.card_state_text->setStyleSheet("border: 1px solid black; font-size: 14pt;");
        //new_gui_ui_.card_state_warning_box->setStyleSheet("background: rgb(255, 255, 255); " );
        green_buttons = false;
    } else {
        new_gui_ui_.card_state_text->setStyleSheet("border: 1px solid rgb(0, 255, 0); font-size: 14pt;");
        //new_gui_ui_.card_state_warning_box->setStyleSheet("background: rgb(0, 255, 0);" );
        green_buttons = true;
    }
}

void TaskLearning::blink_help_state_warning_box(){
    if(red_buttons){
        new_gui_ui_.help_state_text->setStyleSheet("border: 1px solid black; font-size: 14pt;");
        //new_gui_ui_.help_state_warning_box->setStyleSheet("background: rgb(255, 255, 255);" );
        red_buttons = false;
    } else {
        new_gui_ui_.help_state_text->setStyleSheet("border: 1px solid rgb(0, 255, 0); font-size: 14pt;");
        //new_gui_ui_.help_state_warning_box->setStyleSheet("background: rgb(0, 255, 0);" );
        red_buttons = true;
    }
}

void TaskLearning::onExecuteAction()
{
    if(stacked_widget->currentWidget() == new_gui_widget_){
        QString selected = get_selected_action();
        bool success = handle_action_executions(selected.toStdString(), false, false);
        //reset_action_buttons();
    }

    first_help_action = false;

}

void TaskLearning::onPreviewAction(){
    QString selected = get_selected_action();
    if(new_gui_ui_.show_gestures_check_box->isChecked()){
        bool success = handle_action_executions(selected.toStdString(), true, true);
    } else {
        bool success = handle_action_executions(selected.toStdString(),false, true);
    }
    //reset_action_buttons();
    first_help_action = false;
}

void TaskLearning::onCustomizeAction(){
    QString selected = get_selected_action();
    stacked_widget->setCurrentWidget(gesture_widget_);
    updateActionList();

    demonstration_learning_msgs::world_state world_state = world_state_identifier_client.get_world_state();
    std::string robot_state = world_state.robot_state.data();
    selectAction(selected);
    if(robot_state.compare("At front of room") != 0){
        std_msgs::Float32 goal;
        goal.data = -0.35;
        move_straight_cmd_pub.publish(goal);
    }
    init_gesture_ui();
}

bool TaskLearning::handle_action_executions(std::string action, bool run_gestures, bool run_speech){
    if(!action_executor_client.is_goal_active()){
        action_executor_client.send_action_goal(action, run_gestures, run_speech);
        watch_action_executor_thread_->start();
        waitDialog->push("Tangy is currently performing an action. Please wait until the action is complete.");
        if(!new_gui_ui_.cont_act_select_check_box->isChecked()){
            new_gui_widget_->setEnabled(false);
        }
        //new_gui_ui_.detect_card_button->setVisible(true);
        return true;
    } else {
        QMessageBox msg_box;
        msg_box.setIcon(QMessageBox::Warning);
        msg_box.setText("Warning: Tangy is still trying to complete the action so we will not be able to process your command" );
        msg_box.exec();
        return false;
    }
}

bool TaskLearning::handle_world_state_executions(bool user_activity_state, bool assist_req, bool person_id, bool robot_state){
    if(!world_state_identifier_client.is_goal_active()){
        world_state_goal goal;
        goal.user_activity_state = user_activity_state;
        goal.assist_req = assist_req;
        goal.person_id = person_id;
        goal.robot_state = robot_state;
        world_state_identifier_client.send_identification_goal(goal);
        watch_world_state_thread_->start();
        return true;
    } else {
        QMessageBox msg_box;
        msg_box.setIcon(QMessageBox::Warning);
        msg_box.setText("Warning: Tangy is still trying to detect the world around him we will not be able to process your command" );
        msg_box.exec();
        return false;
    }
}
void TaskLearning::onDetectCard(){
    //new_gui_ui_.detect_card_button->setVisible(false);
    //handle_world_state_executions(true,true,false,true);
    handle_world_state_executions(true,false,false,false);
    waitDialog->push("Tangy is currently detecting the card. Please wait until card detection is finished.");
}

void TaskLearning::updateActionExecutorStatus(){
    //Notify everyone that there is a new state action pair
    std_msgs::String msg;
    msg.data = "new state action pair";
    state_action_notifier.publish(msg);
    get_current_trajectory();
    sleep(1);       /*DO NOT REMOVE THIS SLEEP!!!! IT ENSURES THAT THE ACTIVITY LEARNER OBTAINS THE BINGO DETECTION STATE BEFORE IT CHANGES*/

		if(last_selected_action.compare("CELEBRATE") == 0){
      helping_user = false;
      std_msgs::String notify_help_msg;
      notify_help_msg.data = "stop help";
      notify_help_stop_pub.publish(notify_help_msg);
      new_gui_ui_.image_view_tabs->setCurrentIndex(0);
		}

    if(stacked_widget->currentWidget() == new_gui_widget_){
        if(debug) {
            handle_world_state_executions(false,true,false,false);
        } else {
            if(helping_user == true){
                handle_world_state_executions(true,true,false,true);
            } else {
                handle_world_state_executions(false,true,false,true);
            }
        }
    }

    if(action_executor_client.did_goal_fail()){
        std_msgs::String notify_undo_msg;
        notify_undo_msg.data = "undo action";
        notify_undo_action_pub.publish(notify_undo_msg);
        get_current_trajectory();
        QMessageBox msg_box;
       QString error_msg;
       error_msg = QString::fromStdString(action_executor_client.get_error_msg());
       msg_box.setText(error_msg);
       msg_box.setIcon(QMessageBox::Critical);
       int ret = msg_box.exec();
       return;
    }
}
void TaskLearning::onEndTask(){
    std_msgs::String notify_end_task_msg;
    notify_end_task_msg.data = "end task";
    notify_end_task_pub.publish(notify_end_task_msg);
    save_trajectory();

    std_msgs::String database_save_msg;
    database_save_msg.data = gesture_directory + "/actions.xml";
    save_action_database_pub.publish(database_save_msg);

    stacked_widget->setCurrentWidget(start_menu_widget_);
}

void TaskLearning::onUndoAction(){
    std_msgs::String notify_undo_msg;
    notify_undo_msg.data = "undo action";
    notify_undo_action_pub.publish(notify_undo_msg);
    if(stacked_widget->currentWidget() != new_gui_widget_){
    }
    get_current_trajectory();
}

void TaskLearning::updateWorldStateStatus(){
    if (stacked_widget->currentWidget() == new_gui_widget_){
        demonstration_learning_msgs::world_state world_state = world_state_identifier_client.get_world_state();
        QString text;
        if(world_state.user_activity_state.compare("Missing Numbers") == 0){
            QString miss_num_msg = "Missing the following numbers: ";
            for(int i=0; i<world_state.bad_bingo_numbers.size(); i++){
              miss_num_msg += QString::fromStdString(world_state.bad_bingo_numbers[i]);
              if(i != (world_state.bad_bingo_numbers.size()-1) ){
                miss_num_msg += QString(", ");
              }
            }
            miss_num_msg += QString(". What should Tangy do next?");
            text = miss_num_msg;
            notify_card_state_timer->start(500);
        } else if(world_state.user_activity_state.compare("Incorrectly Marked") == 0){
            QString inc_num_msg = "The following numbers have not been called but were marked: ";
            for(int i=0; i<world_state.bad_bingo_numbers.size(); i++){
              inc_num_msg += QString::fromStdString(world_state.bad_bingo_numbers[i]);
              inc_num_msg += QString(", ");
            }
            inc_num_msg += QString(". What should Tangy do next?");
            text = inc_num_msg;
            notify_card_state_timer->start(500);
        } else if(world_state.user_activity_state.compare("Bingo") == 0){
            text = QString("The card has Bingo. What should Tangy do next?");
            notify_card_state_timer->start(500);
        } else if(world_state.user_activity_state.compare("Occluded") == 0){
            text = QString("Tangy could not see the card. What should Tangy do next?");
            notify_card_state_timer->start(500);
        } else if(world_state.user_activity_state.compare("Correctly Marked") == 0){
            text = QString("The card was marked correctly. What should Tangy do next?");
            notify_card_state_timer->start(500);
        } else {
            notify_card_state_timer->stop();
            new_gui_ui_.card_state_text->setStyleSheet("border: 1px solid black; font-size: 14pt;");
            text = QString("The card was not detected");
        }

        new_gui_ui_.card_state_text->setText(text);

        if(world_state.user_assistance_requests.size() > 0){
            text = "Assistance was requested by "
                    + QString::number(world_state.user_assistance_requests.size())
                    + " people. What should Tangy do next?";

            if(world_state.user_assistance_requests.size() == 1){
                text = text + " person.";
            } else {
                text = text + " people.";
            }

            new_gui_ui_.help_state_text->setText(text);
            if(helping_user == false){
                new_gui_ui_.image_view_tabs->setCurrentIndex(1);
            }
            notify_help_timer->start(500);
        } else {
            text = "Nobody has requested for assistance";
            new_gui_ui_.help_state_text->setText(text);
            notify_help_timer->stop();
            new_gui_ui_.help_state_text->setStyleSheet("border: 1px solid black; font-size: 14pt;");
            //new_gui_ui_.help_state_warning_box->setStyleSheet("background: rgb(255, 255, 255);" );
        }
        waitDialog->pop();
        get_current_trajectory();
        new_gui_widget_->setEnabled(true);
        selectAction("Please choose an action");

        if(helping_user == true && first_help_action == false){
            QMessageBox msg_box;
            msg_box.setText("We are currently providing help.");
            msg_box.setInformativeText("Are you done helping the player?");
            msg_box.setStandardButtons(QMessageBox::Yes| QMessageBox::No);
            msg_box.setDefaultButton(QMessageBox::No);
            int ret = msg_box.exec();

            if(ret == QMessageBox::Yes){
                helping_user = false;
                std_msgs::String notify_help_msg;
                notify_help_msg.data = "stop help";
                notify_help_stop_pub.publish(notify_help_msg);
                new_gui_ui_.image_view_tabs->setCurrentIndex(0);
                /*
                if(!world_state_identifier_client.is_goal_active()){
                    handle_world_state_executions(false,true,false,false);
                }*/
            }
        }

        if(new_gui_ui_.cont_act_select_check_box->isChecked()){
            emit select_new_action();
        }
    }
}

void TaskLearning::new_triangle_callback(const std_msgs::String::ConstPtr& msg){
    if(!world_state_identifier_client.is_goal_active()){
        handle_world_state_executions(false,true,false,false);
    }
}

void TaskLearning::help_start(const std_msgs::String::ConstPtr& msg){
    std::string expected_string = "start help";
    if(expected_string.compare(msg->data) == 0){
        helping_user = true;
        first_help_action = true;
        ROS_INFO("Help was started in action executor server (i.e. move_to_help action was executed)");
    } else {
        ROS_ERROR("Unexpected message published on notify_help_start topic");
    }
    new_gui_ui_.image_view_tabs->setCurrentIndex(2);
}

void TaskLearning::sensor_msg_to_qimage(const sensor_msgs::Image::ConstPtr& msg, QImage &qimage_){
    try
    {
      // First let cv_bridge do its magic
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
      conversion_mat_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
          ROS_ERROR("Something bad happened with the image callback");
    }

    QMutex qimage_mutex_;
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);
    qimage_mutex_.lock();
    qimage_ = image.copy();
    qimage_mutex_.unlock();
}

void TaskLearning::help_image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
  sensor_msg_to_qimage(msg, help_qimage_);
  new_gui_ui_.help_image_frame->setAspectRatio(help_qimage_.width(), help_qimage_.height());
  new_gui_ui_.help_image_frame->update();
}


void TaskLearning::card_image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
  sensor_msg_to_qimage(msg, card_qimage_);
  new_gui_ui_.card_image_frame->setAspectRatio(card_qimage_.width(), card_qimage_.height());
  new_gui_ui_.card_image_frame->update();
}

void TaskLearning::eye_image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
  sensor_msg_to_qimage(msg, eye_qimage_);

  new_gui_ui_.eye_image_frame->setAspectRatio(eye_qimage_.width(), eye_qimage_.height());
  new_gui_ui_.eye_image_frame->update();
}

void TaskLearning::gesture_image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
  sensor_msg_to_qimage(msg, gesture_qimage_);

  gesture_ui_.gesture_img_frame->setAspectRatio(gesture_qimage_.width(), gesture_qimage_.height());
  gesture_ui_.gesture_img_frame->update();
}

void TaskLearning::create_new_training_set(){
    activity_learner::create_new_training_set srv;
    srv.request.create = "create";
    if(new_training_set_client.call(srv)){
        ROS_INFO("New training set created");
    } else {
        ROS_ERROR("Bad service call for create_new_training_set service");
    }
}

std::string TaskLearning::save_trajectory(){
    activity_learner::save_trajectory srv;
    srv.request.save = "save";
    if(save_trajectory_client.call(srv)){
        ROS_INFO("Saved trajectory to path: %s", srv.response.file_path.c_str());
        return srv.response.file_path.c_str();
    } else {
        ROS_ERROR("Bad service call for save_trajectory service");
    }
    return "No file created";
}

void TaskLearning::get_current_trajectory(){
    activity_learner::get_current_trajectory srv;
    std::vector<std::string> header;
    std::vector<std::string> data;
    srv.request.go = "go";
    if(get_trajectory_client.call(srv)){
        ROS_INFO("Getting current trajectory");
        header = srv.response.trajectory_header;
        data = srv.response.trajectory_data;
    } else {
        ROS_ERROR("Bad service call for get_current_trajectory service");
    }

    QStandardItemModel *trajectory_data;


    std::vector<std::string> header_titles = {"Player needs help?", "Observed card", "Robot Position", "Action Taught"};
    trajectory_data = new QStandardItemModel(data.size(),header_titles.size());

    for(int col=0; col<header_titles.size(); col++){
        std::string col_title = header_titles[col];
        trajectory_data->setHorizontalHeaderItem(col, new QStandardItem(QString::fromStdString(col_title)));
    }

    // Initialize Col = 1 because we want to remove Activity State for the GUI
    for(int col=1; col<header.size(); col++){
        //Get data from the 1D array
        for(int row=0; row<data.size()/header.size(); row++){
            std::string row_data = data[row*header.size() + col];
            trajectory_data->setItem(row,col-1,new QStandardItem(QString::fromStdString(row_data)));
            //ROS_INFO("loop");
        }

    }

    new_gui_ui_.trajectory_table->setModel(trajectory_data);
    new_gui_ui_.trajectory_table->resizeColumnsToContents();
    new_gui_ui_.trajectory_table->resizeRowsToContents();
    new_gui_ui_.trajectory_table->horizontalHeader()->setStretchLastSection(true);
    new_gui_ui_.trajectory_table->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
    QModelIndex next_index = new_gui_ui_.trajectory_table->model()->index((data.size()/header.size()-1), 0);
    new_gui_ui_.trajectory_table->setCurrentIndex(next_index);
}

void TaskLearning::create_empty_save_folder(){
    demonstration_learning_msgs::init_action_database srv;
    srv.request.go = "go";
    if(init_action_database_client.call(srv)){
        ROS_INFO("Created empty database with directory: [%s]", srv.response.database_dir.c_str());
        gesture_directory = srv.response.database_dir;
    } else {
        ROS_ERROR("Bad service call for init_action_database service. No new directory returned.");
    }
}


}

PLUGINLIB_EXPORT_CLASS(rqt_task_learning::TaskLearning, rqt_gui_cpp::Plugin)
