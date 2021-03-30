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

#ifndef rqt_task_learning__TaskLearning_H
#define rqt_task_learning__TaskLearning_H

#include <rqt_gui_cpp/plugin.h>
#include "ros/ros.h"
#include "ros/package.h"

#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <ui_policy_gui.h>
#include <ui_start_menu.h>
#include <ui_new_gui.h>
#include <ui_gesture.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>

#include <QImage>
#include <QList>
#include <QMutex>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QStackedWidget>
#include <QFuture>
#include <QtConcurrentRun>
#include <QThread>
#include <QStandardItemModel>
#include <QTableWidgetItem>
#include <QDialog>
#include <QTimer>
#include <QMessageBox>
#include <QStringListModel>
#include <QKeyEvent>
#include <QInputDialog>
#include <QDir>

#include <vector>
#include <action_executor_client.h>
#include <world_state_identifier_client.h>
#include <activity_learner/create_new_training_set.h>
#include <activity_learner/predict_action.h>
#include <activity_learner/train_policy.h>
#include <activity_learner/probabilistic_predict_action.h>
#include <activity_learner/save_trajectory.h>
#include <activity_learner/get_current_trajectory.h>
#include <demonstration_learning_msgs/customize_action.h>
#include <demonstration_learning_msgs/init_action_database.h>
#include <boost/filesystem.hpp>
namespace rqt_task_learning {

enum CustomizationState{SPEECH, GESTURE, SPEECH_GESTURE, NONE};

class WatchActionExecutorThread
  : public QThread
{
    ActionExecutorClient *action_executor_client;
    Q_OBJECT
public:
    WatchActionExecutorThread(ActionExecutorClient *input_client);
    void run();
signals:
    void action_complete_signal();
};

class WatchWorldStateThread
  : public QThread
{
    WorldStateIdentifierClient *world_state_identifier_client;
    Q_OBJECT
public:
    WatchWorldStateThread(WorldStateIdentifierClient *input_client);
    void run();
signals:
    void identification_complete_signal();
};
/*
class SkeletonWatcher
  : public QObject
{
    Q_OBJECT
private:
    int num_skeletons;
    QMutex *m_mutex;

public:
    SkeletonWatcher(QObject *parent = 0);
    void update_num_skeletons(int value);
public slots:
    void track_skeletons();
signals:
    void skeleton_tracked_signal();
};
*/
class SkeletonWatcher
  : public QObject
{
    Q_OBJECT
private:
    int num_skeletons;
    QMutex *m_mutex;
public:
    SkeletonWatcher(QObject *parent = 0);
    void update_num_skeletons(int value);
public slots:
    void track_skeletons();
signals:
    void done();
};

class GestureSpeechExecutor
  : public QObject
{
    Q_OBJECT
private:
    ActionExecutorClient *action_executor_client;
    std::string speech;
    std::string gesture_containing_package;
    std::string gesture_relative_file_path;
		bool turn_off_display;
public:
    GestureSpeechExecutor(ActionExecutorClient *input_client,QObject *parent = 0);
    void set_spch_n_gest(std::string speech, std::string gesture_containing_package, std::string gesture_relative_file_path, bool turn_off_display = false);
public slots:
    void run_spch_n_gest();
signals:
    void done();
};

class WaitDialog : public QDialog
{
    Q_OBJECT

    public:
        WaitDialog(const QString& message, QWidget *parent = 0);

public slots:
    void show();
    void hide();
    void push(QString msg = "");
    void pop();

signals:
        void visibilityChanged(bool enable);

private:
    int stack;
        QLabel* msgLabel;
};

class WirelessDeviceReceiver : public QObject
{
    Q_OBJECT
protected:
    bool eventFilter(QObject* obj, QEvent* event);

signals:
    void start_record_signal();
    void stop_record_signal();
};


class TaskLearning
    : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:QVBoxLayout *layout = new QVBoxLayout;

    TaskLearning();
    ~TaskLearning();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    virtual bool eventFilter(QObject* watched, QEvent* event);

    virtual void greyOutImageFrame(rqt_task_learning::RatioLayoutedFrame *frame);

    virtual void drawImageFrame(rqt_task_learning::RatioLayoutedFrame *frame, QImage qimage_);

    virtual void sensor_msg_to_qimage(const sensor_msgs::Image::ConstPtr& msg, QImage &qimage_);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

    virtual void updateActionList();
    virtual void updateActionExecutorStatus();
    virtual void updateWorldStateStatus();
protected:

    virtual void selectAction(const QString& action);
    virtual void help_start(const std_msgs::String::ConstPtr& msg);

signals:
    void msgbox_signal(int icon,QString msg);
    void run_gest_spch_signal();
    void watch_for_skeletons();
    void select_new_action();

protected slots:

    virtual void onExecuteAction();
    virtual void onPreviewAction();
    virtual void onEndTask();
    virtual void onUndoAction();
    virtual void onActionChanged(int);

    //General
    virtual void create_message_box(int icon, QString msg);

    //Start Menu
    virtual void onNewGUINewGame();
    virtual void onNewGUIContinueGame();
    virtual void onNewGUIDebug();
    virtual void onGesture();

    //Old GUI
    virtual void onPredict();
    virtual void onProbPredict();
    virtual void onTrain();


    //New GUI
    virtual void onDetectCard();
    virtual void blink_action_buttons();
    virtual void blink_card_state_warning_box();
    virtual void blink_help_state_warning_box();
    virtual void onCustomizeAction();

    //Gesture Learning
    virtual void onStartRecordGesture();
    virtual void onStopRecordGesture();
    virtual void onRedoCustomization();
    virtual void onPreviewCustomization();
    virtual void onSaveCustomization();
    virtual void onCustomGesture();
    virtual void onCustomSpeech();
    virtual void onCustomSpeechGesture();
    virtual void onCancelCustomization();
    virtual void onYesDoneGesture();
    virtual void onNoDoneGesture();
    virtual void onFinishedSpeech();
    virtual void instruct_psi_pose();
    virtual void instruct_start_record();
    virtual void instruct_stop_record();
    virtual void spch_gest_done();

protected:

    //General
    virtual void help_image_cb(const sensor_msgs::Image::ConstPtr& msg);
    virtual void card_image_cb(const sensor_msgs::Image::ConstPtr& msg);
    virtual void eye_image_cb(const sensor_msgs::Image::ConstPtr& msg);
    virtual void gesture_image_cb(const sensor_msgs::Image::ConstPtr& msg);
    virtual void called_numbers_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    virtual void create_new_training_set();
    virtual void get_current_trajectory();
    virtual bool handle_action_executions(std::string action, bool run_gestures, bool run_speech);
    virtual bool handle_world_state_executions(bool user_activity_state, bool assist_req, bool person_id, bool robot_state);
    virtual std::string save_trajectory();
    virtual void moveEvent(QMoveEvent* event);

    //New GUI Functions
    virtual QString get_selected_action();
    virtual void new_triangle_callback(const std_msgs::String::ConstPtr& msg);
    virtual void add_number_to_table(int called_number);
    virtual std::string sample_class_probs(std::vector<std::string> class_list, std::vector<float> probabilities);
    //Gesture functions
    virtual void create_empty_save_folder();
    virtual void change_text(QString text);
    virtual int max_font_size(QString text, QTextEdit *text_browser);
    virtual void init_gesture_ui();
    virtual void skel_status_cb(const std_msgs::Int8::ConstPtr& msg);
    virtual void finish_customization(QString input_speech= " ");

    Ui::StartMenuWidget start_menu_ui_;
    Ui::NewGuiWidget new_gui_ui_;
    Ui::GestureWidget gesture_ui_;

    QWidget* start_menu_widget_;
    QWidget* new_gui_widget_;
    QWidget* gesture_widget_;
    QStackedWidget *stacked_widget;

    image_transport::Subscriber help_subscriber_;
    image_transport::Subscriber card_subscriber_;
    image_transport::Subscriber eye_subscriber_;
    image_transport::Subscriber gesture_img_subscriber_;
    ros::Publisher state_action_notifier;
    ros::Publisher notify_help_stop_pub;
    ros::Publisher notify_end_task_pub;
    ros::Publisher notify_undo_action_pub;
    ros::Publisher learning_state_pub;
    ros::Publisher record_traj_pub;
    ros::Publisher save_traj_pub;
    ros::Publisher exec_traj_pub;
    ros::Publisher save_action_database_pub;
    ros::Publisher customize_action_pub;
    ros::Publisher move_straight_cmd_pub;
    ros::Publisher set_arms_neutral_pub;
    ros::Subscriber notify_help_start_sub;
    ros::Subscriber called_numbers_sub;
    ros::Subscriber skel_status_sub;
    ros::Subscriber notify_new_tri_sub;
    ros::ServiceClient new_training_set_client;
    ros::ServiceClient train_policy_client;
    ros::ServiceClient predict_action_client;
    ros::ServiceClient prob_predict_action_client;
    ros::ServiceClient save_trajectory_client;
    ros::ServiceClient get_trajectory_client;
    ros::ServiceClient init_action_database_client;
    bool helping_user;
    bool demo_started;
    bool debug;
    bool green_buttons;
    bool red_buttons;
    bool first_help_action;
    bool recording_gesture;

    QImage eye_qimage_;
    QImage help_qimage_;
    QImage card_qimage_;
    QImage gesture_qimage_;

    cv::Mat conversion_mat_;

    ActionExecutorClient action_executor_client;
    WorldStateIdentifierClient world_state_identifier_client;

    QThread *watch_action_executor_thread_;
    QThread *watch_world_state_thread_;
    QThread *watch_skeleton_thread_;
    QThread *watch_gest_spch_exe_thread_;

    QStandardItemModel *bingo_number_data;

    WaitDialog* waitDialog;

    QTimer *notify_card_state_timer;
    QTimer *notify_help_timer;

    std::string gesture_directory;
    CustomizationState customization_state;
    QString final_speech;
    QString final_gesture_file_path;
    WirelessDeviceReceiver* wireless_device_receiver;
    bool gesture_process_started;
    int number_of_skeletons;
    SkeletonWatcher *skele_watcher;
    GestureSpeechExecutor *gest_spch_exe;
    bool is_tracking_skeletons;
    std::map<std::string,std::string> gui_text_action_map;
		std::string last_selected_action;

};


}

#endif // rqt_task_learning__TaskLearning_H
