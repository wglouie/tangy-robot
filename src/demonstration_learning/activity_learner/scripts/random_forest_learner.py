#!/usr/bin/env python
import numpy as np
import sys
import csv
import cPickle
from pprint import pprint
from sklearn.feature_extraction import DictVectorizer
from sklearn.ensemble import RandomForestClassifier
import os
import rospkg
import rospy
from std_msgs.msg import String

from demonstration_learning_msgs.srv import *
from activity_learner.srv import *


class ActivityLearner(object):
    def __init__(self):
        self.node = rospy.init_node('activity_learner_server')
        self.ros_pack = rospkg.RosPack()

        self.training_set_path = rospy.get_param('training_set_path',
                                                 self.ros_pack.get_path('activity_learner') +
                                                 '/train/trajectory.txt')
        self.save_path = rospy.get_param('save_path',
                                         self.ros_pack.get_path('activity_learner') +
                                         '/train/save/')
        # Create Random Forest classifier
        self.clf = RandomForestClassifier()
        self.feature_vec = DictVectorizer()

        self.notify_undo_action_sub = rospy.Subscriber('rqt_task_learning/notify_undo_action', String,
                                                       self.handle_undo_action)
        self.state_action_notifier_sub = rospy.Subscriber('rqt_task_learning/state_action_notifier', String,
                                                          self.handle_record_training_data)
        self.create_train_set_srv = rospy.Service('activity_learner/create_new_training_set', create_new_training_set,
                                                  self.handle_create_new_training_set)
        self.train_policy_srv = rospy.Service('activity_learner/train_policy', train_policy, self.handle_train_policy)
        self.predict_action_srv = rospy.Service('activity_learner/predict_action', predict_action,
                                                self.handle_predict_action)
        self.probabilistic_predict_action_srv = rospy.Service('activity_learner/probabilistic_predict_action',
                                                              probabilistic_predict_action,
                                                              self.handle_probabilistic_predict_action)
        self.save_trajectory_srv = rospy.Service('activity_learner/save_trajectory', save_trajectory,
                                                 self.handle_save_trajectory)
        self.get_current_trajectory_srv = rospy.Service('activity_learner/get_current_trajectory',
                                                        get_current_trajectory, self.handle_get_trajectory)
        rospy.spin()

    def get_last_action_client(self):
        # rospy.wait_for_service('action_executor/get_last_action_executed')
        try:
            self.get_last_action = rospy.ServiceProxy('action_executor/get_last_executed_action',
                                                      get_last_executed_action)
            res = self.get_last_action('go')
            return res.last_action
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_world_state_client(self):
        # rospy.wait_for_service('world_state_identifier/get_current_world_state')
        try:
            self.get_world_state = rospy.ServiceProxy('world_state_identifier/get_current_world_state',
                                                      get_current_world_state)
            res = self.get_world_state('go');
            return res.current_world_state
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def state_action_pair_string(self, world_state, action):
        discretized_state = []
        discretized_state.append(world_state.activity_state)

        if len(world_state.user_assistance_requests) > 0:
            discretized_state.append("Yes")
        else:
            discretized_state.append("No")

        discretized_state.append(world_state.user_activity_state)
        discretized_state.append(world_state.robot_state)
        discretized_state.append(action)
        return discretized_state

    def handle_get_trajectory(self, req):
        # Extract training features and Actions from file and put them in dictionary objects
        trajectory_header = ['Activity_State', 'Help_State', 'Card_State', 'Robot_State', 'Action']
        trajectory_data = []

        try:
            with open(self.training_set_path, 'r') as f:
                records = csv.DictReader(f, delimiter="\t")
                for row in records:
                    for col in trajectory_header:
                        trajectory_data.append(row[col])
                        #{key: row[key] for key in
                        #['Activity_State', 'Help_State', 'Card_State', 'Robot_State', 'Action']})
            return get_current_trajectoryResponse(trajectory_header, trajectory_data)
        except IOError:
            rospy.logerr("ERROR: could not find path \'" + self.training_set_path + "\'. Cannot train policy")
            return get_current_trajectoryResponse(trajectory_header, trajectory_data)

    def handle_undo_action(self, data):
        try:
            with open(self.training_set_path) as f:
                lines = f.readlines()
                f.close()
                w = open(self.training_set_path, 'w')
                w.writelines([item for item in lines[:-1]])
                w.close()
        except IOError:
            rospy.logerr("ERROR: could not find path \'" + self.training_set_path + "\'. Can't undo action.")

    def handle_record_training_data(self, data):
        print "attemping to record"
        last_action = self.get_last_action_client()
        world_state = self.get_world_state_client()
        discretized_state = self.state_action_pair_string(world_state, last_action)
        try:
            with open(self.training_set_path, 'a') as f:
                training_set_writer = csv.writer(f, delimiter="\t")
                training_set_writer.writerow(discretized_state)
        except IOError:
            rospy.logerr("ERROR: could not find path \'" + self.training_set_path + "\'. Can't record training data.")

    def handle_create_new_training_set(self, req):
        try:
            with open(self.training_set_path, 'w') as f:
                training_set_writer = csv.writer(f, delimiter="\t")
                training_set_writer.writerow(['Activity_State', 'Help_State', 'Card_State', 'Robot_State', 'Action'])
            rospy.loginfo("Created new training set")
            return create_new_training_setResponse(True)

        except IOError:
            rospy.logerr("ERROR: could not find path \'" + self.training_set_path + "\'. Not creating new training set")
            return create_new_training_setResponse(False)

    def handle_save_trajectory(self, req):
        try:
            with open(self.training_set_path) as f:
                lines = f.readlines()

            i = 0
            while os.path.exists(self.save_path + "saved_trajectory%s.txt" % i):
                i += 1

            w = open(self.save_path + "saved_trajectory%s.txt" % i, "w")
            w.writelines([item for item in lines[:-1]])
            w.close()
            return save_trajectoryResponse(self.save_path + "saved_trajectory%s.txt" % i)

        except IOError:
            rospy.logerr("ERROR: could not find path \'" + self.training_set_path + "\'. Can't save trajectory.")
            return save_trajectoryResponse("No File Saved!")

    def handle_train_policy(self, req):
        # Extract training features and Actions from file and put them in dictionary objects
        train_features = []
        class_labels = []
        training_set_file = req.training_set_path;

        try:
            with open(training_set_file, 'r') as f:
                records = csv.DictReader(f, delimiter="\t")
                for row in records:
                    # print row
                    train_features.append(
                        {key: row[key] for key in ['Activity_State', 'Help_State', 'Card_State', 'Robot_State']})
                    class_labels.append(row['Action'])

        except IOError:
            rospy.logerr("ERROR: could not find path \'" + training_set_file + "\'. Cannot train policy")
            return train_policyResponse(False)

        if len(train_features) == 0:
            rospy.logerr("ERROR: There is no data to train! Record some state-action pairs first.")
            return train_policyResponse(False)

        # One-hot encoding for categorical variables
        self.feature_vec.fit_transform(train_features).toarray()
        training_set = self.feature_vec.transform(train_features).toarray()

        # Create Random Forest classifier and fit model to training data
        self.clf = self.clf.fit(training_set, class_labels)

        rospy.loginfo("Policy has been trained")
        return train_policyResponse(True)

    def world_state_to_dict(self, world_state):
        formatted_world_state = []
        dict_world_state = {}
        dict_world_state['Activity_State'] = world_state.activity_state

        if len(world_state.user_assistance_requests) > 0:
            dict_world_state['Help_State'] = "Yes"
        else:
            dict_world_state['Help_State'] = "No"

        dict_world_state['Card_State'] = world_state.user_activity_state
        dict_world_state['Robot_State'] = world_state.robot_state
        formatted_world_state.append(dict_world_state)
        return formatted_world_state

    def handle_predict_action(self, req):
        test_set_features = []
        # Extract test set from file and put them in dictionary objects
        # with open(self.ros_pack.get_path('activity_learner') +'/observations.txt', 'r') as f:
        #	records = csv.DictReader(f, delimiter = "\t")
        #	for row in records:
        #		test_set_features.append({key:row[key] for key in ['Activity_State',	'Help_State',	'Card_State']})

        test_set_features = self.world_state_to_dict(req.current_world_state)
        print test_set_features
        try:
            # One-hot encoding for categorical variables
            test_set = self.feature_vec.transform(test_set_features).toarray()
            # Make a prediction
            predicted_action = self.clf.predict(test_set)
            # print clf.predict_proba(test_set)
        except AttributeError:
            rospy.logerr("ERROR: Policy has not been trained yet. Please train the policy before trying to predict")
            return predict_actionResponse("Fail to predict action")

        return predict_actionResponse(predicted_action[0])

    def handle_probabilistic_predict_action(self, req):
        test_set_features = []
        # Extract test set from file and put them in dictionary objects
        # with open(package_path + '/observation.txt', 'r') as f:
        #	records = csv.DictReader(f, delimiter = "\t")
        #	for row in records:
        #		test_set_features.append({key:row[key] for key in ['Activity_State',	'Help_State',	'Card_State']})

        test_set_features = self.world_state_to_dict(req.current_world_state)
        print test_set_features

        try:
            # One-hot encoding for categorical variables
            test_set = self.feature_vec.transform(test_set_features).toarray()

            # Make a prediction
            class_probabilities = self.clf.predict_proba(test_set)
            # print clf.predict_proba(test_set)
        except AttributeError:
            rospy.logerr("ERROR: Policy has not been trained yet. Please train the policy before trying to predict")
            return probabilistic_predict_actionResponse(['Policy not trained'], [0])

        return probabilistic_predict_actionResponse(self.get_class_list(), class_probabilities[0].tolist())

    def get_class_list(self):
        return self.clf.classes_.tolist()


def main():
    learner = ActivityLearner()


if __name__ == "__main__":
    main()
