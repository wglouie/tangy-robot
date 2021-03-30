#include <trivia/trivia.h>


vector<string> question0;
vector<string> question1;

void init(){
    // robotGuiClient("test_gui");
    //Store fake questions
    
    question0.push_back("What is the process by which green plants convert the energy of sunlight into chemical energy?");
    question0.push_back("Osmosis");
    question0.push_back("Hydrolysis");
    question0.push_back("Photosynthesis");
    question0.push_back("A3");
    question0.push_back("This process uses carbon dioxide, water, and sunlight to produce sugar and oxygen.");
    
    
    question1.push_back("Who plays Catherine the Great in the 1934 film The Scarlet Empress?");
    question1.push_back("Tallulah Bankhead");
    question1.push_back("Marlene Dietrich");
    question1.push_back("Greta Garbo");
    question1.push_back("A2");
    question1.push_back("This actress received honors from multiple countries for her work improving morale on the front lines in World War Two.");
}

/*Say and display on the screen the same string*/
robot_gui::Robot_guiGoal say_and_display(std::string activity, std::string str0, std::string str1, int subtab){
    robot_gui::Robot_guiGoal goal;
    goal.activity = activity;
    goal.code=2;
    goal.speech=str0;
    goal.text=str1;
    goal.subtab=subtab;
    return goal;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_gui");
    ros::NodeHandle nh;
    RobotGuiClient robotGuiClient("test gui");
    init();
    
    string qs= question1.at(0);
    robot_gui::Robot_guiGoal goal1=say_and_display("trivia", qs, qs, 0);
    robotGuiClient.sendGoalAndWait(goal1);
    
    string fmt_str=question1.at(0)+"\n A: "+question1.at(1)+"\n B: "+question1.at(2)+"\n C: "+question1.at(3);

    robot_gui::Robot_guiGoal goal2=say_and_display("trivia", question1.at(1), fmt_str, 2);
    robotGuiClient.sendGoalAndWait(goal2);
    robot_gui::Robot_guiGoal goal3=say_and_display("trivia", question1.at(2), fmt_str, 2);
    robotGuiClient.sendGoalAndWait(goal3);
    robot_gui::Robot_guiGoal goal4=say_and_display("trivia", question1.at(3), fmt_str, 2);
    robotGuiClient.sendGoalAndWait(goal4);
    sleep(4);

    return 0;
}