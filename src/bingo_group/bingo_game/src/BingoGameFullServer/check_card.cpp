#include <BingoGameFullServer/BingoGameFullServer.h>

/*Communicates with the bingo search client to determine if the bingo card has been correctly
filled out*/

void BingoGameFullServer::check_bingo_card() {
  ROS_INFO("Checking card now.");
  //ros::spinOnce();
  display_text("Checking Bingo Card");
  look_at_card_1();

  bingoDetectionClient.sendGoalAndWait(numberHandler.get_previously_called_numbers());
  string gameState=bingoDetectionClient.get_GameState();
  if(gameState.compare("error")==0 || gameState.compare("unknown")==0){
    bingoDetectionClient.sendGoalAndWait(numberHandler.get_previously_called_numbers());
  }
  gameState=bingoDetectionClient.get_GameState();
  if(gameState.compare("error")==0 || gameState.compare("unknown")==0){
    //ros::spinOnce();
    display_text("Checking Bingo Card");
    look_at_card_2();
    bingoDetectionClient.sendGoalAndWait(numberHandler.get_previously_called_numbers());
  }
  gameState=bingoDetectionClient.get_GameState();
  if(gameState.compare("error")==0 || gameState.compare("unknown")==0){
    bingoDetectionClient.sendGoalAndWait(numberHandler.get_previously_called_numbers());
  }
  gameState=bingoDetectionClient.get_GameState();
  if(gameState.compare("error")==0 || gameState.compare("unknown")==0){
    //ros::spinOnce();
    display_text("Checking Bingo Card");
    look_at_card_3();
    bingoDetectionClient.sendGoalAndWait(numberHandler.get_previously_called_numbers());
  }
  gameState=bingoDetectionClient.get_GameState();
  if(gameState.compare("error")==0 || gameState.compare("unknown")==0){
    bingoDetectionClient.sendGoalAndWait(numberHandler.get_previously_called_numbers());
  }
  //ros::spinOnce();
  look_back_to_person();
  display_text("Checking Bingo Card");

}

