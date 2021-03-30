#include <BingoGameFullServer/BingoGameFullServer.h>

/*Joke interaction. Jokes are in the vector format {question, answer}*/
void BingoGameFullServer::tell_joke() {
	if(pause_game) {
		pause_everything();
	}
	state_pub("Interaction");
	if(end_game == false) {
		vector<string> joke = jokeHandler.get_rand_joke();

		if(joke.at(0).size() != 1){
      		set_music_volume(QUIET);
			// convo_gesture();
			say_and_display("How about a joke?");
			// convo_gesture();
			say_and_display(joke.at(0));
			// convo_gesture();
			
			sleep(2);
			int rand_n=rand()%100;
			if(rand_n < 50){
				laugh_gesture();
			}
			
			say_and_display(joke.at(1));
			say_and_display("Hee! hee! hee!",joke.at(1));
      		set_music_volume(LOUD);
		}
	}
}

/*Small Talk interaction.*/
void BingoGameFullServer::make_small_talk() {
	if(pause_game){
		pause_everything();
	}
	state_pub("Interaction");
	if(end_game == false) {
	  set_music_volume(QUIET);
		// convo_gesture();
		string small_talk = smallTalkHandler.get_rand_line();

		if(small_talk.size() != 0){
			say_and_display(small_talk, "Did you know?");
            //sleep(1);
		}
		set_music_volume(LOUD);
	}

}
