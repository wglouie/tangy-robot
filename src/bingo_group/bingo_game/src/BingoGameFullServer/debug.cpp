#include <BingoGameFullServer/BingoGameFullServer.h>

//debug printer
void BingoGameFullServer::debug_print(std::string string) {
	debug_string.data = string;
	debug_pub.publish(debug_string);
}
   
