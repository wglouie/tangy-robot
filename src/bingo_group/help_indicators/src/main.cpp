#include <help_indicators/help_indicators.h>

int main(int argc, char** argv) {
	ros::init( argc, argv, "help_node" );
	ros::NodeHandle n;
	Help help_object(n);

	ros::Subscriber go_sub = n.subscribe("help_indicators_go", 1, &Help::go_cb, &help_object);

	ros::Subscriber clear_sub = n.subscribe("clear_help_indicator", 1, &Help::clear_help_indicators_cb, &help_object);
	
	ros::spin();

	return 0;
}
