cmake_minimum_required(VERSION 2.8.3)
project(demonstration_learning_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs help_indicators geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES world_state.msg bingo_numbers_string.msg customize_action.msg
)

add_service_files(
	 FILES
	 get_current_world_state.srv
         get_action_list.srv
         get_last_executed_action.srv
         load_last_state.srv
         exe_spch_and_gest.srv
	 init_action_database.srv
	 preview_customization.srv
)


generate_messages(DEPENDENCIES
  std_msgs geometry_msgs help_indicators)

catkin_package()

install(DIRECTORY include/demonstration_learning_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
