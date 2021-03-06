#ifndef WORLD_STATE_ENUM_H
#define WORLD_STATE_ENUM_H

#include <string>

enum CardState {BINGO=0,INCORRECTLY_MARKED,MISSING_NUMBERS, CORRECTLY_MARKED, OCCLUDED};
enum HelpState {ASSISTANCE_NOT_REQUIRED=0, ASSISTANCE_REQUIRED};
enum ActivityState {START=0,SOCIALIZE,FACILITATE,HELP,NAVIGATE,END};
//enum SimulationState {START_SCREEN, ACTIVITY};
//enum Move {DOWN=0, LEFT, RIGHT, UP, NUMBER_OF_DIRECTIONS};
//enum EntityTypes {ENTITY, ROBOT, PERSON, PLAYER, BINGO_PLAYER};
//enum RobotBehaviorType {GREETING=0, CALLING, REMOVEMARKER, MARKNUMBER, ENCOURAGE, CELEBRATE, JOKE, NAVIGATING_TO_FRONT, NAVIGATING_TO_HELP, VALEDICTION, NOACTION, SHOWBINGOCARD,NUMBER_OF_BEHAVIORS = SHOWBINGOCARD + 1};
//enum Emotion {HAPPY,SAD,ANGRY};
//enum Gesture {POINT,WAVE};
//enum PersuasionType {NEUTRAL=0, PRAISE, SCARCITY, SUGGESTION, NUMBER_OF_COMPLIANCE_TYPES = SUGGESTION +1,NO_PERSUASION};
//Unused Persuasive strategies: RECIPROCATION, AUTHORITY, CONSENSUS,LIKING, COMMITMENT
#endif 
