#ifndef __TRACKMODULE_H
#define __TRACKMODULE_H
#include "sys.h"

#define DH4 PAin(3)
#define DH3 PAin(2)
#define DH2 PAin(11)
#define DH1 PAin(8)

typedef enum
{
	TRACK_STATE_UNKNOWN = 0,
	TRACK_STATE_CROSSROAD = 1,
	TRACK_STATE_LEFT_RIGHT_ANGLE = 2,
	TRACK_STATE_RIGHT_RIGHT_ANGLE = 3,
	TRACK_STATE_LEFT_SHARP_TURN = 4,
	TRACK_STATE_RIGHT_SHARP_TURN = 5,
	TRACK_STATE_LEFT_FINE_TUNE = 6,
	TRACK_STATE_RIGHT_FINE_TUNE = 7,
	TRACK_STATE_STRAIGHT = 8,
	TRACK_STATE_LOST = 9
} TrackState;

void TrackModule_Init(void);
u8 TrackModule_ReadTruthValue(void);
TrackState TrackModule_GetState(u8 truth_value);
const char *TrackModule_GetStateName(TrackState state);


#endif

// by codex

