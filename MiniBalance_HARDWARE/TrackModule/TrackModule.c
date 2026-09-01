#include "TrackModule.h" 


void TrackModule_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOA clock. DH1-DH4 are PA8, PA11, PA2 and PA3.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 |
                                  GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

u8 TrackModule_ReadTruthValue(void)
{
	u8 dh1;
	u8 dh2;
	u8 dh3;
	u8 dh4;

	dh1 = DH1 ? 1 : 0;
	dh2 = DH2 ? 1 : 0;
	dh3 = DH3 ? 1 : 0;
	dh4 = DH4 ? 1 : 0;

	return (u8)((dh1 << 3) | (dh2 << 2) | (dh3 << 1) | dh4);
}

TrackState TrackModule_GetState(u8 truth_value)
{
	switch (truth_value & 0x0F)
	{
		case 0x00: return TRACK_STATE_CROSSROAD;        // 0000
		case 0x01: return TRACK_STATE_LEFT_RIGHT_ANGLE; // 0001
		case 0x08: return TRACK_STATE_RIGHT_RIGHT_ANGLE;// 1000
		case 0x07: return TRACK_STATE_LEFT_SHARP_TURN;  // 0111
		case 0x0E: return TRACK_STATE_RIGHT_SHARP_TURN; // 1110
		case 0x0B: return TRACK_STATE_LEFT_FINE_TUNE;   // 1011
		case 0x0D: return TRACK_STATE_RIGHT_FINE_TUNE;  // 1101
		case 0x09: return TRACK_STATE_STRAIGHT;         // 1001
		case 0x0F: return TRACK_STATE_LOST;             // 1111
		default:   return TRACK_STATE_UNKNOWN;
	}
}

const char *TrackModule_GetStateName(TrackState state)
{
	switch (state)
	{
		case TRACK_STATE_CROSSROAD:         return "CROSSROAD";
		case TRACK_STATE_LEFT_RIGHT_ANGLE:  return "LEFT_RIGHT_ANGLE";
		case TRACK_STATE_RIGHT_RIGHT_ANGLE: return "RIGHT_RIGHT_ANGLE";
		case TRACK_STATE_LEFT_SHARP_TURN:   return "LEFT_SHARP_TURN";
		case TRACK_STATE_RIGHT_SHARP_TURN:  return "RIGHT_SHARP_TURN";
		case TRACK_STATE_LEFT_FINE_TUNE:    return "LEFT_FINE_TUNE";
		case TRACK_STATE_RIGHT_FINE_TUNE:   return "RIGHT_FINE_TUNE";
		case TRACK_STATE_STRAIGHT:          return "STRAIGHT";
		case TRACK_STATE_LOST:              return "LOST";
		default:                            return "UNKNOWN";
	}
}

// by codex













