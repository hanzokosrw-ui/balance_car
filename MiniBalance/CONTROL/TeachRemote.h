#ifndef TEACH_REMOTE_H
#define TEACH_REMOTE_H
#include <stdint.h>

/* Bluetooth (9600 baud, uppercase ASCII outside parameter frames):
 * R -> record a new route, discarding the old route and starting stopped.
 * A/E -> forward/back; B/C/D -> right; F/G/H -> left; Z -> stop.
 * X/Y -> speed up/down (100 mm/s); legacy raw bytes 0..9 also work.
 * Q -> save/abort and stop in Normal mode; T -> save/abort and patrol.
 * P -> restore starting speed/gear, replay once, then stop in Normal mode.
 * Example: R, A, wait, C, wait, Z, wait, Q; later P.
 * R/P require the existing balance safety checks to permit motor operation.
 * This replays timed commands, NOT measured position or autonomous avoidance.
 * Reposition the car to its original pose before playback; power loss erases it.
 * During playback, live motion commands are ignored; use Q to interrupt.
 */

#define TEACH_TICK_MS     10u
#define TEACH_MAX_EVENTS  512u
#define TEACH_MAX_TICKS   60000u /* Ten minutes; one RAM-only route. */
#define TEACH_MAX_SPEED   800u   /* mm/s; prevent unsigned speed overflow. */

typedef enum {
    TEACH_IDLE = 0, TEACH_RECORDING, TEACH_SAVED, TEACH_PLAYING,
    TEACH_FINISHED, TEACH_EMPTY, TEACH_BUFFER_FULL, TEACH_TIME_LIMIT,
    TEACH_SAFETY_STOP, TEACH_RX_OVERFLOW
} TeachRemoteStatus;

/* Read-only debugger status; duration includes initial/trailing stops. */
extern volatile TeachRemoteStatus Teach_Status;
extern volatile uint16_t Teach_EventCount;
extern volatile uint32_t Teach_DurationTicks;
void TeachRemote_Init(void); /* Before enabling UART interrupts. */
void TeachRemote_ReceiveByte(uint8_t byte); /* USART3 producer only. */
void TeachRemote_Tick(uint8_t safe); /* Once per 10ms, control ISR only. */
void TeachRemote_SetMode(uint8_t mode); /* Board key, control ISR only. */
void TeachRemote_SafetyStop(void);
uint8_t TeachRemote_IsActive(void);
#endif
/* by codex */
