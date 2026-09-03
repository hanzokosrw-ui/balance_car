#include "control.h"
#include "TeachRemote.h"
#include "TrackModule.h"

#define RX_SIZE 128u
#define FRAME_SIZE 50u
#define FRAME_TIMEOUT_TICKS 100u

typedef struct {
    uint32_t tick;
    uint8_t command;
} TeachEvent;
static TeachEvent route[TEACH_MAX_EVENTS];
static uint32_t elapsed;
static uint16_t play_index, initial_speed, initial_gear;
/* Only USART3 writes head; only the control ISR writes tail. */
static volatile uint8_t rx[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_overflow;
static uint8_t frame[FRAME_SIZE], frame_length, in_frame, frame_invalid;
static uint16_t frame_age;
volatile TeachRemoteStatus Teach_Status = TEACH_IDLE;
volatile uint16_t Teach_EventCount;
volatile uint32_t Teach_DurationTicks;

static void stop_motion(void)
{
    Flag_front = Flag_back = Flag_Left = Flag_Right = 0;
    Move_X = Move_Z = 0;
    Ros_Rate = 0;
}

uint8_t TeachRemote_IsActive(void)
{
    return Mode == Teach_Record_Mode || Mode == Teach_Play_Mode;
}

static void change_mode(uint8_t mode)
{
    uint32_t irq_mask = __get_PRIMASK();
    if (mode == ROS_Mode) mode = Normal_Mode;
    /* UART1 can preempt control: publish mode and clear ROS atomically. */
    __disable_irq();
    if (Mode == Teach_Record_Mode) {
        Teach_DurationTicks = elapsed;
        Teach_Status = TEACH_SAVED;
    }
    Mode = mode;
    stop_motion();
    __set_PRIMASK(irq_mask);
    if (mode == IRDM_Line_Patrol_Mode) TrackModule_Init();
}

static void reset_input(void)
{
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    rx_tail = rx_head;
    rx_overflow = 0;
    __set_PRIMASK(irq_mask);
    in_frame = frame_invalid = frame_length = 0;
    frame_age = 0;
}

void TeachRemote_Init(void)
{
    rx_head = rx_tail = 0;
    reset_input();
    elapsed = 0;
    play_index = 0;
    Teach_EventCount = 0;
    Teach_DurationTicks = 0;
    Teach_Status = TEACH_IDLE;
}

void TeachRemote_SetMode(uint8_t mode)
{
    /* Key cycling cannot enter teach modes without initialization. */
    if (mode == Teach_Record_Mode || mode == Teach_Play_Mode) return;
    change_mode(mode);
    if (Teach_Status == TEACH_PLAYING) Teach_Status = TEACH_SAVED;
    reset_input();
}

void TeachRemote_SafetyStop(void)
{
    if (TeachRemote_IsActive()) {
        change_mode(Normal_Mode);
        Teach_Status = TEACH_SAFETY_STOP;
        reset_input();
    }
}

void TeachRemote_ReceiveByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_head + 1u) % RX_SIZE);
    if (rx_overflow) return;
    if (next == rx_tail) { rx_overflow = 1; return; }
    rx[rx_head] = byte;
    rx_head = next;
}

static uint8_t is_action(uint8_t byte)
{
    return byte <= 9u || (byte >= 'A' && byte <= 'H') ||
           byte == 'Z' || byte == 'X' || byte == 'Y';
}

/* Live remote and playback share this exact motor-target mapping. */
static void apply_action(uint8_t byte)
{
    stop_motion();
    if (byte <= 9u) Flag_velocity = 1;
    if (byte == 'A' || byte == 1u) Flag_front = 1;
    else if (byte == 'E' || byte == 5u) Flag_back = 1;
    else if ((byte >= 'B' && byte <= 'D') || (byte >= 2u && byte <= 4u))
        Flag_Right = 1;
    else if ((byte >= 'F' && byte <= 'H') || (byte >= 6u && byte <= 8u))
        Flag_Left = 1;
    else if (byte == 'X') {
        Target_Velocity = Target_Velocity >= TEACH_MAX_SPEED - 100u ?
                          TEACH_MAX_SPEED : Target_Velocity + 100u;
        Flag_velocity = 1;
    } else if (byte == 'Y') {
        Target_Velocity = Target_Velocity < 100u ? 0u : Target_Velocity - 100u;
        Flag_velocity = 2;
    }
    /* Z/0 stop; X/Y also stop, as in the original Bluetooth handler. */
}

static void start_record(void)
{
    change_mode(Teach_Record_Mode);
    Teach_EventCount = 0; /* A new R overwrites the only saved route. */
    Teach_DurationTicks = 0;
    elapsed = 0;
    initial_speed = Target_Velocity;
    initial_gear = Flag_velocity;
    Teach_Status = TEACH_RECORDING;
}

static void start_play(void)
{
    change_mode(Teach_Play_Mode); /* Also finalizes an ongoing recording. */
    if (Teach_EventCount == 0 || Teach_DurationTicks == 0) {
        change_mode(Normal_Mode);
        Teach_Status = TEACH_EMPTY;
        return;
    }
    Target_Velocity = initial_speed;
    Flag_velocity = initial_gear;
    elapsed = 0;
    play_index = 0;
    Teach_Status = TEACH_PLAYING;
}

static uint8_t record_action(uint8_t byte)
{
    /* Repeated direction packets do not change the action or its duration. */
    if (Teach_EventCount && byte != 'X' && byte != 'Y' &&
        route[Teach_EventCount - 1u].command == byte) return 1;
    if (Teach_EventCount >= TEACH_MAX_EVENTS) {
        change_mode(Normal_Mode);
        Teach_Status = TEACH_BUFFER_FULL;
        reset_input();
        return 0;
    }
    route[Teach_EventCount].tick = elapsed;
    route[Teach_EventCount].command = byte;
    ++Teach_EventCount;
    return 1;
}

static void apply_frame(void)
{
    uint8_t i;
    float data = 0;
    if (frame_invalid || frame_length < 4u) return;
    if (frame[3] == 'P') { PID_Send = 1; return; }
    /* Parameter edits are not recorded: freeze them during teach/play. */
    if (TeachRemote_IsActive() || frame[1] == '#') return;
    for (i = 3; i < frame_length; ++i) {
        if (frame[i] < '0' || frame[i] > '9') return;
        data = data * 10 + (frame[i] - '0');
        if (data > 1000000.0f) return;
    }
    switch (frame[1]) {
    case '0': Target_Velocity = (uint16_t)(data > TEACH_MAX_SPEED ? TEACH_MAX_SPEED : data); break;
    case '1': Balance_Kp = data; break;
    case '2': Balance_Kd = data; break;
    case '3': Velocity_Kp = data; break;
    case '4': Velocity_Ki = data; break;
    case '5': Turn_Kp = data; break;
    case '6': Turn_Kd = data; break;
    case '7': Distance_KP = data; break;
    case '8': Distance_KD = data; break;
    default: break;
    }
}

static void handle_byte(uint8_t byte, uint8_t safe)
{
    if (in_frame) {
        frame_age = 0;
        if (byte == '}') { apply_frame(); in_frame = 0; }
        else if (frame_length < FRAME_SIZE) frame[frame_length++] = byte;
        else frame_invalid = 1;
        return; /* R/P/Q/T within {...} are never mode commands. */
    }
    if (byte == '{') {
        in_frame = 1;
        frame_invalid = 0;
        frame_length = 1;
        frame[0] = byte;
        frame_age = 0;
        return;
    }
    switch (byte) {
    case 'Q':
        change_mode(Normal_Mode);
        if (Teach_Status == TEACH_PLAYING) Teach_Status = TEACH_SAVED;
        reset_input(); /* Discard stale bytes queued after the stop. */
        return;
    case 'T':
        change_mode(IRDM_Line_Patrol_Mode);
        if (Teach_Status == TEACH_PLAYING) Teach_Status = TEACH_SAVED;
        return;
    case 'R': if (safe) start_record(); return;
    case 'P': if (safe) start_play(); return;
    default: break;
    }
    if (!safe || Mode == Teach_Play_Mode || !is_action(byte)) return;
    if (Mode == Teach_Record_Mode && !record_action(byte)) return;
    apply_action(byte);
}

void TeachRemote_Tick(uint8_t safe)
{
    uint16_t budget = RX_SIZE;
    uint8_t byte;
    if (!safe) TeachRemote_SafetyStop();
    if (rx_overflow) {
        change_mode(Normal_Mode);
        Teach_Status = TEACH_RX_OVERFLOW;
        reset_input();
        return;
    }
    if (in_frame && ++frame_age >= FRAME_TIMEOUT_TICKS) in_frame = 0;
    /* Bounded work even if UART preempts this loop continuously. */
    while (rx_tail != rx_head && budget--) {
        byte = rx[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RX_SIZE);
        handle_byte(byte, safe);
    }
    if (Mode == Teach_Record_Mode) {
        if (elapsed >= TEACH_MAX_TICKS) {
            change_mode(Normal_Mode);
            Teach_Status = TEACH_TIME_LIMIT;
            reset_input();
        } else {
            ++elapsed;
            Teach_DurationTicks = elapsed;
        }
    } else if (Mode == Teach_Play_Mode) {
        if (elapsed >= Teach_DurationTicks) {
            change_mode(Normal_Mode);
            Teach_Status = TEACH_FINISHED;
        } else {
            while (play_index < Teach_EventCount && route[play_index].tick <= elapsed) {
                apply_action(route[play_index].command);
                ++play_index;
            }
            ++elapsed;
        }
    }
}
/* by codex */
