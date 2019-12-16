#include "ChRt.h"
#include "FlexCAN.h"

#define LOOP_TIME_MS 5
#define CAN_DEVICE_TYPE 10  // 5-bit 24-28
#define CAN_MANUFACTURER 8  // 8-bit 16-23
#define CAN_API_CLASS 0     // 6-bit 10-15
#define CAN_API_INDEX 0     // 4-bit 6-9
#define CAN_DEVICE_NUMBER 0 // 6-bit 0-5

static CAN_message_t msg;
static uint32_t arb_id = CAN_DEVICE_NUMBER + CAN_API_INDEX << 6 + CAN_API_CLASS << 10 + CAN_MANUFACTURER << 16 + CAN_DEVICE_TYPE << 24;

class CANClass : public CANListener {
    public:
        void printFrame(CAN_message_t &frame, int mailbox);
        void frameHandler (CAN_message_t &frame, int mailbox, uint8_t controller);
};

void CANClass::printFrame(CAN_message_t &frame, int mailbox) {
    Serial.print(mailbox);
    Serial.print(" ID: ");
    Serial.print(frame.id, HEX);
    Serial.print(" Data: ");
    for (int c = 0; c < frame.len; c++) {
        Serial.print(frame.buf[c], HEX);
        Serial.write(' ');
    }
    Serial.println();
}

bool CANClass::frameHandler (CAN_message_t &frame, int mailbox, uint8_t controller) {
    printFrame(frame, mailbox);
    return true;
}

//CANClass CANClass0;
CANClass CANClass1;


//------------------------------------------------------------------------------
static THD_WORKING_AREA(chThd1Wa, 200);

static THD_FUNCTION(chThd1Fcn, arg) {
    systime_t time = chVTGetSystemTimeX(); // T0
    uint32_t tlast;
  
    while (true) {
        time += TIME_MS2I(LOOP_TIME_MS);
        tlast = micros();
        Serial.println(tlast);
        chThdSleepUntil(time);
    }
}

void chStartup() {
    chThdCreateStatic(chThd1Wa, sizeof(chThd1Wa), NORMALPRIO, chThd1Fcn, NULL);
}

//------------------------------------------------------------------------------
void setup() {
    Serial.begin(1000000);
    while (!Serial) {}
    //Can0.begin(1000000);  
    //Can0.attachObj(&CANClass0);
    Can1.begin(1000000);  
    Can1.attachObj(&CANClass1);

    CAN_filter_t allPassFilter;
    allPassFilter.id=0;
    allPassFilter.extended=1;
    allPassFilter.remote=0;

    for (uint8_t filterNum = 0; filterNum < 16; filterNum++) {
        //Can0.setFilter(allPassFilter,filterNum);
        Can1.setFilter(allPassFilter,filterNum);
    }
  
    //CANClass0.attachGeneralHandler();
    CANClass1.attachGeneralHandler();

    msg.id = arb_id;
    msg.extended = 1;
    msg.remote = 0;
    msg.len = 8;
    msg.buf[0] = 10;
    msg.buf[1] = 20;
    msg.buf[2] = 0;
    msg.buf[3] = 100;
    msg.buf[4] = 128;
    msg.buf[5] = 64;
    msg.buf[6] = 32;
    msg.buf[7] = 16;
  
    chBegin(chStartup);     // Start kernel - loop() becomes main thread
    while (true) {}         // chBegin() resets stacks and should never return
}

//------------------------------------------------------------------------------
void loop() {
    // Not used.
}
