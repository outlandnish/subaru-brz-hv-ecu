#include <unity.h>
#include "params.h"
#include "ivt-s/ivt_shunt.h"

void setUp() {}
void tearDown() {}

// Test 1: safety-critical params are within safe ranges after LoadDefaults()
void test_default_params_safe() {
    Param::LoadDefaults();

    float target_mv = Param::GetFloat(Param::targetCellVolt);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(4200.0f, target_mv);

    float max_current_a = Param::GetFloat(Param::maxChargeCurrent);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(200.0f, max_current_a);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, max_current_a);
}

// Test 2: IVT 0x521 frame encode/decode round-trip
// muxid=0x00, status=0x00, value=1000 mA → get_current() == 1.0 A
void test_ivt_current_decode() {
    IVTShunt ivt;

    CAN_FRAME frame;
    frame.id = 0x521;
    frame.length = 8;
    frame.data.uint8[0] = 0x00;   // muxid = current
    frame.data.uint8[1] = 0x00;   // status/counter = 0
    // 1000 mA = 0x000003E8 big-endian
    frame.data.uint8[2] = 0x00;
    frame.data.uint8[3] = 0x00;
    frame.data.uint8[4] = 0x03;
    frame.data.uint8[5] = 0xE8;
    frame.data.uint8[6] = 0x00;
    frame.data.uint8[7] = 0x00;

    ivt.process_can_frame(&frame);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, ivt.get_current());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_default_params_safe);
    RUN_TEST(test_ivt_current_decode);
    return UNITY_END();
}
