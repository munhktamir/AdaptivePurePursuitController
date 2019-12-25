#include <check.h>
#include <stdio.h>
#include "../motion/Motion.h"


START_TEST(test_IsSegmentValid) {
    motionSegment_t segment;
    int valid;

    segment.start.t = 2.0;
    segment.start.pos = 2.5;
    segment.start.vel = 9.0;
    segment.start.acc = 0.0;

    segment.end.t = 2.5;
    segment.end.pos = 7.0;
    segment.end.vel = 9.0;
    segment.end.acc = 0.0;

    // Acceleration is not constant
    segment.start.acc = 1.0;
    valid = IsSegmentValid(&segment);
    ck_assert_int_eq(0, valid);

    // Velocity direction changes
    segment.start.acc = 0.0;
    segment.start.vel = -9.0;
    valid = IsSegmentValid(&segment);
    ck_assert_int_eq(0, valid);

    // Inconsistent
    segment.start.vel = 2.0;
    valid = IsSegmentValid(&segment);
    ck_assert_int_eq(0, valid);

    // Consistent
    segment.start.vel = 9.0;
    valid = IsSegmentValid(&segment);
    ck_assert_int_eq(1, valid);


} END_TEST


START_TEST(test_ContainsTime) {
    motionSegment_t segment;
    int valid;

    segment.start.t = 2.0;
    segment.start.pos = 2.5;
    segment.start.vel = 9.0;
    segment.start.acc = 0.0;
    segment.end.t = 2.5;
    segment.end.pos = 7.0;
    segment.end.vel = 9.0;
    segment.end.acc = 0.0;
    valid = ContainsTime(&segment, 2.1);
    ck_assert_int_eq(1, valid);
    valid = ContainsTime(&segment, 2.6);
    ck_assert_int_eq(0, valid);

} END_TEST


START_TEST(test_ContainsPosition) {
    motionSegment_t segment;
    int valid;

    segment.start.t = 2.0;
    segment.start.pos = 2.5;
    segment.start.vel = 9.0;
    segment.start.acc = 0.0;
    segment.end.t = 2.5;
    segment.end.pos = 7.0;
    segment.end.vel = 9.0;
    segment.end.acc = 0.0;
    valid = ContainsPosition(&segment, 3.4);
    ck_assert_int_eq(1, valid);
    valid = ContainsTime(&segment, 7.2);
    ck_assert_int_eq(0, valid);

} END_TEST


Suite *motionSegment_suite(void) {
    Suite *s;
    TCase *tc;

    s = suite_create("MotionSegment");
    tc = tcase_create("Core");

    tcase_add_test(tc, test_IsSegmentValid);
    tcase_add_test(tc, test_ContainsTime);
    tcase_add_test(tc, test_ContainsPosition);
    suite_add_tcase(s, tc);
    return s;
}

