#include <check.h>
#include <stdio.h>
#include <stdlib.h>
#include "../motion/Motion.h"


START_TEST(test_ClearProfile) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;

    ClearProfile(&profile);

    ck_assert_ptr_null(profile.head);
    ck_assert_ptr_null(profile.tail);
    ck_assert_int_eq(0, profile.length);


} END_TEST


START_TEST(test_ResetProfile) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;
    motionState_t state;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    state.t = 1.5;
    state.pos = 4.2;
    state.vel = 2.0;
    state.acc = 10.0;

    ResetProfile(&profile, &state);

    ck_assert_ptr_nonnull(profile.head);
    ck_assert_ptr_nonnull(profile.tail);
    ck_assert_ptr_eq(profile.head, profile.tail);
    ck_assert_int_eq(1, profile.length);
    ck_assert_double_eq(1.5, state.t);
    ck_assert_double_eq(4.2, state.pos);
    ck_assert_double_eq(2.0, state.vel);
    ck_assert_double_eq(10.0, state.acc);

} END_TEST


START_TEST(test_AppendSegment) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;
    motionSegment_t segment;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    segment.start.t = 0.0;
    segment.start.pos = 0.0;
    segment.start.vel = 5.0;
    segment.start.acc = 5.0;
    segment.end.t = 1.0;
    segment.end.pos = 10.0;
    segment.end.vel = 5.0;
    segment.end.acc = 5.0;

    AppendSegment(&profile, &segment);
    ck_assert_ptr_nonnull(profile.head);
    ck_assert_ptr_nonnull(profile.tail);
    ck_assert_int_eq(4, profile.length);
    ck_assert_double_eq(0.0, profile.tail->segment.start.t);
    ck_assert_double_eq(0.0, profile.tail->segment.start.pos);
    ck_assert_double_eq(5.0, profile.tail->segment.start.vel);
    ck_assert_double_eq(5.0, profile.tail->segment.start.acc);
    ck_assert_double_eq(1.0, profile.tail->segment.end.t);
    ck_assert_double_eq(10.0, profile.tail->segment.end.pos);
    ck_assert_double_eq(5.0, profile.tail->segment.end.vel);
    ck_assert_double_eq(5.0, profile.tail->segment.end.acc);


} END_TEST


START_TEST(test_AppendControl) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    tail->segment.start.t = 0.0;
    tail->segment.start.pos = 0.0;
    tail->segment.start.vel = 5.0;
    tail->segment.start.acc = 5.0;
    tail->segment.end.t = 0.5;
    tail->segment.end.pos = 2.5;
    tail->segment.end.vel = 5.0;
    tail->segment.end.acc = 5.0;

    AppendControl (&profile, 2.5, 1.5);
    ck_assert_ptr_nonnull(profile.head);
    ck_assert_ptr_nonnull(profile.tail);
    ck_assert_int_eq(4, profile.length);
    ck_assert_double_eq(tail->segment.end.t, profile.tail->segment.start.t);
    ck_assert_double_eq(tail->segment.end.pos, profile.tail->segment.start.pos);
    ck_assert_double_eq(tail->segment.end.vel, profile.tail->segment.start.vel);
    ck_assert_double_eq(2.5, profile.tail->segment.start.acc);
    ck_assert_double_eq(tail->segment.end.t + 1.5, profile.tail->segment.end.t);
    ck_assert_double_eq(12.8125, profile.tail->segment.end.pos);
    ck_assert_double_eq(8.75, profile.tail->segment.end.vel);
    ck_assert_double_eq(2.5, profile.tail->segment.end.acc);

} END_TEST


START_TEST(test_Consolidate) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    head->segment.start.t = 0.0;
    head->segment.start.pos = 0.0;
    head->segment.start.vel = 5.0;
    head->segment.start.acc = 0.0;
    head->segment.end.t = 0.5;
    head->segment.end.pos = 2.5;
    head->segment.end.vel = 5.0;
    head->segment.end.acc = 0.0;
    middle->segment.start.t = 0.5;
    middle->segment.start.pos = 2.5;
    middle->segment.start.vel = 5.0;
    middle->segment.start.acc = 0.0;
    middle->segment.end.t = 0.5;
    middle->segment.end.pos = 2.5;
    middle->segment.end.vel = 5.0;
    middle->segment.end.acc = 0.0;
    tail->segment.start.t = 1.0;
    tail->segment.start.pos = 5.0;
    tail->segment.start.vel = 5.0;
    tail->segment.start.acc = 0.0;
    tail->segment.end.t = 1.5;
    tail->segment.end.pos = 7.5;
    tail->segment.end.vel = 5.0;
    tail->segment.end.acc = 0.0;

    Consolidate(&profile);
    ck_assert_ptr_nonnull(profile.head);
    ck_assert_ptr_nonnull(profile.tail);
    ck_assert_int_eq(2, profile.length);

} END_TEST


START_TEST(test_AppendProfile) {
    motionProfileList_t currentProfile, addProfile;
    motionProfileNode_t *head, *middle, *tail, *addHead, *addMiddle, *addTail;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    currentProfile.head = head;
    currentProfile.tail = tail;
    currentProfile.length = 3;
    addHead = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    addMiddle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    addTail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    addHead->next = addMiddle;
    addMiddle->next = addTail;
    addTail->next = NULL;
    addProfile.head = addHead;
    addProfile.tail = addTail;
    addProfile.length = 3;

    AppendProfile(&currentProfile, &addProfile);
    ck_assert_int_eq(6, currentProfile.length);
    ck_assert_ptr_eq(currentProfile.head, head);
    ck_assert_ptr_eq(currentProfile.head->next, middle);
    ck_assert_ptr_eq(currentProfile.head->next->next, tail);

} END_TEST


START_TEST(test_TrimBeforeTime) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    head->segment.start.t = 0.0;
    head->segment.start.pos = 0.0;
    head->segment.start.vel = 5.0;
    head->segment.start.acc = 0.0;
    head->segment.end.t = 0.5;
    head->segment.end.pos = 2.5;
    head->segment.end.vel = 5.0;
    head->segment.end.acc = 0.0;
    middle->segment.start.t = 0.5;
    middle->segment.start.pos = 2.5;
    middle->segment.start.vel = 5.0;
    middle->segment.start.acc = 0.0;
    middle->segment.end.t = 1.0;
    middle->segment.end.pos = 5.0;
    middle->segment.end.vel = 5.0;
    middle->segment.end.acc = 0.0;
    tail->segment.start.t = 1.0;
    tail->segment.start.pos = 5.0;
    tail->segment.start.vel = 5.0;
    tail->segment.start.acc = 0.0;
    tail->segment.end.t = 1.5;
    tail->segment.end.pos = 7.5;
    tail->segment.end.vel = 5.0;
    tail->segment.end.acc = 0.0;

    // Shorten the segment
    TrimBeforeTime(&profile, 0.25);
    ck_assert_int_eq(3, profile.length);
    ck_assert_double_eq(0.25, profile.head->segment.start.t);
    ck_assert_double_eq(1.25, profile.head->segment.start.pos);
    ck_assert_double_eq(5.0, profile.head->segment.start.vel);
    ck_assert_double_eq(0.0, profile.head->segment.start.acc);

    // Remove the segment
    TrimBeforeTime(&profile, 0.5);
    ck_assert_int_eq(2, profile.length);
    ck_assert_ptr_eq(profile.head, middle);


} END_TEST


START_TEST(test_IsProfileValid) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;
    int valid;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    head->segment.start.t = 0.0;
    head->segment.start.pos = 0.0;
    head->segment.start.vel = 5.0;
    head->segment.start.acc = 0.0;
    head->segment.end.t = 0.5;
    head->segment.end.pos = 2.5;
    head->segment.end.vel = 5.0;
    head->segment.end.acc = 0.0;
    middle->segment.start.t = 0.5;
    middle->segment.start.pos = 2.5;
    middle->segment.start.vel = 5.0;
    middle->segment.start.acc = 0.0;
    middle->segment.end.t = 1.0;
    middle->segment.end.pos = 5.0;
    middle->segment.end.vel = 5.0;
    middle->segment.end.acc = 0.0;
    tail->segment.start.t = 1.0;
    tail->segment.start.pos = 5.0;
    tail->segment.start.vel = 5.0;
    tail->segment.start.acc = 0.0;
    tail->segment.end.t = 1.5;
    tail->segment.end.pos = 7.5;
    tail->segment.end.vel = 5.0;
    tail->segment.end.acc = 0.0;

    // Valid
    valid = IsProfileValid(&profile);
    ck_assert_int_eq(1, valid);

    // Invalid segment
    head->segment.end.acc = 1.0;
    valid = IsProfileValid(&profile);
    ck_assert_int_eq(0, valid);

    // Not continuous
    head->segment.end.acc = 0.0;
    head->segment.end.vel = 3.0;
    middle->segment.start.vel = 4.0;
    valid = IsProfileValid(&profile);
    ck_assert_int_eq(0, valid);


} END_TEST


START_TEST(test_StateByTime) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;
    motionState_t state;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    head->segment.start.t = 0.0;
    head->segment.start.pos = 0.0;
    head->segment.start.vel = 5.0;
    head->segment.start.acc = 0.0;
    head->segment.end.t = 0.5;
    head->segment.end.pos = 2.5;
    head->segment.end.vel = 5.0;
    head->segment.end.acc = 0.0;
    middle->segment.start.t = 0.5;
    middle->segment.start.pos = 2.5;
    middle->segment.start.vel = 5.0;
    middle->segment.start.acc = 0.0;
    middle->segment.end.t = 1.0;
    middle->segment.end.pos = 5.0;
    middle->segment.end.vel = 5.0;
    middle->segment.end.acc = 0.0;
    tail->segment.start.t = 1.0;
    tail->segment.start.pos = 5.0;
    tail->segment.start.vel = 5.0;
    tail->segment.start.acc = 0.0;
    tail->segment.end.t = 1.5;
    tail->segment.end.pos = 7.5;
    tail->segment.end.vel = 5.0;
    tail->segment.end.acc = 0.0;

    // Head start
    state = StateByTime(&profile, -0.0000001);
    ck_assert_double_eq(0.0, state.t);
    ck_assert_double_eq(0.0, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);

    // Tail end
    state = StateByTime(&profile, 1.5000001);
    ck_assert_double_eq(1.5, state.t);
    ck_assert_double_eq(7.5, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);

    // Extrapolate the start
    state = StateByTime(&profile, 0.25);
    ck_assert_double_eq(0.25, state.t);
    ck_assert_double_eq(1.25, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);

    // Outside of range
    state = StateByTime(&profile, 2.0);
    ck_assert_double_nan(state.t);
    ck_assert_double_nan(state.pos);
    ck_assert_double_nan(state.vel);
    ck_assert_double_nan(state.acc);


} END_TEST


START_TEST(test_StateByTimeClamped) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;
    motionState_t state;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    head->segment.start.t = 0.0;
    head->segment.start.pos = 0.0;
    head->segment.start.vel = 5.0;
    head->segment.start.acc = 0.0;
    head->segment.end.t = 0.5;
    head->segment.end.pos = 2.5;
    head->segment.end.vel = 5.0;
    head->segment.end.acc = 0.0;
    middle->segment.start.t = 0.5;
    middle->segment.start.pos = 2.5;
    middle->segment.start.vel = 5.0;
    middle->segment.start.acc = 0.0;
    middle->segment.end.t = 1.0;
    middle->segment.end.pos = 5.0;
    middle->segment.end.vel = 5.0;
    middle->segment.end.acc = 0.0;
    tail->segment.start.t = 1.0;
    tail->segment.start.pos = 5.0;
    tail->segment.start.vel = 5.0;
    tail->segment.start.acc = 0.0;
    tail->segment.end.t = 1.5;
    tail->segment.end.pos = 7.5;
    tail->segment.end.vel = 5.0;
    tail->segment.end.acc = 0.0;

    // Head start
    state = StateByTimeClamped(&profile, -0.1);
    ck_assert_double_eq(0.0, state.t);
    ck_assert_double_eq(0.0, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);

    // Tail end
    state = StateByTimeClamped(&profile, 1.6);
    ck_assert_double_eq(1.5, state.t);
    ck_assert_double_eq(7.5, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);

    // Extrapolate the start
    state = StateByTimeClamped(&profile, 0.25);
    ck_assert_double_eq(0.25, state.t);
    ck_assert_double_eq(1.25, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);

} END_TEST


START_TEST(test_FirstStateByPosition) {
    motionProfileList_t profile;
    motionProfileNode_t *head, *middle, *tail;
    motionState_t state;

    head = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    middle = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    tail = (motionProfileNode_t *) malloc( sizeof( motionProfileNode_t ) );
    head->next = middle;
    middle->next = tail;
    tail->next = NULL;
    profile.head = head;
    profile.tail = tail;
    profile.length = 3;
    head->segment.start.t = 0.0;
    head->segment.start.pos = 0.0;
    head->segment.start.vel = 5.0;
    head->segment.start.acc = 0.0;
    head->segment.end.t = 0.5;
    head->segment.end.pos = 2.5;
    head->segment.end.vel = 5.0;
    head->segment.end.acc = 0.0;
    middle->segment.start.t = 0.5;
    middle->segment.start.pos = 2.5;
    middle->segment.start.vel = 5.0;
    middle->segment.start.acc = 0.0;
    middle->segment.end.t = 1.0;
    middle->segment.end.pos = 5.0;
    middle->segment.end.vel = 5.0;
    middle->segment.end.acc = 0.0;
    tail->segment.start.t = 1.0;
    tail->segment.start.pos = 5.0;
    tail->segment.start.vel = 5.0;
    tail->segment.start.acc = 0.0;
    tail->segment.end.t = 1.5;
    tail->segment.end.pos = 7.5;
    tail->segment.end.vel = 5.0;
    tail->segment.end.acc = 0.0;

    // Middle end
    state = FirstStateByPosition(&profile, 5.0);
    ck_assert_double_eq(1.0, state.t);
    ck_assert_double_eq(5.0, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);

    // Middle extrapolate
    state = FirstStateByPosition(&profile, 3.75);
    ck_assert_double_eq(0.75, state.t);
    ck_assert_double_eq(3.75, state.pos);
    ck_assert_double_eq(5.0, state.vel);
    ck_assert_double_eq(0.0, state.acc);


} END_TEST


Suite *motionProfile_suite(void) {
    Suite *s;
    TCase *tc;

    s = suite_create("MotionProfile");
    tc = tcase_create("Core");

    tcase_add_test(tc, test_ClearProfile);
    tcase_add_test(tc, test_ResetProfile);
    tcase_add_test(tc, test_AppendSegment);
    tcase_add_test(tc, test_AppendControl);
    tcase_add_test(tc, test_Consolidate);
    tcase_add_test(tc, test_AppendProfile);
    tcase_add_test(tc, test_TrimBeforeTime);
    tcase_add_test(tc, test_IsProfileValid);
    tcase_add_test(tc, test_StateByTime);
    tcase_add_test(tc, test_StateByTimeClamped);
    tcase_add_test(tc, test_FirstStateByPosition);
    suite_add_tcase(s, tc);
    return s;
}

