#include <AP_gtest.h>
#include <AP_Mission/AP_Mission_TakeoffUTC.h>

using Scheduler = AP_Mission_TakeoffUTC;
using State = Scheduler::State;
static constexpr uint64_t UTC = 1794150000000000ULL;
static constexpr uint64_t NOW = 10000000;

TEST(TakeoffUTC, NoEarlyLaunchAndNoAltitudeDeadline)
{
    Scheduler s;
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 5000000, 10));
    EXPECT_EQ(s.update(UTC + 4999999, NOW + 4999999, true, true, false), State::Waiting);
    EXPECT_EQ(s.update(UTC + 5000000, NOW + 5000000, true, true, false), State::Launch);
    EXPECT_EQ(s.update(UTC + 14999999, NOW + 14999999, true, true, false), State::Climbing);
    EXPECT_EQ(s.update(UTC + 15000000, NOW + 15000000, true, true, true), State::Airborne);
    EXPECT_EQ(s.update(0, NOW + 90000000, false, true, false), State::Airborne);
}

TEST(TakeoffUTC, GroundTimeoutAndNoRetry)
{
    for (const uint16_t timeout : {
             1, 10, 60
         }) {
        Scheduler s;
        ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, timeout));
        EXPECT_EQ(s.update(UTC + 1000000, NOW + 1000000, true, true, false), State::Launch);
        EXPECT_EQ(s.update(0, NOW + 1000000 + uint64_t(timeout) * 1000000 - 1, true, true, false), State::Climbing);
        EXPECT_EQ(s.update(0, NOW + 1000000 + uint64_t(timeout) * 1000000, true, true, false), State::Aborted);
        EXPECT_EQ(s.update(UTC + 2000000, NOW + 2000000, true, true, false), State::Aborted);
    }
}

TEST(TakeoffUTC, InvalidExpiredAndClockDiscontinuity)
{
    Scheduler s;
    EXPECT_FALSE(s.prepare(0, NOW, UTC, 10));
    EXPECT_FALSE(s.prepare(UTC, NOW, UTC, 10));
    EXPECT_FALSE(s.prepare(UTC, NOW, UTC - 1, 10));
    EXPECT_FALSE(s.prepare(UTC, NOW, UTC + 86400000001ULL, 10));
    EXPECT_FALSE(s.prepare(UTC, NOW, UTC + 1000000, 0));
    EXPECT_FALSE(s.prepare(UTC, NOW, UTC + 1000000, 61));
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, 10));
    EXPECT_EQ(s.update(UTC + 500000, NOW, true, true, false), State::Aborted);
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, 10));
    EXPECT_EQ(s.update(0, NOW + 10000, true, true, false), State::Aborted);
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, 10));
    EXPECT_EQ(s.update(UTC, NOW + 500000, true, true, false), State::Aborted);
}

TEST(TakeoffUTC, MissedReadinessNeverStartsLater)
{
    Scheduler s;
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, 10));
    EXPECT_EQ(s.update(UTC + 1000000, NOW + 1000000, false, true, false), State::Aborted);
    EXPECT_EQ(s.update(UTC + 1010000, NOW + 1010000, true, true, false), State::Aborted);
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, 10));
    EXPECT_EQ(s.update(UTC + 1100001, NOW + 1100001, true, true, false), State::Aborted);
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, 10));
    EXPECT_EQ(s.update(UTC + 1000000, NOW + 1000000, true, false, false), State::Aborted);
}

TEST(TakeoffUTC, DisarmDuringLaunchDoesNotRestart)
{
    Scheduler s;
    ASSERT_TRUE(s.prepare(UTC, NOW, UTC + 1000000, 10));
    EXPECT_EQ(s.update(UTC + 1000000, NOW + 1000000, true, true, false), State::Launch);
    EXPECT_EQ(s.update(UTC + 1010000, NOW + 1010000, true, false, false), State::Aborted);
    EXPECT_EQ(s.update(UTC + 1020000, NOW + 1020000, true, true, false), State::Aborted);
}

TEST(TakeoffUTC, MidnightAndLongUptime)
{
    Scheduler s;
    const uint64_t midnight = 20765ULL * 86400000000ULL;
    const uint64_t uptime = 5000000000000ULL;
    ASSERT_TRUE(s.prepare(midnight - 1000000, uptime, midnight, 10));
    EXPECT_EQ(s.update(midnight, uptime + 1000000, true, true, false), State::Launch);
}

AP_GTEST_MAIN()
