#include "utils/CustomTime.hpp"
#include "utils/Units.hpp"
#include <gtest/gtest.h>
#include <stdlib.h>

TEST(TimeTest, GetLocalReadableTime) {
  Time m_time = Time("2024-03-21 10:30:00", -9.5);
  EXPECT_EQ(m_time.get_local_readable_time(), "2024-03-21 10:30:00.000");
}

TEST(TimeTest, UpdateSecondsUTCReadable) {
  // Create Time t for testing
  // -9.5 to convert from time in Alice Springs (which is 9.5 hours ahead of
  // UTC)
  Time t = Time("2024-03-21 10:30:00", -9.5);

  // Test update function
  t.update_time_seconds(14);

  // Test both the local time and the UTC time
  EXPECT_EQ(t.get_local_readable_time(), "2024-03-21 10:30:14.000");
  EXPECT_EQ(t.get_utc_readable_time(), "2024-03-21 01:00:14.000");

  t.update_time_seconds(60);
  EXPECT_EQ(t.get_local_readable_time(), "2024-03-21 10:31:14.000");
  EXPECT_EQ(t.get_utc_readable_time(), "2024-03-21 01:01:14.000");

  t.update_time_seconds(5400);
  EXPECT_EQ(t.get_local_readable_time(), "2024-03-21 12:01:14.000");
  EXPECT_EQ(t.get_utc_readable_time(), "2024-03-21 02:31:14.000");

  t.update_time_seconds(43200);
  // Test day update
  EXPECT_EQ(t.get_local_readable_time(), "2024-03-22 00:01:14.000");
  EXPECT_EQ(t.get_utc_readable_time(), "2024-03-21 14:31:14.000");

  Time time = Time("2023-12-31 23:30:00", -9.5);
  EXPECT_EQ(time.get_local_readable_time(), "2023-12-31 23:30:00.000");
  EXPECT_EQ(time.get_utc_readable_time(), "2023-12-31 14:00:00.000");

  time.update_time_seconds(5400);
  // Test month and year change
  EXPECT_EQ(time.get_local_readable_time(), "2024-01-01 01:00:00.000");
  EXPECT_EQ(time.get_utc_readable_time(), "2023-12-31 15:30:00.000");
}

TEST(TimeTest, GetUTCtmAndTimePoint) {
  Time t = Time("2024-03-21 10:30:14", -9.5);

  // Verify all parts of tm
  EXPECT_EQ(t.get_utc_tm().tm_year, 124);
  EXPECT_EQ(t.get_utc_tm().tm_mon, 2);
  EXPECT_EQ(t.get_utc_tm().tm_mday, 21);
  EXPECT_EQ(t.get_utc_tm().tm_hour, 1);
  EXPECT_EQ(t.get_utc_tm().tm_min, 0);
  EXPECT_EQ(t.get_utc_tm().tm_sec, 14);
  EXPECT_EQ(t.get_utc_tm().tm_wday, 4);
  EXPECT_EQ(t.get_utc_tm().tm_yday, 80);
  EXPECT_EQ(t.get_utc_tm().tm_isdst, 0);

  // Test the total number of seconds since  Jan 1, 1970 which is the timepoint
  // value
  EXPECT_EQ(t.get_utc_time_point(), 1710982814);

  Time time = Time("2023-12-31 23:30:00", -9.5);
  EXPECT_EQ(time.get_utc_tm().tm_year, 123);
  EXPECT_EQ(time.get_utc_tm().tm_mon, 11);
  EXPECT_EQ(time.get_utc_tm().tm_mday, 31);
  EXPECT_EQ(time.get_utc_tm().tm_hour, 14);
  EXPECT_EQ(time.get_utc_tm().tm_min, 0);
  EXPECT_EQ(time.get_utc_tm().tm_sec, 0);
  EXPECT_EQ(time.get_utc_tm().tm_wday, 0);
  EXPECT_EQ(time.get_utc_tm().tm_yday, 364);
  EXPECT_EQ(time.get_utc_tm().tm_isdst, 0);

  EXPECT_EQ(time.get_utc_time_point(), 1704031200);
}

TEST(TimeTest, CopyHHMMSS) {
  Time t("2025-03-25 16:30:00", -9.5);
  Time hh_mm_ss_only("17:00:00");

  EXPECT_EQ(t.get_utc_readable_time(), "2025-03-25 07:00:00.000");
  EXPECT_EQ(t.get_local_readable_time(), "2025-03-25 16:30:00.000");

  t.copy_hh_mm_ss(hh_mm_ss_only);
  EXPECT_EQ(t.get_local_readable_time(), "2025-03-25 17:00:00.000");
  EXPECT_EQ(t.get_utc_readable_time(), "2025-03-25 07:30:00.000");

  t.update_time_seconds(5.5);
  EXPECT_EQ(t.get_local_readable_time(), "2025-03-25 17:00:05.500");
}

TEST(TimeTest, Operator) {
  Time t_finish("2025-08-23 00:00:00", -9.5);
  Time t_start("2025-08-22 23:30:00", -9.5);

  EXPECT_EQ(t_finish.get_utc_readable_time(), "2025-08-22 14:30:00.000");

  double result = t_finish - t_start;
  EXPECT_EQ(result, 1800.0);

  t_finish.update_time_seconds(4.32);
  result = t_finish - t_start;
  EXPECT_EQ(result, 1804.32);

  EXPECT_TRUE(t_finish > t_start);

  Time t_same(t_finish);
  EXPECT_TRUE(t_finish <= t_same);

  t_start.update_time_seconds(t_finish - t_start);
  EXPECT_EQ(t_finish - t_start, 0.0);
}
