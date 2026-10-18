// The Runtime-LOG shared-memory format copy must produce the frozen v1 sample byte for byte.
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "rt_control_semantic_components/runtime_log_shm_ring.hpp"

namespace
{
std::vector<unsigned char> sample_bytes()
{
  const std::string name = "/runtime-log.formatcheck-" + std::to_string(getpid());
  runtime_log::ShmRingWriter writer;
  std::string error;
  EXPECT_TRUE(writer.create(name, "format", "check", {"a", "b/c"}, 4, &error)) << error;
  writer.set_state(runtime_log::WriterState::kActive);
  double * values = writer.begin();
  values[0] = 1.5;
  values[1] = -2.0;
  writer.commit(1000, 1000000);
  values = writer.begin();
  values[0] = 3.0;
  values[1] = 4.0;
  writer.commit(2000, 1000000);
  const size_t size = writer.size_bytes();
  writer.close();
  std::vector<unsigned char> bytes(size);
  const int fd = shm_open(name.c_str(), O_RDONLY, 0);
  EXPECT_GE(fd, 0);
  EXPECT_EQ(pread(fd, bytes.data(), size, 0), static_cast<ssize_t>(size));
  close(fd);
  shm_unlink(name.c_str());
  std::memset(bytes.data() + 32, 0, 24);  // writer pid and creation times
  return bytes;
}
}  // namespace

TEST(RuntimeLogShmRing, MatchesTheFrozenFormatSample)
{
  std::ifstream golden(RUNTIME_LOG_GOLDEN, std::ios::binary);
  ASSERT_TRUE(golden.good()) << RUNTIME_LOG_GOLDEN;
  const std::vector<unsigned char> expected(
    (std::istreambuf_iterator<char>(golden)), std::istreambuf_iterator<char>());
  EXPECT_EQ(sample_bytes(), expected);
}

TEST(RuntimeLogShmRing, WriterIsRealtimeShaped)
{
  // begin()/commit() only touch the mapped, pre-faulted, locked segment.
  const std::string name = "/runtime-log.rtcheck-" + std::to_string(getpid());
  runtime_log::ShmRingWriter writer;
  std::string error;
  ASSERT_TRUE(writer.create(name, "rt", "", {"x"}, 100, &error)) << error;
  for (int cycle = 0; cycle < 250; ++cycle) {
    writer.begin()[0] = cycle;
    writer.commit(cycle, 1000000);
  }
  EXPECT_TRUE(writer.ok());
  writer.close();
  shm_unlink(name.c_str());
}
