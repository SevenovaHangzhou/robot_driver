#include "atomic_text_file.hpp"
#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

namespace dm_swerve_driver {
void write_atomic_text(const std::filesystem::path & path, const std::string & text)
{
  std::string name{path.string() + ".tmpXXXXXX"};
  std::vector<char> pattern(name.begin(), name.end());
  pattern.push_back('\0');
  const int fd = ::mkstemp(pattern.data());
  if (fd < 0) {throw std::system_error{errno, std::generic_category(), "create atomic file"};}
  bool open{true};
  try {
    std::size_t offset{0U};
    while (offset < text.size()) {
      const auto written = ::write(fd, text.data() + offset, text.size() - offset);
      if (written < 0 && errno == EINTR) {continue;}
      if (written <= 0) {throw std::system_error{errno, std::generic_category(), "write atomic file"};}
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(fd) != 0) {throw std::system_error{errno, std::generic_category(), "sync atomic file"};}
    open = false;
    if (::close(fd) != 0) {throw std::system_error{errno, std::generic_category(), "close atomic file"};}
    std::filesystem::rename(pattern.data(), path);
    const auto parent = path.parent_path().empty() ? std::filesystem::path{"."} : path.parent_path();
    const int dir = ::open(parent.c_str(), O_DIRECTORY | O_RDONLY);
    if (dir < 0) {throw std::system_error{errno, std::generic_category(), "open state directory"};}
    const int result = ::fsync(dir);
    const int saved_errno = errno;
    ::close(dir);
    if (result != 0) {throw std::system_error{saved_errno, std::generic_category(), "sync state directory"};}
  } catch (...) {
    if (open) {::close(fd);}
    std::error_code ignored;
    std::filesystem::remove(pattern.data(), ignored);
    throw;
  }
}
}
