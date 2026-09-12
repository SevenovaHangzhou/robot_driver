#ifndef DM_SWERVE_DRIVER__SRC__ATOMIC_TEXT_FILE_HPP_
#define DM_SWERVE_DRIVER__SRC__ATOMIC_TEXT_FILE_HPP_
#include <filesystem>
#include <string>
namespace dm_swerve_driver {
void write_atomic_text(const std::filesystem::path & path, const std::string & text);
}
#endif
