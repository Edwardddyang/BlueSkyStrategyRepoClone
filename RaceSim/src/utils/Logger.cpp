#include "utils/Logger.hpp"

FileLogger& FileLogger::operator<<(const std::string text) {
  file_stream << text << "\n";
  file_stream.flush();
  return *this;
}

FileLogger& FileLogger::operator<<(const char* txt) {
  file_stream << txt << "\n";
  file_stream.flush();
  return *this;
}

void FileLogger::operator()(const std::string text, bool use_log, bool add_newline) {
  if (!use_log) {
    return;
  }

  file_stream << text;

  if (add_newline) {
    file_stream << "\n";
  }
  file_stream.flush();
}

void FileLogger::operator()(const char* txt, bool use_log, bool add_newline) {
  if (!use_log) {
    return;
  }

  file_stream << txt;

  if (add_newline) {
    file_stream << "\n";
  }

  file_stream.flush();
}
