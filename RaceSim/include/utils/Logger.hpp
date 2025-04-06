/* Text file logger */

#pragma once

#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <string>

class FileLogger {
 private:
  std::string file_name;
  std::ofstream file_stream;
  bool log;

 public:
  explicit FileLogger(std::string file_name, bool use_logger = false) : file_name(file_name), log(use_logger) {
    if (!use_logger) return;
    file_stream.open(file_name);
  }
  FileLogger() {}

  FileLogger(const FileLogger& other) : file_name(other.file_name) {
    if (!other.file_name.empty()) {
      file_stream.open(other.file_name, std::ios::out | std::ios::app);
    }
  }

  // Copy assignment operator
  FileLogger& operator=(const FileLogger& other) {
    if (this != &other) {
      file_name = other.file_name;
      log = other.log;
      if (file_stream.is_open()) {
        file_stream.close();
      }
      if (!other.file_name.empty()) {
        file_stream.open(other.file_name, std::ios::out | std::ios::app);
      }
    }
    return *this;
  }

  FileLogger& operator<<(const std::string text);
  FileLogger& operator<<(const char* txt);
  void operator()(const std::string text, bool add_newline = true);
  void operator()(const char* txt, bool add_newline = true);

  ~FileLogger() {
    if (file_stream.is_open()) {
      file_stream.close();
    }
  }
};
