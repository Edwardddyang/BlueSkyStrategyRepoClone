/* Text file logger */

#pragma once

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>

class FileLogger {
 private:
  std::string file_name;
  std::ofstream file_stream;

 public:
  FileLogger(std::string file_name) : file_name(file_name) {
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
  void operator()(const std::string text, bool use_log = true, bool add_newline = true);
  void operator()(const char* txt, bool use_log = true, bool add_newline = true);

  ~FileLogger() {
    if (file_stream.is_open()) {
      file_stream.close();
    }
  }
};
