/* Defines custom exceptions */

#pragma once

#include <iostream>
#include <exception>
#include <string>

// For invalid calculation results e.g. negative discriminant
// in the quadratic formula
class InvalidCalculation : public std::exception {
 private:
  std::string message;

 public:
  explicit InvalidCalculation(const std::string& msg) : message("Invalid calculation result " + msg) {}

  const char* what() const noexcept override {
    return message.c_str();
  }
};
