#pragma once
#include <fstream>
#include <iostream>
#include <string>


using namespace std;

class Logger {


public:
  std::ofstream of_;
  ~Logger() {
    if (of_.is_open()) {
      of_ << "******** END *******" << endl;
      of_.close();
    }
  }
  Logger(std::string file) {
    if (!of_.is_open()) {
      of_.open(file);
    }
  }

  void write(std::string text)
  {
    of_ << text << endl;
  }
};