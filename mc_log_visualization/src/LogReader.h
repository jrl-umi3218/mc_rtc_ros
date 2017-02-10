#pragma once

#include <map>
#include <string>
#include <vector>

struct LogReader : public std::map<std::string, std::vector<double>>
{
  LogReader();

  void read(const std::string & filename);
};
