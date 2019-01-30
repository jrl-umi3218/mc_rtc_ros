#include "LogReader.h"

#include <mc_rtc/logging.h>

#include <fstream>
#include <sstream>

LogReader::LogReader() {}

void LogReader::read(const std::string & filename)
{
  clear();
  std::ifstream ifs(filename);
  if(!ifs.is_open())
  {
    LOG_ERROR("Unable to open " << filename)
    throw(std::runtime_error("Unable to open log file"));
  }

  bool first_line = true;
  std::vector<std::string> entries;
  std::vector<std::vector<double>> fdata;
  for(std::string line; std::getline(ifs, line);)
  {
    std::stringstream liness;
    liness << line;
    unsigned int i = 0;
    for(std::string data; std::getline(liness, data, ';'); ++i)
    {
      if(data.size() != 0)
      {
        if(first_line)
        {
          entries.push_back(data);
        }
        else
        {
          if(i < fdata.size())
          {
            try
            {
              fdata[i].push_back(std::stod(data));
            }
            catch(std::out_of_range & e)
            {
              fdata[i].push_back(0);
            }
          }
        }
      }
    }
    if(first_line)
    {
      fdata.resize(entries.size());
    }
    first_line = false;
  }
  for(size_t i = 0; i < entries.size(); ++i)
  {
    (*this)[entries[i]] = std::move(fdata[i]);
  }
  LOG_SUCCESS("Parsed " << filename)
}
