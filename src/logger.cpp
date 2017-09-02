#include <iostream>
#include <sstream>

// namespace LOGGER {
  
std::string Log::ToString(TLogLevel level)
{
  return logLevelStrings[level];
}

std::ostringstream& Log::Get(TLogLevel level)
{
   os << " " << ToString(level) << ": ";
   os << std::string(level > logDEBUG ? 0 : level - logDEBUG, '\t');
   messageLevel = level;
   return os;
}
Log::~Log()
{
   if (messageLevel >= Log::ReportingLevel())
   {
      os << std::endl;
      fprintf(stderr, "%s", os.str().c_str());
      fflush(stderr);
   }
}

TLogLevel& Log::ReportingLevel()
{
  return messageLevel;
}

// }