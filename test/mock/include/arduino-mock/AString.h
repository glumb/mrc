#ifndef STRING_H
#define STRING_H 1

#include <string>
// #include <gmock/gmock.h>

// namespace {
// std::string ReplaceAll(std::string str, const std::string& from, const std::string& to) {
//     size_t start_pos = 0;
//     while((start_pos = str.find(from, start_pos)) != std::string::npos) {
//         str.replace(start_pos, from.length(), to);
//         start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
//     }
//     return str;
// }
// }


class String : public std::string {
 public:
  String() {}
  String(const String& string) : String(string.c_str()) {}
  String(const std::string& string) : std::string(string) {}
  String(const char* string) : std::string(string) {}
  String(const char string) : std::string(1,string) {}
  String(const float val) : std::string(std::to_string(val)) {}
  String(const int val) : std::string(std::to_string(val)) {}
  String(const unsigned int val) : std::string(std::to_string(val)) {}
  String(const long val) : std::string(std::to_string(val)) {}
  String(const unsigned long val) : std::string(std::to_string(val)) {}
  String(const double val) : std::string(std::to_string(val)) {}

  String substring(int start) {
    return substr(start);
  }
  String substring(int start, int end) {
    return substr(start, end-start);
  }

  String remove(int start) {
    return erase(start);
  }

  // void replace(const String& from, const String& to) {
  //   ReplaceAll(*this, from, to);
  // }

  int indexOf(const char needle) {
    return find(needle);
  }
};

#endif
