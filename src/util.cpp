#include "util.h"

#include <cstring>
#include <string>

const char* concat(const char* pref, int id) {
    std::string s = pref;
    s += id+'0';
    return s.c_str();
}

// checks if the given string is a number
bool is_number(const char *ss) {
    std::string s(ss, strlen(ss));
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}