#ifndef METRONOME_STRING_HPP
#define METRONOME_STRING_HPP

#include <string>
#include <vector>
#include <sstream>

void split(const std::string &string, char delimiter, std::vector<std::string> &tokens) {
    std::stringstream ss;
    ss.str(string);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        tokens.push_back(item);
    }
}

std::vector<std::string> split(const std::string &string, char delimiter) {
    std::vector<std::string> tokens;
    split(string, delimiter, tokens);
    return tokens;
}

/** Returns false if EOF, true if success */
bool getNextLine(std::string& line, std::istream& input) {
  do {
    if(!getline(input, line)) {
      return false;
    }
  } while (line.empty());
  return true;
}

#endif //METRONOME_STRING_HPP
