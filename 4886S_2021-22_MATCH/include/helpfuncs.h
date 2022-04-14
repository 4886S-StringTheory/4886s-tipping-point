#include "robot-config.h"
#include <string.h>
void print_centered(int xcenter, int y, std::string str);

void print_centered(int xcenter, int y, std::string str) {
  // Count chars in string
  const char *cstr = str.c_str();
  int strWidth = BSCREEN.getStringWidth(cstr);

  // Center the on given x value
  int xpos = xcenter - strWidth / 2;
  BSCREEN.setCursor(xpos, y);
  BSCREEN.print(cstr);
}