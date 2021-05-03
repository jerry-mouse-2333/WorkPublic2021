#ifndef _ASSIT_H
#define _ASSIT_H
#include <signal.h>
#include <csignal>
#include <iostream>
// #include <stdio.h>
#include <vector>
#include <string>
#include "json/json.h"
using namespace std;

vector<string> split_function(string input_str,const char *split_signal);

string get_value(istream &inputfile);

#endif