#ifndef FILEIO_H
#define FILEIO_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include <fstream>

using namespace std;

void load_data(string file_name, vector<double> *data, string delimiter, unsigned int* size=nullptr);
void load_data(string file_name, vector<int32_t> *data, string delimiter, unsigned int* size=nullptr);

#endif // FILEIO_H
