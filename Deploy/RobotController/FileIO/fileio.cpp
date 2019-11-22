/*
 * fileio.cpp
 *
 *  Created on: 2019. 5. 27.
 *      Author: keti-hajun
 */

#include "fileio.h"

void load_data(string file_name, vector<double> *data, string delimiter)
{
    data->clear();
    FILE *fp_in;
    const int buffer = 1000000;
    char *ptr, basic[buffer];
    fp_in = fopen(file_name.c_str(), "r");
    while (fgets(basic, buffer, fp_in) != nullptr)
    {
        ptr = strtok(basic, delimiter.c_str());
        while (ptr != nullptr) {
            data->push_back(atof(ptr));
            ptr = strtok(nullptr, delimiter.c_str());
        }
    }
    fclose(fp_in);
}

void load_data(string file_name, vector< vector<float> > *data)
{
    data->clear();
    FILE *fp_in;
    const int buffer = 1000000;
    char *ptr, basic[buffer];
    fp_in = fopen(file_name.c_str(), "r");
    while (fgets(basic, buffer, fp_in) != nullptr)
    {
        ptr = strtok(basic, "\t");
        vector<float> data_temp;
        while (ptr != nullptr) {
            data_temp.push_back(static_cast<float>(atof(ptr)));
            ptr = strtok(nullptr, "\t");
        }
        data->push_back(data_temp);
    }
    fclose(fp_in);
}

