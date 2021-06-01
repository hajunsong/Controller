#include "fileio.h"

void load_data(string file_name, vector<double> *data, string delimiter, unsigned int* size)
{
    data->clear();
    FILE *fp_in;
    const int buffer = 1000000;
    char *ptr, basic[buffer];
    fp_in = fopen(file_name.c_str(), "r");
    int row = 0, col = 0;
    while (fgets(basic, buffer, fp_in) != nullptr)
    {
        row++;
        ptr = strtok(basic, delimiter.c_str());
        while (ptr != nullptr) {
            data->push_back(atof(ptr));
            ptr = strtok(nullptr, delimiter.c_str());
            col++;
        }
    }
    fclose(fp_in);
    if(size != nullptr){
        size[0] = row;
        size[1] = (col/row);
    }
}

void load_data(string file_name, vector<int32_t> *data, string delimiter, unsigned int* size)
{
    data->clear();
    FILE *fp_in;
    const int buffer = 1000000;
    char *ptr, basic[buffer];
    fp_in = fopen(file_name.c_str(), "r");
    int row = 0, col = 0;
    while (fgets(basic, buffer, fp_in) != nullptr)
    {
        row++;
        ptr = strtok(basic, delimiter.c_str());
        while (ptr != nullptr) {
            data->push_back(atoi(ptr));
            ptr = strtok(nullptr, delimiter.c_str());
            col++;
        }
    }
    fclose(fp_in);
    if(size != nullptr){
        size[0] = row;
        size[1] = (col/row);
    }
}
