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
