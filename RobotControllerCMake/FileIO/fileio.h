#ifndef FILEIO_H
#define FILEIO_H

#include <QtCore/qglobal.h>

#if defined(FILEIOLIB_LIBRARY)
#  define FILEIOLIB_EXPORT Q_DECL_EXPORT
#else
#  define FILEIOLIB_EXPORT Q_DECL_IMPORT
#endif

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include <fstream>

using namespace std;

void FILEIOLIB_EXPORT load_data(string file_name, vector<double> *data, string delimiter);

#endif // FILEIO_H
