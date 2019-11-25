#ifndef KEYINPUTLIB_GLOBAL_H
#define KEYINPUTLIB_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(KEYINPUTLIB_LIBRARY)
#  define KEYINPUTLIB_EXPORT Q_DECL_EXPORT
#else
#  define KEYINPUTLIB_EXPORT Q_DECL_IMPORT
#endif

#endif // KEYINPUTLIB_GLOBAL_H
