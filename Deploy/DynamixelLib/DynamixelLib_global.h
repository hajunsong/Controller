#ifndef DYNAMIXELLIB_GLOBAL_H
#define DYNAMIXELLIB_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(DYNAMIXELLIB_LIBRARY)
#  define DYNAMIXELLIB_EXPORT Q_DECL_EXPORT
#else
#  define DYNAMIXELLIB_EXPORT Q_DECL_IMPORT
#endif

#endif // DYNAMIXELLIB_GLOBAL_H