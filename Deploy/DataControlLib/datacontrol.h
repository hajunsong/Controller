#ifndef DATACONTROL_H
#define DATACONTROL_H

#include <QtCore/qglobal.h>

#if defined(DATACONTROLLIB_LIBRARY)
#  define DATACONTROLLIB_EXPORT Q_DECL_EXPORT
#else
#  define DATACONTROLLIB_EXPORT Q_DECL_IMPORT
#endif

class DATACONTROLLIB_EXPORT DataControl
{
public:
    DataControl();
};

#endif // DATACONTROL_H
