#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QPlainTextEdit>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <sys/stat.h>
#include <sys/types.h>
#include <QApplication>
#include <QDir>

#include <QtCore/qglobal.h>

#if defined(LOGGERLIB_LIBRARY)
#  define LOGGERLIB_EXPORT Q_DECL_EXPORT
#else
#  define LOGGERLIB_EXPORT Q_DECL_IMPORT
#endif

class Logger : public QObject
{
    Q_OBJECT
public:
    explicit Logger(QObject *parent, QString fileName, QPlainTextEdit *editer = nullptr);
    ~Logger();
    void setShowDateTime(bool value);
    QFile *file;
    bool loggerStart;

private:
    QPlainTextEdit *m_editor;
    bool m_showDate;

signals:

public slots:
    void write(const QString &value);
};

#endif // LOGGER_H
