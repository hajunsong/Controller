#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QPlainTextEdit>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#if __win32__
#include <direct.h>
#elif __linux__
#include <sys/stat.h>
#include <sys/types.h>
#endif

class Logger : public QObject
{
    Q_OBJECT
public:
    explicit Logger(QObject *parent, QString fileName, QPlainTextEdit *editer = 0);
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
