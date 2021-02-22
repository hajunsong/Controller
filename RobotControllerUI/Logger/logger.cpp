#include "logger.h"

Logger::Logger(QObject *parent, QString fileName, QPlainTextEdit *editor) : QObject(parent) {
    m_editor = editor;
    m_showDate = false;

    QString folder = qApp->applicationDirPath() + "/logging";
    QString newFilename = folder + "/" + fileName;
    QDir dir;
    dir.mkdir(folder);

    if (!newFilename.isEmpty()) {
        file = new QFile;
        file->setFileName(newFilename);
        file->open(QIODevice::Append | QIODevice::Text);
    }
}

void Logger::write(const QString &value) {
    QString text = value;// + "";
    if (m_showDate)
        text = QDateTime::currentDateTime().toString("ss.zzz,") + text + "\n";
    QTextStream out(file);
//    out.setCodec("UTF-8");
    out.setEncoding(QStringConverter::Encoding::Utf8);
    if (file != nullptr) {
        out << text;
    }
    if (m_editor != nullptr)
        m_editor->appendPlainText(text);
}

void Logger::setShowDateTime(bool value) {
    m_showDate = value;
}

Logger::~Logger() {
    if (file != nullptr){
        file->close();
    }
}
