/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtScxml module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 2.0 or (at your option) the GNU General
** Public license version 3 or any later version approved by the KDE Free
** Qt Foundation. The licenses are as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL2 and LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-2.0.html and
** https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef SCXMLPARSER_H
#define SCXMLPARSER_H

#include <QtScxml/qscxmlerror.h>

#include <QStringList>
#include <QString>

QT_BEGIN_NAMESPACE
class QXmlStreamReader;
class QScxmlStateMachine;

class QScxmlParserPrivate;
class Q_SCXML_EXPORT QScxmlParser
{
public:
    class Q_SCXML_EXPORT Loader
    {
    public:
        Loader(QScxmlParser *parser);
        virtual ~Loader();
        virtual QByteArray load(const QString &name, const QString &baseDir, bool *ok) = 0;

    protected:
        QScxmlParser *parser() const;

    private:
        QScxmlParser *m_parser;
    };

    enum QtMode {
        QtModeDisabled,
        QtModeEnabled,
        QtModeFromInputFile
    };

public:
    QScxmlParser(QXmlStreamReader *xmlReader);
    ~QScxmlParser();

    QString fileName() const;
    void setFileName(const QString &fileName);

    Loader *loader() const;
    void setLoader(Loader *newLoader);

    void parse();
    QScxmlStateMachine *instantiateStateMachine() const;
    void instantiateDataModel(QScxmlStateMachine *stateMachine) const;

    QVector<QScxmlError> errors() const;
    void addError(const QString &msg);

    QtMode qtMode() const;
    void setQtMode(QtMode mode);

private:
    friend class QScxmlParserPrivate;
    QScxmlParserPrivate *d;
};

QT_END_NAMESPACE

#endif // SCXMLPARSER_H
