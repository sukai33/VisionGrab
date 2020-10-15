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

#ifndef SCXMLSTATEMACHINE_H
#define SCXMLSTATEMACHINE_H

#include <QtScxml/qscxmldatamodel.h>
#include <QtScxml/qscxmlexecutablecontent.h>
#include <QtScxml/qscxmlerror.h>
#include <QtScxml/qscxmlevent.h>

#include <QString>
#include <QVector>
#include <QUrl>
#include <QVariantList>

QT_BEGIN_NAMESPACE
class QIODevice;
class QXmlStreamWriter;
class QTextStream;
class QScxmlEventBuilder;
class QScxmlInvokableServiceFactory;
class QScxmlInvokableService;
class QScxmlParser;
class QScxmlStateMachine;
class QScxmlTableData;

class Q_SCXML_EXPORT QScxmlEventFilter
{
public:
    virtual ~QScxmlEventFilter();
    virtual bool handle(QScxmlEvent *event, QScxmlStateMachine *stateMachine) = 0;
};

class QScxmlStateMachinePrivate;
class Q_SCXML_EXPORT QScxmlStateMachine: public QObject
{
    Q_DECLARE_PRIVATE(QScxmlStateMachine)
    Q_OBJECT
    Q_ENUMS(BindingMethod)
    Q_PROPERTY(bool running READ isRunning WRITE setRunning NOTIFY runningChanged)
    Q_PROPERTY(bool initialized READ isInitialized NOTIFY initializedChanged)
    Q_PROPERTY(QScxmlDataModel *dataModel READ dataModel WRITE setDataModel NOTIFY dataModelChanged)
    Q_PROPERTY(QVariantMap initialValues READ initialValues WRITE setInitialValues NOTIFY initialValuesChanged)

protected:
#ifndef Q_QDOC
    explicit QScxmlStateMachine(QObject *parent = nullptr);
    QScxmlStateMachine(QScxmlStateMachinePrivate &dd, QObject *parent);
#endif // Q_QDOC

public:
    enum BindingMethod {
        EarlyBinding,
        LateBinding
    };

    static QScxmlStateMachine *fromFile(const QString &fileName);
    static QScxmlStateMachine *fromData(QIODevice *data, const QString &fileName = QString());
    QVector<QScxmlError> parseErrors() const;

    QString sessionId() const;
    void setSessionId(const QString &id);
    static QString generateSessionId(const QString &prefix);

    bool isInvoked() const;
    bool isInitialized() const;

    void setDataModel(QScxmlDataModel *model);
    QScxmlDataModel *dataModel() const;

    BindingMethod dataBinding() const;

    bool isRunning() const;
    void setRunning(bool running);

    QVariantMap initialValues();
    void setInitialValues(const QVariantMap &initialValues);

    QString name() const;
    QStringList stateNames(bool compress = true) const;
    QStringList activeStateNames(bool compress = true) const;
    bool isActive(const QString &scxmlStateName) const;

    QMetaObject::Connection connectToState(const QString &scxmlStateName,
                                    const QObject *receiver, const char *method,
                                    Qt::ConnectionType type = Qt::AutoConnection);

    QScxmlEventFilter *scxmlEventFilter() const;
    void setScxmlEventFilter(QScxmlEventFilter *newFilter);

    Q_INVOKABLE void submitEvent(QScxmlEvent *event);
    Q_INVOKABLE void submitEvent(const QString &eventName);
    Q_INVOKABLE void submitEvent(const QString &eventName, const QVariant &data);
    void cancelDelayedEvent(const QString &sendId);

    bool isDispatchableTarget(const QString &target) const;

Q_SIGNALS:
    void runningChanged(bool running);
    void log(const QString &label, const QString &msg);
    void reachedStableState();
    void finished();
    void eventOccurred(const QScxmlEvent &event);
    void dataModelChanged(QScxmlDataModel *model);
    void initialValuesChanged(const QVariantMap &initialValues);
    void initializedChanged(bool initialized);
    void externalEventOccurred(const QScxmlEvent &event);

public Q_SLOTS:
    void start();
    void stop();
    bool init();

protected: // methods for friends:
    friend QScxmlDataModel;
    friend QScxmlEventBuilder;
    friend QScxmlInvokableServiceFactory;
    friend QScxmlExecutableContent::QScxmlExecutionEngine;

#ifndef Q_QDOC
    void setDataBinding(BindingMethod bindingMethod);
    virtual void setService(const QString &id, QScxmlInvokableService *service);

    QScxmlTableData *tableData() const;
    void setTableData(QScxmlTableData *tableData);
#endif // Q_QDOC
};

QT_END_NAMESPACE

#endif // SCXMLSTATEMACHINE_H
