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

#ifndef SCXMLQSTATES_H
#define SCXMLQSTATES_H

#include <QtScxml/qscxmlstatemachine.h>
#include <QtScxml/qscxmlinvokableservice.h>

#include <QAbstractTransition>
#include <QFinalState>
#include <QHistoryState>
#include <QState>

QT_BEGIN_NAMESPACE

template<class T>
class QScxmlInvokeScxmlFactory: public QScxmlInvokableScxmlServiceFactory
{
public:
    QScxmlInvokeScxmlFactory(QScxmlExecutableContent::StringId invokeLocation,
                             QScxmlExecutableContent::StringId id,
                             QScxmlExecutableContent::StringId idPrefix,
                             QScxmlExecutableContent::StringId idlocation,
                             const QVector<QScxmlExecutableContent::StringId> &namelist,
                             bool doAutoforward,
                             const QVector<Param> &params,
                             QScxmlExecutableContent::ContainerId finalize)
        : QScxmlInvokableScxmlServiceFactory(invokeLocation, id, idPrefix, idlocation, namelist,
                                             doAutoforward, params, finalize)
    {}

    QScxmlInvokableService *invoke(QScxmlStateMachine *parent) Q_DECL_OVERRIDE
    {
        return finishInvoke(new T, parent);
    }
};

class QScxmlStatePrivate;
class Q_SCXML_EXPORT QScxmlState: public QState
{
    Q_OBJECT

public:
    QScxmlState(QState *parent = Q_NULLPTR);
    QScxmlState(QScxmlStateMachine *parent);
    ~QScxmlState();

    void setAsInitialStateFor(QScxmlState *state);
    void setAsInitialStateFor(QScxmlStateMachine *stateMachine);

    QScxmlStateMachine *stateMachine() const;
    QString stateLocation() const;

    void setInitInstructions(QScxmlExecutableContent::ContainerId instructions);
    void setOnEntryInstructions(QScxmlExecutableContent::ContainerId instructions);
    void setOnExitInstructions(QScxmlExecutableContent::ContainerId instructions);
    void setInvokableServiceFactories(const QVector<QScxmlInvokableServiceFactory *>& factories);

Q_SIGNALS:
    void didEnter(); // TODO: REMOVE!
    void willExit(); // TODO: REMOVE!

protected:
    void onEntry(QEvent * event) Q_DECL_OVERRIDE;
    void onExit(QEvent * event) Q_DECL_OVERRIDE;

private:
    Q_DECLARE_PRIVATE(QScxmlState)
};

class QScxmlFinalStatePrivate;
class Q_SCXML_EXPORT QScxmlFinalState: public QFinalState
{
    Q_OBJECT

public:
    QScxmlFinalState(QState *parent = Q_NULLPTR);
    QScxmlFinalState(QScxmlStateMachine *parent);
    ~QScxmlFinalState();

    void setAsInitialStateFor(QScxmlState *state);
    void setAsInitialStateFor(QScxmlStateMachine *stateMachine);

    QScxmlStateMachine *stateMachine() const;

    QScxmlExecutableContent::ContainerId doneData() const;
    void setDoneData(QScxmlExecutableContent::ContainerId doneData);

    void setOnEntryInstructions(QScxmlExecutableContent::ContainerId instructions);
    void setOnExitInstructions(QScxmlExecutableContent::ContainerId instructions);

protected:
    void onEntry(QEvent * event) Q_DECL_OVERRIDE;
    void onExit(QEvent * event) Q_DECL_OVERRIDE;

private:
    Q_DECLARE_PRIVATE(QScxmlFinalState)
};

class Q_SCXML_EXPORT QScxmlHistoryState: public QHistoryState
{
    Q_OBJECT

public:
    QScxmlHistoryState(QState *parent = Q_NULLPTR);
    ~QScxmlHistoryState();

    void setAsInitialStateFor(QScxmlState *state);
    void setAsInitialStateFor(QScxmlStateMachine *stateMachine);

    QScxmlStateMachine *stateMachine() const;
};

class QScxmlBaseTransitionPrivate;
class Q_SCXML_EXPORT QScxmlBaseTransition: public QAbstractTransition
{
    Q_OBJECT
    class Data;

public:
    QScxmlBaseTransition(QState * sourceState = Q_NULLPTR,
                         const QStringList &eventSelector = QStringList());
    ~QScxmlBaseTransition();

    QScxmlStateMachine *stateMachine() const;
    QString transitionLocation() const;

    bool eventTest(QEvent *event) Q_DECL_OVERRIDE;

protected:
    void onTransition(QEvent *event) Q_DECL_OVERRIDE;

    QScxmlBaseTransition(QScxmlBaseTransitionPrivate &dd, QState *parent,
                         const QStringList &eventSelector = QStringList());

private:
    Q_DECLARE_PRIVATE(QScxmlBaseTransition)
};

class QScxmlTransitionPrivate;
class Q_SCXML_EXPORT QScxmlTransition: public QScxmlBaseTransition
{
    Q_OBJECT

public:
    QScxmlTransition(QState * sourceState = Q_NULLPTR,
                     const QStringList &eventSelector = QStringList());
    QScxmlTransition(const QStringList &eventSelector);
    ~QScxmlTransition();

    void addTransitionTo(QScxmlState *state);
    void addTransitionTo(QScxmlStateMachine *stateMachine);

    bool eventTest(QEvent *event) Q_DECL_OVERRIDE;
    QScxmlStateMachine *stateMachine() const;

    void setInstructionsOnTransition(QScxmlExecutableContent::ContainerId instructions);
    void setConditionalExpression(QScxmlExecutableContent::EvaluatorId evaluator);

protected:
    void onTransition(QEvent *event) Q_DECL_OVERRIDE;

private:
    Q_DECLARE_PRIVATE(QScxmlTransition)
};

QT_END_NAMESPACE

#endif // SCXMLQSTATES_H
