/****************************************************************************
** Meta object code from reading C++ file 'dynamic3dtracking.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/dynamic3dtracking.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dynamic3dtracking.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Dynamic3DTracking_t {
    QByteArrayData data[14];
    char stringdata0[297];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Dynamic3DTracking_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Dynamic3DTracking_t qt_meta_stringdata_Dynamic3DTracking = {
    {
QT_MOC_LITERAL(0, 0, 17), // "Dynamic3DTracking"
QT_MOC_LITERAL(1, 18, 22), // "on_btnSavePath_clicked"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 22), // "on_btnOpenCams_clicked"
QT_MOC_LITERAL(4, 65, 9), // "timerLout"
QT_MOC_LITERAL(5, 75, 9), // "timerRout"
QT_MOC_LITERAL(6, 85, 18), // "on_btnExit_clicked"
QT_MOC_LITERAL(7, 104, 30), // "on_btnSingleCamCapture_clicked"
QT_MOC_LITERAL(8, 135, 28), // "on_btnTwoCamsCapture_clicked"
QT_MOC_LITERAL(9, 164, 23), // "on_btnCloseCams_clicked"
QT_MOC_LITERAL(10, 188, 24), // "on_btnSetExpTime_clicked"
QT_MOC_LITERAL(11, 213, 23), // "on_btnSetPicNum_clicked"
QT_MOC_LITERAL(12, 237, 29), // "on_externalTrigButton_clicked"
QT_MOC_LITERAL(13, 267, 29) // "on_btnDynamicTracking_clicked"

    },
    "Dynamic3DTracking\0on_btnSavePath_clicked\0"
    "\0on_btnOpenCams_clicked\0timerLout\0"
    "timerRout\0on_btnExit_clicked\0"
    "on_btnSingleCamCapture_clicked\0"
    "on_btnTwoCamsCapture_clicked\0"
    "on_btnCloseCams_clicked\0"
    "on_btnSetExpTime_clicked\0"
    "on_btnSetPicNum_clicked\0"
    "on_externalTrigButton_clicked\0"
    "on_btnDynamicTracking_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Dynamic3DTracking[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    0,   85,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Dynamic3DTracking::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Dynamic3DTracking *_t = static_cast<Dynamic3DTracking *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_btnSavePath_clicked(); break;
        case 1: _t->on_btnOpenCams_clicked(); break;
        case 2: _t->timerLout(); break;
        case 3: _t->timerRout(); break;
        case 4: _t->on_btnExit_clicked(); break;
        case 5: _t->on_btnSingleCamCapture_clicked(); break;
        case 6: _t->on_btnTwoCamsCapture_clicked(); break;
        case 7: _t->on_btnCloseCams_clicked(); break;
        case 8: _t->on_btnSetExpTime_clicked(); break;
        case 9: _t->on_btnSetPicNum_clicked(); break;
        case 10: _t->on_externalTrigButton_clicked(); break;
        case 11: _t->on_btnDynamicTracking_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject Dynamic3DTracking::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_Dynamic3DTracking.data,
      qt_meta_data_Dynamic3DTracking,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Dynamic3DTracking::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Dynamic3DTracking::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Dynamic3DTracking.stringdata0))
        return static_cast<void*>(const_cast< Dynamic3DTracking*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int Dynamic3DTracking::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
