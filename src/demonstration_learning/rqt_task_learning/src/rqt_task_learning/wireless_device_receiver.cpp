#include <rqt_task_learning/task_learning.h>
namespace rqt_task_learning{

bool WirelessDeviceReceiver::eventFilter(QObject* obj, QEvent* event){
    if (event->type()==QEvent::KeyPress) {
        QKeyEvent* key = static_cast<QKeyEvent*>(event);
        if ( (key->key()==Qt::Key_F5) || (key->key()==Qt::Key_PageUp) ) {
            //Start Record
            emit start_record_signal();
        } else if ( (key->key()==Qt::Key_Period) || (key->key()==Qt::Key_PageDown) ) {
            //Stop Record
            emit stop_record_signal();
        } else {
            return QObject::eventFilter(obj, event);
        }
        return true;
    } else {
        return QObject::eventFilter(obj, event);
    }
    return false;
}

}
