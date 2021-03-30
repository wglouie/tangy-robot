#include <rqt_task_learning/task_learning.h>
namespace rqt_task_learning{


SkeletonWatcher::SkeletonWatcher(QObject *parent) :
    QObject(parent){
    num_skeletons = 0;
    m_mutex = new QMutex();
}
void SkeletonWatcher::update_num_skeletons(int value){
    QMutexLocker locker(m_mutex);
    num_skeletons = value;
}

void SkeletonWatcher::track_skeletons(){
    while(true){
        QMutexLocker locker(m_mutex);
        if(num_skeletons > 0){
            break;
        }
    }
    emit done();
}

}
