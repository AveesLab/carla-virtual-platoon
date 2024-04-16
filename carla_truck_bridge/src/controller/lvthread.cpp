#include "lvthread.h"
#include "controller.h"

LVThread::LVThread(QObject *parent) : QThread(parent)
{

}

void LVThread::run()
{
    CmdData lv_tmp;
    while(1)
    {
        //lv_tmp.src_index = 255;
        //lv_tmp.tar_index = 0;
        //emit request(lv_tmp); // -> requestData() -> Publish()

        Controller::lv_mutex_.lock();
        lv_tmp = Controller::lv_data_;
        Controller::lv_mutex_.unlock();

        emit setValue(lv_tmp);

        msleep(10);
    }
}
