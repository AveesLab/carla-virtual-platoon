#include "fv2thread.h"
#include "controller.h"

FV2Thread::FV2Thread(QObject *parent) : QThread(parent)
{

}

void FV2Thread::run()
{
    CmdData fv2_tmp;

    while(1)
    {
//        fv2_tmp.src_index = 255;
//        fv2_tmp.tar_index = 2;
//        emit request(fv2_tmp);

        Controller::fv2_mutex_.lock();
        fv2_tmp = Controller::fv2_data_;
        Controller::fv2_mutex_.unlock();

        emit setValue(fv2_tmp);

        msleep(10);
    }
}
