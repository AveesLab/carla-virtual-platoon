#include "fv1thread.h"
#include "controller.h"

FV1Thread::FV1Thread(QObject *parent) : QThread(parent)
{

}

void FV1Thread::run()
{
    CmdData fv1_tmp;

    while(1)
    {
//        fv1_tmp.src_index = 255;
//        fv1_tmp.tar_index = 1;
//        emit request(fv1_tmp);

        Controller::fv1_mutex_.lock();
        fv1_tmp = Controller::fv1_data_;
        Controller::fv1_mutex_.unlock();

        emit setValue(fv1_tmp);

        msleep(10);
    }
}
