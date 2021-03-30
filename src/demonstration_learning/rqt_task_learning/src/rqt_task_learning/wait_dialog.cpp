#include <rqt_task_learning/task_learning.h>
namespace rqt_task_learning{

WaitDialog::WaitDialog(const QString &message, QWidget *parent)
: QDialog(parent, Qt::FramelessWindowHint),
      msgLabel(new QLabel(message, this)),
      stack(0)
{
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->setContentsMargins(30, 30, 30, 30);
    mainLayout->addWidget(msgLabel);
    setLayout(mainLayout);
}

void WaitDialog::show()
{
    QDialog::show();

        emit visibilityChanged(true);
}

void WaitDialog::hide()
{
    QDialog::hide();
    emit visibilityChanged(false);
}

void WaitDialog::push(QString msg)
{
    if(msg.trimmed().isEmpty()){
        //Default message
        msg = "Conducting process";
    }
    msgLabel->setText(msg);
    stack++;
    show();

    emit visibilityChanged(true);
}

void WaitDialog::pop()
{
    if (stack > 0)
        --stack;
    if (stack == 0)
    {
        hide();
    }
}

}
