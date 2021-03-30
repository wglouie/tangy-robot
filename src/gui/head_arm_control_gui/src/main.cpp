#include <QApplication>
#include <ros/ros.h>
#include "mainwindow.h"
#include <QStringList>
#include <QString>
#include <QDomElement>
#include <QDomDocument>
#include <QFile>

int main(int argc, char *argv[])
{
  // initialise ros
  ros::init(argc, argv, "interface");
  ros::NodeHandle n;

  QApplication app(argc, argv);

  // window;
  MainWindow window(0, &n);

  window.setWindowTitle("Head and Arm Control");
  window.show();

 // initialise ros
  QStringList * max = new QStringList;
  QStringList * min = new QStringList;
  QStringList * reset = new QStringList;

  std::string path = "/home/tangy/tangy-robot/src/gui/head_arm_control_gui/settings/config.xml";

  ROS_INFO("The path is: %s", path.c_str());

  QString Qpath(path.c_str());

  QDomDocument doc("config");
  QFile file(Qpath);
  if(!file.open(QIODevice::ReadOnly)) {
    ROS_INFO("File Error: Could not open");
    return false;
  }
  if(!doc.setContent(&file)) {
     ROS_INFO("Doc Error: could not set content of file");
     return false;
  }
  file.close();

  QDomElement docElem = doc.documentElement();

  for(QDomNode n = docElem.firstChild(); !n.isNull(); n = n.nextSibling()) {
    QDomElement element = n.toElement();
    if(element.isNull()) {
      continue;
    }
    if(element.tagName() == "head") {
      for(QDomNode n1 = n.firstChild(); !n1.isNull(); n1 = n1.nextSibling()) {
        element = n1.toElement();
        if(element.isNull()) {
          continue;
        }
        if(element.tagName() == "servo") {
          for(QDomNode n2 = n1.firstChild(); !n2.isNull(); n2 = n2.nextSibling()) {
            element = n2.toElement();
            if(element.isNull()) {
              continue;
            }
            if(element.tagName() == "reset") {
              reset->append(element.attribute("value"));
            }        
            if(element.tagName() == "max") {
              max->append(element.attribute("value"));
            }
            if(element.tagName() == "min") {
              min->append(element.attribute("value"));
            }
          }
        }
        if(element.tagName() == "time") {
          for(QDomNode n2 = n1.firstChild(); !n2.isNull(); n2 = n2.nextSibling()) {
            element = n2.toElement();
            if(element.tagName() == "period") {
              reset->append(element.attribute("value"));
            }
          }
        }
      }
    }


    if(element.tagName() == "right_arm") {
      for(QDomNode n1 = n.firstChild(); !n1.isNull(); n1 = n1.nextSibling()) {
        element = n1.toElement();
        if(element.isNull()) {
          continue;
        }
        if(element.tagName() == "servo") {
          for(QDomNode n2 = n1.firstChild(); !n2.isNull(); n2 = n2.nextSibling()) {
            element = n2.toElement();
            if(element.isNull()) {
              continue;
            }
            if(element.tagName() == "reset") {
              reset->append(element.attribute("value"));
            }        
            if(element.tagName() == "max") {
              max->append(element.attribute("value"));
            }
            if(element.tagName() == "min") {
              min->append(element.attribute("value"));
            }
          }
        }
        if(element.tagName() == "time") {
          for(QDomNode n2 = n1.firstChild(); !n2.isNull(); n2 = n2.nextSibling()) {
            element = n2.toElement();
            if(element.tagName() == "period") {
              reset->append(element.attribute("value"));
            }
          }
        }
      }
    }

    if(element.tagName() == "left_arm") {
      for(QDomNode n1 = n.firstChild(); !n1.isNull(); n1 = n1.nextSibling()) {
        element = n1.toElement();
        if(element.isNull()) {
          continue;
        }
        if(element.tagName() == "servo") {
          for(QDomNode n2 = n1.firstChild(); !n2.isNull(); n2 = n2.nextSibling()) {
            element = n2.toElement();
            if(element.isNull()) {
              continue;
            }
            if(element.tagName() == "reset") {
              reset->append(element.attribute("value"));
            }        
            if(element.tagName() == "max") {
              max->append(element.attribute("value"));
            }
            if(element.tagName() == "min") {
              min->append(element.attribute("value"));
            }
          }
        }
        if(element.tagName() == "time") {
          for(QDomNode n2 = n1.firstChild(); !n2.isNull(); n2 = n2.nextSibling()) {
            element = n2.toElement();
            if(element.tagName() == "period") {
              reset->append(element.attribute("value"));
            }
          }
        }
      }
    }


  }
/*
  ROS_INFO("The values in reset are:");
  for(int i = 0; i < (*reset).size(); i++)
    ROS_INFO("%d", ((*reset)[i]).toInt());
  ROS_INFO("The values in max are:");
  for(int i = 0; i < (*max).size(); i++)
    ROS_INFO("%d", ((*max)[i]).toInt());
  ROS_INFO("The values in min are:");
  for(int i = 0; i < (*min).size(); i++)
    ROS_INFO("%d", ((*min)[i]).toInt());
  ROS_INFO("The values in time is:");
  for(int i = 0; i < (*time).size(); i++)
    ROS_INFO("%d", ((*time)[i]).toInt());*/
  
  window.setParameters(*max, *min, *reset); 
  window.setConnections(); 

  // spin
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ROS_INFO("Starting application");
  int return_value = app.exec();
  ROS_INFO("Qutting app");
  window.close();

  return return_value;
}
