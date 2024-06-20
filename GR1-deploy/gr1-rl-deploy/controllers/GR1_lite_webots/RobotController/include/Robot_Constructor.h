#ifndef ROBOT_CONSTRUCTOR_H
#define ROBOT_CONSTRUCTOR_H
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>

#include "Robot_Data.h"
// #include "../../MPC_Gait/include/PlaningData.h"
/**
 * @brief The Robot_Constructor class
 */
class Robot_Constructor
{
public:
    Robot_Constructor();
    //construct robot data from jason
    void robotconstructor(QString path,//input jason path
                          Robot_Data * robotdata// robot data
                          );
};

#endif // ROBOT_CONSTRUCTOR_H
