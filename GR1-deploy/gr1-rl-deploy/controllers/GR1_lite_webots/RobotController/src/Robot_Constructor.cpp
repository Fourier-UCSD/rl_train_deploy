#include "../include/Robot_Constructor.h"
// #include "Robot_Constructor.h"
#include <vector>
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <QStringList>
#include <QDebug>
#include <QFile>
Robot_Constructor::Robot_Constructor()
{

}
// to be done:
// friction cone constriants
void Robot_Constructor::robotconstructor(QString path, Robot_Data *robotdata)
{
    // task card set
    int _npriority = -1;
    int _ntask = 0;
    //std::vector<Task*> _task_card_set;
    // sensor set
    int _nsensor;
    //std::vector<Sensor*> _sensor_set;
    // tau external
    Eigen::VectorXd _tau_ext;
    // total joint num
    int _ndof;
    // wheel dof
    int _ndof_wheel=0;
    //double _radius_wheel;
    //double _length_wheel;
    // robot type
    enum robot_type _robottype;
    // solver type
    enum Wbc_Solver_type _wbcsolver;
    // gravity
    Eigen::Vector3d _gravity;
    _gravity<<0.0,0.0,-9.81;
    // link mass, com and inertial
    std::vector<double> _mass;
    std::vector<Eigen::MatrixXd> _link_com;
    std::vector<Eigen::MatrixXd> _link_inertial;
        // Homogeneous transformation matrix
        /**
         * @brief _T_list :
         * Float_Base_Open_Chain: _T_list[0] is the transformation matrix from floating base to the first link,
         * floating base joint has the order: x y z theta_x thata_y thata_z
         * Fixed_Base_Open_Chain: _T_list[0] is the transformaton matrix from base to the first link
         * Mobile_Wheel_Open_Chain: is same to the Float_Base_Open_Chain, except for,
         * the mobile wheel joint is always at the bottom of the _T_list,
         * the coordinate frame of the its floating base has the default: x-direction is the headed direction, z_direction is in the opposite direction of gravity
         */
        std::vector<Eigen::MatrixXd> _T_list;

        /**
         * @brief _No:
         * Fixed_Base_Open_Chain _No = _No - 1; _parent = _parent - 1;
         * Float_Base_Open_Chain _No = _No; _parent = _parent
         */
        std::vector<int> _No;
        std::vector<int> _parent;
        /**
         * @brief _axis
         * 0 for FloatJoint, 1 for +X, 2 for -X, 3 for +Y, 4 for -Y, 5 for +Z, 6 for -Z
         */
        std::vector<int> _axis;

    // task direction selection
        std::vector<bool> _task_direction_selection;
        _task_direction_selection.resize(6);
        for(int i = 0;i<6;i++)
        {
            _task_direction_selection[i] = false;
        }
    // task weight
        std::vector<double> _task_weight;
        _task_weight.resize(6);
        for(int i=0;i<6;i++)
        {
            _task_weight[i] = 1.0;
        }
    // read the json file
    QFile loadFile(path);

    if(!loadFile.open(QIODevice::ReadOnly))
    {
        qDebug() << "could't open projects json";
        return;
    }

    QByteArray allData = loadFile.readAll();
    loadFile.close();

    QJsonParseError jsonerror;
    QJsonDocument doc(QJsonDocument::fromJson(allData, &jsonerror));

    if(jsonerror.error != QJsonParseError::NoError)
    {
        qDebug() << "json error!" << jsonerror.errorString();
        return;
    }

    if (!doc.isNull() && jsonerror.error == QJsonParseError::NoError)
    {
        if(doc.isObject())
        {
            QJsonObject object = doc.object();
            QJsonObject::iterator it = object.begin();
            // read the dimesion of the variables
            while(it != object.end())
            {
                if(it.key()=="algorithm_type")
                {
                    _wbcsolver = Wbc_Solver_type(it.value().toInt() + 1);
                    std::cout<<"algorithm_type is set to:"<<_wbcsolver<<std::endl;
                }

                if(it.key()=="robot_type")
                {
                    _robottype = robot_type(it.value().toInt() + 1);
                    std::cout<<"robot_type is set to:"<<_robottype<<std::endl;
                }

                if(it.key()=="joint_num")
                {
                    _ndof = it.value().toInt();
                    std::cout<<"joint_num is set to:"<<_ndof<<std::endl;
                }

                if(it.key()=="task_num")
                {
                    _ntask = it.value().toInt();
                    std::cout<<"task_num is set to:"<<_ntask<<std::endl;
                }

                if(it.key()=="sensor_num")
                {
                    _nsensor = it.value().toInt();
                    std::cout<<"sensor_num is set to:"<<_nsensor<<std::endl;
                }

                if(it.key()=="gravityArray")
                {
                    _gravity(0) = it.value().toArray().at(0).toDouble();
                    _gravity(1) = it.value().toArray().at(1).toDouble();
                    _gravity(2) = it.value().toArray().at(2).toDouble();
                }
                it++;
            }
            // init set
            robotdata->wbcsolver = _wbcsolver;
            robotdata->robottype = _robottype;
            if(robotdata->robottype == robot_type::Float_Base_Open_Chain || robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
            {
                robotdata->ndof = _ndof + 5;
            }else if(robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
            {
                robotdata->ndof = _ndof;
            }else{
                std::cout<<"robot_type is error!"<<std::endl;
                return;
            }
            _No.resize(_ndof);
            _parent.resize(_ndof);
            _axis.resize(_ndof);
            _T_list.resize(_ndof);
            for(int _T_i = 0; _T_i<_ndof;_T_i++)
            {
                _T_list[_T_i].setIdentity(4,4);
            }
            robotdata->ntask = _ntask;
            robotdata->id_body.resize(_ndof);
            robotdata->robot_body.resize(_ndof);
            robotdata->robot_joint.resize(_ndof);
            // npriority not init
            robotdata->ntask = _ntask;
            robotdata->task_card_set.resize(_ntask);
            robotdata->sensor_set.resize(_nsensor);
            robotdata->tau_ext.setZero(robotdata->ndof);
            // ndof_wheel not init
            // radius_wheel not init
            // length_wheel not init
            robotdata->q_lbound.setZero(robotdata->ndof,1);
            robotdata->q_ubound.setZero(robotdata->ndof,1);
            robotdata->qd_bound.setZero(robotdata->ndof,1);
            robotdata->tau_bound.setZero(robotdata->ndof,1);
            robotdata->q_lbound = Eigen::MatrixXd::Ones(robotdata->ndof,1)*(-_INF);
            robotdata->q_ubound = Eigen::MatrixXd::Ones(robotdata->ndof,1)*(_INF);
            robotdata->qd_bound = Eigen::MatrixXd::Ones(robotdata->ndof,1)*(_INF);

//            robotdata->tau_bound = Eigen::MatrixXd::Ones(robotdata->ndof,1)*(0.0);

            robotdata->q_a.setZero(robotdata->ndof,1);
            robotdata->q_dot_a.setZero(robotdata->ndof,1);
            robotdata->q_ddot_a.setZero(robotdata->ndof,1);
            robotdata->tau_a.setZero(robotdata->ndof,1);

            robotdata->q_d.setZero(robotdata->ndof,1);
            robotdata->q_dot_d.setZero(robotdata->ndof,1);
            robotdata->q_ddot_d.setZero(robotdata->ndof,1);
            robotdata->tau_d.setZero(robotdata->ndof,1);

            robotdata->q_c.setZero(robotdata->ndof,1);
            robotdata->q_dot_c.setZero(robotdata->ndof,1);
            robotdata->q_ddot_c.setZero(robotdata->ndof,1);
            robotdata->tau_c.setZero(robotdata->ndof,1);

            // PD gains
            robotdata->q_factor.setZero(robotdata->ndof - 6,1);
            robotdata->q_dot_factor.setZero(robotdata->ndof - 6,1);
            _mass.resize(_ndof);
            _link_com.resize(_ndof);
            _link_inertial.resize(_ndof);
            for(int _link_int = 0; _link_int<_ndof;_link_int++)
            {
                _link_com[_link_int].setZero(3,1);
                _link_inertial[_link_int].setZero(3,3);
            }

            // read robot_linkArray
            it = object.begin();
            while(it != object.end())
            {
                if(it.key() == "robot_linkArray")
                {
                    if(robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
                    {
                        for(int i = 0; i < _ndof; i++)
                        {
                                QJsonObject _robot_link_array_object = it.value().toArray().at(i).toObject();
                                QJsonObject::iterator robot_link_array_object = _robot_link_array_object.begin();
//                                qDebug() << it.value().toArray().at(array_i).toObject().keys();
                                //get current id
                                int _current_id = -1;
                                while(robot_link_array_object!=_robot_link_array_object.end())
                                {
//                                    qDebug() << (*robot_link_array_object);
                                    if(robot_link_array_object.key() == "current_ID")
                                    {
                                        _current_id = robot_link_array_object.value().toInt();
                                    }
                                robot_link_array_object++;
                                }
                                if(_current_id == i+1)
                                {
//                                    _T_list[i].setIdentity();
                                    _No[i] = _current_id - 1;
                                    robot_link_array_object = _robot_link_array_object.begin();
                                    while(robot_link_array_object!=_robot_link_array_object.end())
                                    {
                                       if(robot_link_array_object.key() == "COM_x")
                                       {
                                           _link_com[i](0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "COM_y")
                                       {
                                           _link_com[i](1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "COM_z")
                                       {
                                           _link_com[i](2) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "mass")
                                       {
                                           _mass[i] = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xx")
                                       {
                                           _link_inertial[i](0,0) = robot_link_array_object.value().toDouble();
//                                           qDebug() << "Intertial: " <<  _link_inertial[i](0,0);
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xy")
                                       {
                                           _link_inertial[i](0,1) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](1,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xz")
                                       {
                                           _link_inertial[i](0,2) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](2,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_yy")
                                       {
                                           _link_inertial[i](1,1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_yz")
                                       {
                                           _link_inertial[i](1,2) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](2,1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_zz")
                                       {
                                           _link_inertial[i](2,2) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Parent_ID")
                                       {
                                           _parent[i] = robot_link_array_object.value().toInt() - 1;

                                       }

                                       if(robot_link_array_object.key() == "RotArray")
                                       {

                                           _T_list[i](0,0) = robot_link_array_object.value().toArray().at(0).toDouble();
                                           _T_list[i](0,1) = robot_link_array_object.value().toArray().at(1).toDouble();
                                           _T_list[i](0,2) = robot_link_array_object.value().toArray().at(2).toDouble();

                                           _T_list[i](1,0) = robot_link_array_object.value().toArray().at(3).toDouble();
                                           _T_list[i](1,1) = robot_link_array_object.value().toArray().at(4).toDouble();
                                           _T_list[i](1,2) = robot_link_array_object.value().toArray().at(5).toDouble();

                                           _T_list[i](2,0) = robot_link_array_object.value().toArray().at(6).toDouble();
                                           _T_list[i](2,1) = robot_link_array_object.value().toArray().at(7).toDouble();
                                           _T_list[i](2,2) = robot_link_array_object.value().toArray().at(8).toDouble();
                                       }

                                       if(robot_link_array_object.key() == "TransArray")
                                       {
                                           _T_list[i](0,3) = robot_link_array_object->toArray().at(0).toDouble();
                                           _T_list[i](1,3) = robot_link_array_object->toArray().at(1).toDouble();
                                           _T_list[i](2,3) = robot_link_array_object->toArray().at(2).toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Torque_Limit")
                                       {
                                           robotdata->tau_bound(i) = robot_link_array_object.value().toDouble();

                                       }

                                       if(robot_link_array_object.key() == "Joint_Speed_Limit")
                                       {
                                           robotdata->qd_bound(i) = robot_link_array_object.value().toDouble();

                                       }

                                       if(robot_link_array_object.key() == "Maxmum_Joint_position")
                                       {
                                           robotdata->q_ubound(i) = robot_link_array_object.value().toDouble();

                                       }

                                       if(robot_link_array_object.key() == "Minimum_Joint_position")
                                       {
                                           robotdata->q_lbound(i) = robot_link_array_object.value().toDouble();

                                       }

                                       if(robot_link_array_object.key() == "Rotation_Axis")
                                       {
                                           if(robot_link_array_object.value().toString() == "FloatJoint")
                                           {
                                               _axis[i] = 0;
                                           }else if(robot_link_array_object.value().toString() == "+x")
                                           {
                                               _axis[i] = 1;
                                           }else if(robot_link_array_object.value().toString() == "-x")
                                           {
                                               _axis[i] = 2;
                                           }else if(robot_link_array_object.value().toString() == "+y")
                                           {
                                               _axis[i] = 3;
                                           }else if(robot_link_array_object.value().toString() == "-y")
                                           {
                                               _axis[i] = 4;
                                           }else if(robot_link_array_object.value().toString() == "+z")
                                           {
                                               _axis[i] = 5;
                                           }else if(robot_link_array_object.value().toString() == "-z")
                                           {
                                               _axis[i] = 6;
                                           }else{
                                               _axis[i] = -1;
                                           }
                                       }

                                       robot_link_array_object++;
                                    }

//                            robot_link_array++;
                                }

                        }
                    }else if(robotdata->robottype == robot_type::Float_Base_Open_Chain)
                    {
                        for(int i = 0; i < _ndof; i++)
                        {
                                QJsonObject _robot_link_array_object = it.value().toArray().at(i).toObject();
                                QJsonObject::iterator robot_link_array_object = _robot_link_array_object.begin();
                                //get current id
                                int _current_id = -2;
                                while(robot_link_array_object!=_robot_link_array_object.end())
                                {
                                    if(robot_link_array_object.key() == "current_ID")
                                    {
                                        _current_id = robot_link_array_object.value().toInt();
                                    }
                                robot_link_array_object++;
                                }
                                if(_current_id == i)
                                {
                                    _T_list[i].setIdentity();
                                    _No[i] = _current_id;
                                    robot_link_array_object = _robot_link_array_object.begin();
                                    while(robot_link_array_object!=_robot_link_array_object.end())
                                    {
                                       if(robot_link_array_object.key() == "COM_x")
                                       {
                                           _link_com[i](0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "COM_y")
                                       {
                                           _link_com[i](1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "COM_z")
                                       {
                                           _link_com[i](2) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "mass")
                                       {
                                           _mass[i] = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xx")
                                       {
                                           _link_inertial[i](0,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xy")
                                       {
                                           _link_inertial[i](0,1) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](1,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xz")
                                       {
                                           _link_inertial[i](0,2) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](2,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_yy")
                                       {
                                           _link_inertial[i](1,1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_yz")
                                       {
                                           _link_inertial[i](1,2) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](2,1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_zz")
                                       {
                                           _link_inertial[i](2,2) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Parent_ID")
                                       {
                                           if(_No[i] == 0)
                                           {
                                               _parent[i] = robot_link_array_object.value().toInt() - 1;
                                           }else{
                                               _parent[i] = robot_link_array_object.value().toInt();
                                           }
                                       }

                                       if(robot_link_array_object.key() == "RotArray")
                                       {


                                           _T_list[i](0,0) = robot_link_array_object.value().toArray().at(0).toDouble();
                                           _T_list[i](0,1) = robot_link_array_object.value().toArray().at(1).toDouble();
                                           _T_list[i](0,2) = robot_link_array_object.value().toArray().at(2).toDouble();

                                           _T_list[i](1,0) = robot_link_array_object.value().toArray().at(3).toDouble();
                                           _T_list[i](1,1) = robot_link_array_object.value().toArray().at(4).toDouble();
                                           _T_list[i](1,2) = robot_link_array_object.value().toArray().at(5).toDouble();

                                           _T_list[i](2,0) = robot_link_array_object.value().toArray().at(6).toDouble();
                                           _T_list[i](2,1) = robot_link_array_object.value().toArray().at(7).toDouble();
                                           _T_list[i](2,2) = robot_link_array_object.value().toArray().at(8).toDouble();
                                       }

                                       if(robot_link_array_object.key() == "TransArray")
                                       {
                                           _T_list[i](0,3) = robot_link_array_object.value().toArray().at(0).toDouble();
                                           _T_list[i](1,3) = robot_link_array_object.value().toArray().at(1).toDouble();
                                           _T_list[i](2,3) = robot_link_array_object.value().toArray().at(2).toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Rotation_Axis")
                                       {
                                           if(robot_link_array_object.value().toString() == "FloatJoint")
                                           {
                                               _axis[i] = 0;
                                           }else if(robot_link_array_object.value().toString() == "+x")
                                           {
                                               _axis[i] = 1;
                                           }else if(robot_link_array_object.value().toString() == "-x")
                                           {
                                               _axis[i] = 2;
                                           }else if(robot_link_array_object.value().toString() == "+y")
                                           {
                                               _axis[i] = 3;
                                           }else if(robot_link_array_object.value().toString() == "-y")
                                           {
                                               _axis[i] = 4;
                                           }else if(robot_link_array_object.value().toString() == "+z")
                                           {
                                               _axis[i] = 5;
                                           }else if(robot_link_array_object.value().toString() == "-z")
                                           {
                                               _axis[i] = 6;
                                           }else{
                                               _axis[i] = -1;
                                           }
                                       }

                                       if(_No[i]!=0)
                                       {
                                           if(robot_link_array_object.key() == "Torque_Limit")
                                           {
                                                robotdata->tau_bound(i+5) = robot_link_array_object.value().toDouble();
                                           }

                                           if(robot_link_array_object.key() == "Joint_Speed_Limit")
                                           {
                                               robotdata->qd_bound(i+5) = robot_link_array_object.value().toDouble();
                                           }

                                           if(robot_link_array_object.key() == "Maxmum_Joint_position")
                                           {
                                               robotdata->q_ubound(i+5) = robot_link_array_object.value().toDouble();

                                           }

                                           if(robot_link_array_object.key() == "Minimum_Joint_position")
                                           {
                                               robotdata->q_lbound(i+5) = robot_link_array_object.value().toDouble();

                                           }
                                       }
                                    robot_link_array_object++;
                                    }
                                }

                        }

                    }else if(robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain){
                        for(int i = 0; i < _ndof; i++)
                        {
                                QJsonObject _robot_link_array_object = it.value().toArray().at(i).toObject();
                                QJsonObject::iterator robot_link_array_object = _robot_link_array_object.begin();
                                //get current id
                                int _current_id = -2;
                                while(robot_link_array_object!=_robot_link_array_object.end())
                                {
                                    if(robot_link_array_object.key() == "current_ID")
                                    {
                                        _current_id = robot_link_array_object.value().toInt();
                                    }
                                robot_link_array_object++;
                                }
                                if(_current_id == i)
                                {
                                    _T_list[i].setIdentity();
                                    _No[i] = _current_id;
                                    robot_link_array_object = _robot_link_array_object.begin();
                                    while(robot_link_array_object!=_robot_link_array_object.end())
                                    {
                                       if(robot_link_array_object.key() == "COM_x")
                                       {
                                           _link_com[i](0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "COM_y")
                                       {
                                           _link_com[i](1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "COM_z")
                                       {
                                           _link_com[i](2) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "mass")
                                       {
                                           _mass[i] = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xx")
                                       {
                                           _link_inertial[i](0,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xy")
                                       {
                                           _link_inertial[i](0,1) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](1,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_xz")
                                       {
                                           _link_inertial[i](0,2) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](2,0) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_yy")
                                       {
                                           _link_inertial[i](1,1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_yz")
                                       {
                                           _link_inertial[i](1,2) = robot_link_array_object.value().toDouble();
                                           _link_inertial[i](2,1) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Inertia_zz")
                                       {
                                           _link_inertial[i](2,2) = robot_link_array_object.value().toDouble();
                                       }

                                       if(robot_link_array_object.key() == "Parent_ID")
                                       {
                                           if(_No[i] == 0)
                                           {
                                               _parent[i] = robot_link_array_object.value().toInt() - 1;
                                           }else{
                                               _parent[i] = robot_link_array_object.value().toInt();
                                           }
                                       }

                                       if(robot_link_array_object.key() == "RotArray")
                                       {


                                           _T_list[i](0,0) = robot_link_array_object->toArray().at(0).toDouble();
                                           _T_list[i](0,1) = robot_link_array_object->toArray().at(1).toDouble();
                                           _T_list[i](0,2) = robot_link_array_object->toArray().at(2).toDouble();

                                           _T_list[i](1,0) = robot_link_array_object->toArray().at(3).toDouble();
                                           _T_list[i](1,1) = robot_link_array_object->toArray().at(4).toDouble();
                                           _T_list[i](1,2) = robot_link_array_object->toArray().at(5).toDouble();

                                           _T_list[i](2,0) = robot_link_array_object->toArray().at(6).toDouble();
                                           _T_list[i](2,1) = robot_link_array_object->toArray().at(7).toDouble();
                                           _T_list[i](2,2) = robot_link_array_object->toArray().at(8).toDouble();
                                       }

                                       if(robot_link_array_object.key() == "TransArray")
                                       {
                                           _T_list[i](0,3) = robot_link_array_object->toArray().at(0).toDouble();
                                           _T_list[i](1,3) = robot_link_array_object->toArray().at(1).toDouble();
                                           _T_list[i](2,3) = robot_link_array_object->toArray().at(2).toDouble();
                                       }
                                       if(robot_link_array_object.key() == "Rotation_Axis")
                                       {
                                           if(robot_link_array_object.value().toString() == "FloatJoint")
                                           {
                                               _axis[i] = 0;
                                           }else if(robot_link_array_object.value().toString() == "+x")
                                           {
                                               _axis[i] = 1;
                                           }else if(robot_link_array_object.value().toString() == "-x")
                                           {
                                               _axis[i] = 2;
                                           }else if(robot_link_array_object.value().toString() == "+y")
                                           {
                                               _axis[i] = 3;
                                           }else if(robot_link_array_object.value().toString() == "-y")
                                           {
                                               _axis[i] = 4;
                                           }else if(robot_link_array_object.value().toString() == "+z")
                                           {
                                               _axis[i] = 5;
                                           }else if(robot_link_array_object.value().toString() == "-z")
                                           {
                                               _axis[i] = 6;
                                           }else{
                                               _axis[i] = -1;
                                           }
                                       }

                                       if(_No[i]!=0)
                                       {
                                           if(robot_link_array_object.key() == "Torque_Limit")
                                           {
                                                    robotdata->tau_bound(i+5) = robot_link_array_object.value().toDouble();
                                           }

                                           if(robot_link_array_object.key() == "Joint_Speed_Limit")
                                           {
                                               robotdata->qd_bound(i+5) = robot_link_array_object.value().toDouble();
                                           }

                                           if(robot_link_array_object.key() == "Maxmum_Joint_position")
                                           {
                                               robotdata->q_ubound(i+5) = robot_link_array_object.value().toDouble();

                                           }

                                           if(robot_link_array_object.key() == "Minimum_Joint_position")
                                           {
                                               robotdata->q_lbound(i+5) = robot_link_array_object.value().toDouble();

                                           }
                                       }

                                       if(robot_link_array_object.key() == "wheel_radius")
                                       {
                                           if(robot_link_array_object.value().toDouble() > 1.0e-3)
                                           {
                                               robotdata->radius_wheel = robot_link_array_object.value().toDouble();
                                               _ndof_wheel = _ndof_wheel + 1;
                                           }
                                       }
                                    robot_link_array_object++;
                                    }
                                }
                        }
                    }
                }
                qDebug() <<"it: " <<it.key();
                it++;
            }
            // construct the rbdl robot model
            robotdata->robot_model = new RigidBodyDynamics::Model();
            robotdata->robot_model->gravity = _gravity;
            // link com and link inertial
            RigidBodyDynamics::Math::Vector3d _link_com_rbdl;
            RigidBodyDynamics::Math::Matrix3d _link_inertial_rbdl;
            switch(robotdata->robottype)
            {
            case Float_Base_Open_Chain:
            {
                // floating base joint
                // RigidBodyDynamics::Joint * _floating_joint2 = new RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFloatingBase);
                RigidBodyDynamics::Joint * _floating_joint = new RigidBodyDynamics::Joint(RigidBodyDynamics::Math::SpatialVector(0.,0.,0., 1.,0.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,0.,0., 0.,1.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,0.,0., 0.,0.,1.),
                                                                                        //   RigidBodyDynamics::Math::SpatialVector(0.,0.,1., 0.,0.,0.),
                                                                                        //   RigidBodyDynamics::Math::SpatialVector(0.,1.,0., 0.,0.,0.),
                                                                                        //   RigidBodyDynamics::Math::SpatialVector(1.,0.,0., 0.,0.,0.));                        
                                                                                          RigidBodyDynamics::Math::SpatialVector(1.,0.,0., 0.,0.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,1.,0., 0.,0.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,0.,1., 0.,0.,0.));
                _link_com_rbdl = _link_com[0];
                _link_inertial_rbdl = _link_inertial[0];
                RigidBodyDynamics::Body * _floating_body = new RigidBodyDynamics::Body(_mass[0],_link_com_rbdl,_link_inertial_rbdl);
                // spatial transformation
                Eigen::Matrix3d R = _T_list[0].block(0,0,3,3).transpose();
                Eigen::Vector3d r = _T_list[0].block(0,3,3,1);
                RigidBodyDynamics::Math::SpatialTransform X(R,r);
                // link to the base
                robotdata->id_body[0] = robotdata->robot_model->AddBody(0, X, *_floating_joint, *_floating_body);
                robotdata->robot_body[0] = _floating_body;
                robotdata->robot_joint[0] = _floating_joint;
                // construct the Open_Chain
                for(int i = 1;i<_ndof;i++)
                {
                    RigidBodyDynamics::Math::Vector3d Axis;
                    Axis << 0.,0.,1.;
                    if(_axis[i] == 1)
                    {
                        Axis << 1.,0.,0.;
                    }else if(_axis[i] == 2)
                    {
                        Axis << -1.,0.,0.;
                    }else if(_axis[i] == 3)
                    {
                        Axis << 0.,1.,0.;
                    }else if(_axis[i] == 4)
                    {
                        Axis << 0.,-1.,0.;
                    }else if(_axis[i] == 5)
                    {
                        Axis << 0.,0.,1.;
                    }else if(_axis[i] == 6)
                    {
                        Axis << 0.,0.,-1.;
                    }else{
                        std::cout<<"no matching joint axis!"<<std::endl;
                    }
                    RigidBodyDynamics::Joint * _joint = new RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,Axis);
                    _link_com_rbdl = _link_com[i];
                    _link_inertial_rbdl = _link_inertial[i];
                    RigidBodyDynamics::Body * _body = new RigidBodyDynamics::Body(_mass[_No[i]],_link_com_rbdl,_link_inertial_rbdl);
                    //detemine later
                    Eigen::Matrix3d R = _T_list[i].block(0,0,3,3).transpose();
                    Eigen::Vector3d r = _T_list[i].block(0,3,3,1);
                    RigidBodyDynamics::Math::SpatialTransform X(R,r);
                    //link to the base
                    if(_parent[i] == -1)
                    {
                        robotdata->id_body[_No[i]] = robotdata->robot_model->AddBody(0,X,*_joint,*_body);
                    }else{
                        robotdata->id_body[_No[i]] = robotdata->robot_model->AddBody(robotdata->id_body[_parent[i]],X,*_joint,*_body);
                    }
                    robotdata->robot_body[_No[i]] = _body;
                    robotdata->robot_joint[_No[i]] = _joint;
                }
                break;
            }

            case Fixed_Base_Open_Chain:
            {
                // construct the Open_Chain
                for(int i = 0;i<_ndof;i++)
                {
                    RigidBodyDynamics::Math::Vector3d Axis;
                    Axis << 0.,0.,1.;
                    if(_axis[i] == 1)
                    {
                        Axis << 1.,0.,0.;
                    }else if(_axis[i] == 2)
                    {
                        Axis << -1.,0.,0.;
                    }else if(_axis[i] == 3)
                    {
                        Axis << 0.,1.,0.;
                    }else if(_axis[i] == 4)
                    {
                        Axis << 0.,-1.,0.;
                    }else if(_axis[i] == 5)
                    {
                        Axis << 0.,0.,1.;
                    }else if(_axis[i] == 6)
                    {
                        Axis << 0.,0.,-1.;
                    }else{
                        std::cout<<"no matching joint axis!"<<std::endl;
                    }
                    RigidBodyDynamics::Joint * _joint = new RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,Axis);
                    _link_com_rbdl = _link_com[i];
                    _link_inertial_rbdl = _link_inertial[i];
//                    qDebug() << _link_inertial[i](0,0);
                    RigidBodyDynamics::Body * _body = new RigidBodyDynamics::Body(_mass[_No[i]],_link_com_rbdl,_link_inertial_rbdl);
                    //detemine later

                    Eigen::Matrix3d R = _T_list[i].block(0,0,3,3).transpose();
                    Eigen::Vector3d r = _T_list[i].block(0,3,3,1);
                    RigidBodyDynamics::Math::SpatialTransform X(R,r);
                    //link to the base
                    if(_parent[i] == -1)
                    {
                        robotdata->id_body[_No[i]] = robotdata->robot_model->AddBody(0,X,*_joint,*_body);

                    }else{
                        robotdata->id_body[_No[i]] = robotdata->robot_model->AddBody(robotdata->id_body[_parent[i]],X,*_joint,*_body);
                    }
                    robotdata->robot_body[_No[i]] = _body;
                    robotdata->robot_joint[_No[i]] = _joint;
                }
                break;
            }
            case Mobile_Wheel_Open_Chain:
            {
                // floating base joint
                RigidBodyDynamics::Joint * _floating_joint = new RigidBodyDynamics::Joint(RigidBodyDynamics::Math::SpatialVector(0.,0.,0., 1.,0.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,0.,0., 0.,1.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,0.,0., 0.,0.,1.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(1.,0.,0., 0.,0.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,1.,0., 0.,0.,0.),
                                                                                          RigidBodyDynamics::Math::SpatialVector(0.,0.,1., 0.,0.,0.));
                _link_com_rbdl = _link_com[0];
                _link_inertial_rbdl = _link_inertial[0];
                RigidBodyDynamics::Body * _floating_body = new RigidBodyDynamics::Body(_mass[0],_link_com_rbdl,_link_inertial_rbdl);
                // spatial transformation
                Eigen::Matrix3d R = _T_list[0].block(0,0,3,3).transpose();
                Eigen::Vector3d r = _T_list[0].block(0,3,3,1);
                RigidBodyDynamics::Math::SpatialTransform X(R,r);
                // link to the base
                robotdata->id_body[0] = robotdata->robot_model->AddBody(0, X, *_floating_joint, *_floating_body);
                robotdata->robot_body[0] = _floating_body;
                robotdata->robot_joint[0] = _floating_joint;
                // construct the Open_Chain
                for(int i = 1;i<_ndof;i++)
                {
                    RigidBodyDynamics::Math::Vector3d Axis;
                    Axis << 0.,0.,1.;
                    if(_axis[i] == 1)
                    {
                        Axis << 1.,0.,0.;
                    }else if(_axis[i] == 2)
                    {
                        Axis << -1.,0.,0.;
                    }else if(_axis[i] == 3)
                    {
                        Axis << 0.,1.,0.;
                    }else if(_axis[i] == 4)
                    {
                        Axis << 0.,-1.,0.;
                    }else if(_axis[i] == 5)
                    {
                        Axis << 0.,0.,1.;
                    }else if(_axis[i] == 6)
                    {
                        Axis << 0.,0.,-1.;
                    }else{
                        std::cout<<"no matching joint axis!"<<std::endl;
                    }
                    RigidBodyDynamics::Joint * _joint = new RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,Axis);
                    _link_com_rbdl = _link_com[i];
                    _link_inertial_rbdl = _link_inertial[i];
                    RigidBodyDynamics::Body * _body = new RigidBodyDynamics::Body(_mass[_No[i]],_link_com_rbdl,_link_inertial_rbdl);
                    //detemine later
                    Eigen::Matrix3d R = _T_list[i].block(0,0,3,3).transpose();
                    Eigen::Vector3d r = _T_list[i].block(0,3,3,1);
                    RigidBodyDynamics::Math::SpatialTransform X(R,r);
                    //link to the base
                    if(_parent[i] == -1)
                    {
                        robotdata->id_body[_No[i]] = robotdata->robot_model->AddBody(0,X,*_joint,*_body);
                    }else{
                        robotdata->id_body[_No[i]] = robotdata->robot_model->AddBody(robotdata->id_body[_parent[i]],X,*_joint,*_body);
                    }
                    robotdata->robot_body[_No[i]] = _body;
                    robotdata->robot_joint[_No[i]] = _joint;
                }
                // wheel set
                robotdata->ndof_wheel = _ndof_wheel;
                robotdata->length_wheel = (_T_list[1].block(0,3,3,1) - _T_list[2].block(0,3,3,1)).norm();
                break;
            }

            default:
            {
                break;
            }
            }

            // read sensor array and task array
            it = object.begin();
            while(it!=object.end())
            {

                if(it.key() == "sensorArray")
                {
                    QJsonArray _sensor_array = it.value().toArray();
//                    QJsonArray::iterator _sensor_array = it_sensor_array.begin();
//                    int i =0;
                    for(int i = 0; i< _sensor_array.size();i++)
                    {
                        Sensor* _p_sensor = new Sensor();
                        _p_sensor->T_offset.setIdentity();
                        QJsonObject t_sensor_arrray_object = _sensor_array.at(i).toObject();
                        QJsonObject::iterator _sensor_arrray_object = t_sensor_arrray_object.begin();
                        while(_sensor_arrray_object != t_sensor_arrray_object.end())
                        {
                            if(_sensor_arrray_object.key() == "current_ID")
                            {
                                _p_sensor->ID = _sensor_arrray_object.value().toInt();
                            }

                            if(_sensor_arrray_object.key() == "sensor_type")
                            {
                                if(_sensor_arrray_object.value().toString() == "FTSensor")
                                {
                                    _p_sensor->type = sensor_type::FT_sensor;
                                    _p_sensor->_data.setZero(6,1);
                                }else if(_sensor_arrray_object.value().toString() == "IMU")
                                {
                                    _p_sensor->type = sensor_type::Imu_sensor;
                                    _p_sensor->_data.setZero(18,1);
                                }else{
                                    std::cout<<"no such sensor type!"<<std::endl;
                                    return;
                                }

                            }

                            if(_sensor_arrray_object.key() == "Link")
                            {
                                int _joint_id;
                                if(robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
                                {
                                    _joint_id = _sensor_arrray_object.value().toInt() - 1;
                                    _p_sensor->link_num = robotdata->id_body[_joint_id];
                                }else if(robotdata->robottype == robot_type::Float_Base_Open_Chain || robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
                                {
                                    _joint_id = _sensor_arrray_object.value().toInt();
                                    _p_sensor->link_num = robotdata->id_body[_joint_id];
                                }else{
                                    std::cout<<"no such robottype match the sensor!"<<std::endl;
                                    return;
                                }
                            }

                            if(_sensor_arrray_object.key() == "RotArray")
                            {
                                _p_sensor->T_offset(0,0) = _sensor_arrray_object->toArray().at(0).toDouble();
                                _p_sensor->T_offset(0,1) = _sensor_arrray_object->toArray().at(1).toDouble();
                                _p_sensor->T_offset(0,2) = _sensor_arrray_object->toArray().at(2).toDouble();

                                _p_sensor->T_offset(1,0) = _sensor_arrray_object->toArray().at(3).toDouble();
                                _p_sensor->T_offset(1,1) = _sensor_arrray_object->toArray().at(4).toDouble();
                                _p_sensor->T_offset(1,2) = _sensor_arrray_object->toArray().at(5).toDouble();

                                _p_sensor->T_offset(2,0) = _sensor_arrray_object->toArray().at(6).toDouble();
                                _p_sensor->T_offset(2,1) = _sensor_arrray_object->toArray().at(7).toDouble();
                                _p_sensor->T_offset(2,2) = _sensor_arrray_object->toArray().at(8).toDouble();
                            }

                            if(_sensor_arrray_object.key() == "TransArray")
                            {
                                _p_sensor->T_offset(0,3) = _sensor_arrray_object->toArray().at(0).toDouble();
                                _p_sensor->T_offset(1,3) = _sensor_arrray_object->toArray().at(1).toDouble();
                                _p_sensor->T_offset(2,3) = _sensor_arrray_object->toArray().at(2).toDouble();
                            }
                            _sensor_arrray_object++;
                        }
                        robotdata->sensor_set[i] = _p_sensor;
//                        _sensor_array++;
                    }
                    Sensor* _p_sensor_tau_ext = new Sensor();
                    _p_sensor_tau_ext->ID = _nsensor+1;
                    _p_sensor_tau_ext->type = sensor_type::Joint_external_torque_observer;
                    _p_sensor_tau_ext->_data.setZero(robotdata->ndof,1);
                    _p_sensor_tau_ext->T_offset.setIdentity();
                    _p_sensor_tau_ext->link_num = 0;
                    robotdata->sensor_set.push_back(_p_sensor_tau_ext);

                }

                if(it.key() == "taskArray")
                {
                    QJsonArray _task_array = it.value().toArray();
                    for(int i = 0; i<_task_array.size();i++)
                    {
                        Task* _p_task = new Task();
                        robotdata->task_card_set[i] = _p_task;
                        robotdata->task_card_set[i]->T_offset.setIdentity();
                        QJsonObject t_task_array_object = _task_array.at(i).toObject();
                        QJsonObject::iterator _task_array_object = t_task_array_object.begin();
                        // update task type and dimesions of the variables
                        while(_task_array_object != t_task_array_object.end())
                        {
                            //task direction
                            if(_task_array_object.key() == "target_theta_x")
                            {
                                _task_direction_selection[0] = _task_array_object.value().toBool();
                            }

                            if(_task_array_object.key() == "target_theta_y")
                            {
                                _task_direction_selection[1] = _task_array_object.value().toBool();
                            }

                            if(_task_array_object.key() == "target_theta_z")
                            {
                                _task_direction_selection[2] = _task_array_object.value().toBool();
                            }

                            if(_task_array_object.key() == "target_x")
                            {
                                _task_direction_selection[3] = _task_array_object.value().toBool();
                            }

                            if(_task_array_object.key() == "target_y")
                            {
                                _task_direction_selection[4] = _task_array_object.value().toBool();
                            }

                            if(_task_array_object.key() == "target_z")
                            {
                                _task_direction_selection[5] = _task_array_object.value().toBool();
                            }

                            if(_task_array_object.key() == "weight_theta_x")
                            {
                                _task_weight[0] = _task_array_object.value().toDouble();
                            }

                            if(_task_array_object.key() == "weight_theta_y")
                            {
                                _task_weight[1] = _task_array_object.value().toDouble();
                            }

                            if(_task_array_object.key() == "weight_theta_z")
                            {
                                _task_weight[2] = _task_array_object.value().toDouble();
                            }

                            if(_task_array_object.key() == "weight_x")
                            {
                                _task_weight[3] = _task_array_object.value().toDouble();
                            }

                            if(_task_array_object.key() == "weight_y")
                            {
                                _task_weight[4] = _task_array_object.value().toDouble();
                            }

                            if(_task_array_object.key() == "weight_z")
                            {
                                _task_weight[5] = _task_array_object.value().toDouble();
                            }

                            if(_task_array_object.key() == "task_type")
                            {
                                if(_task_array_object.value().toString() == "一般任务")
                                {
                                    robotdata->task_card_set[i]->type = task_type::general_task;
                                }else if(_task_array_object.value().toString() == "质心任务")
                                {
                                    robotdata->task_card_set[i]->type = task_type::com_task;
                                }else if(_task_array_object.value().toString() == "接触任务")
                                {
                                    robotdata->task_card_set[i]->type = task_type::contact_task;
                                }else if(_task_array_object.value().toString() == "关节空间任务")
                                {
                                    robotdata->task_card_set[i]->type = task_type::joint_task;
                                }else{
                                    std::cout<<"no such type task!"<<std::endl;
                                    return;
                                }
                            }
                            // record biped robot task
                            if(_task_array_object.key() == "taskname")
                            {
                                if(_task_array_object.value().toString() == "left_foot_contact_task")
                                {
                                    robotdata->left_foot_id = i;
                                }

                                if(_task_array_object.value().toString() == "right_foot_contact_task")
                                {
                                    robotdata->right_foot_id = i;
                                }

                                if(_task_array_object.value().toString() == "floatingbase_orientation")
                                {
                                    robotdata->body_task_id = i;
                                }

                                if(_task_array_object.value().toString() == "com_task")
                                {
                                    robotdata->com_task_id = i;
                                }
                            }
                            // end
                            _task_array_object++;
                        }

                        if(robotdata->task_card_set[i]->type == task_type::joint_task)
                        {
                           if(robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
                           {
                                robotdata->task_card_set[i]->dim = robotdata->ndof;
                                robotdata->task_card_set[i]->jacobi = Eigen::MatrixXd::Identity(robotdata->ndof,robotdata->ndof);
                                robotdata->task_card_set[i]->weight = Eigen::MatrixXd::Ones(robotdata->ndof,1);
                                robotdata->task_card_set[i]->jacobi_dot_q_dot = Eigen::MatrixXd::Zero(robotdata->ndof,1);
                           }
                                else if((robotdata->robottype == robot_type::Float_Base_Open_Chain)||(robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain))
                            {
                                int _dim =  robotdata->ndof - 6;
                                robotdata->task_card_set[i]->dim =_dim;
                                robotdata->task_card_set[i]->jacobi = Eigen::MatrixXd::Zero(_dim,robotdata->ndof);
                                robotdata->task_card_set[i]->jacobi.block(0,6,_dim,_dim) = Eigen::MatrixXd::Identity(_dim,_dim);
                                robotdata->task_card_set[i]->weight = Eigen::MatrixXd::Ones(_dim,1);
                                robotdata->task_card_set[i]->jacobi_dot_q_dot = Eigen::MatrixXd::Zero(_dim,1);
                            }

                            robotdata->task_card_set[i]->contact_state_d = false;
                            robotdata->task_card_set[i]->X_a.setZero(4,robotdata->task_card_set[i]->dim);
                            robotdata->task_card_set[i]->X_d.setZero(4,robotdata->task_card_set[i]->dim);
                            robotdata->task_card_set[i]->X_c.setZero(4,robotdata->task_card_set[i]->dim);

                            robotdata->task_card_set[i]->T_offset.setIdentity();
                            robotdata->task_card_set[i]->IG.setIdentity();
                        }else{
                            robotdata->task_card_set[i]->dim = 0;

                            if(_task_direction_selection[0] == true)
                            {
                                robotdata->task_card_set[i]->dim += 1;
                                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_x_theta);
                            }

                            if(_task_direction_selection[1] == true)
                            {
                                robotdata->task_card_set[i]->dim += 1;
                                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_y_theta);
                            }

                            if(_task_direction_selection[2] == true)
                            {
                                robotdata->task_card_set[i]->dim += 1;
                                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_z_theta);
                            }

                            if(_task_direction_selection[3] == true)
                            {
                                robotdata->task_card_set[i]->dim += 1;
                                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_x);
                            }

                            if(_task_direction_selection[4] == true)
                            {
                                robotdata->task_card_set[i]->dim += 1;
                                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_y);
                            }

                            if(_task_direction_selection[5] == true)
                            {
                                robotdata->task_card_set[i]->dim += 1;
                                robotdata->task_card_set[i]->task_selection_matrix.push_back(task_direction::task_z);
                            }

                            robotdata->task_card_set[i]->weight.setOnes(robotdata->task_card_set[i]->dim,1);
                            int count = 0;
                            for(int j = 0; j < 6; j++)
                            {
                                if(_task_direction_selection[j] == true)
                                {
                                    robotdata->task_card_set[i]->weight(count) = _task_weight[j];
                                    count++;
                                }
                            }

                            robotdata->task_card_set[i]->contact_state_d = false;
                            robotdata->task_card_set[i]->X_a.setZero(4,robotdata->task_card_set[i]->dim);
                            robotdata->task_card_set[i]->X_d.setZero(4,robotdata->task_card_set[i]->dim);
                            robotdata->task_card_set[i]->X_c.setZero(4,robotdata->task_card_set[i]->dim);

                            robotdata->task_card_set[i]->jacobi.setZero(robotdata->task_card_set[i]->dim,robotdata->ndof);
                            robotdata->task_card_set[i]->jacobi_dot_q_dot.setZero(robotdata->task_card_set[i]->dim,1);

                            robotdata->task_card_set[i]->T_offset.setIdentity();
                            robotdata->task_card_set[i]->IG.setIdentity();
                        }

                        // update task
                        _task_array_object = t_task_array_object.begin();
                        while(_task_array_object!=t_task_array_object.end())
                        {
                            if(_task_array_object.key() == "task_level")
                            {
                                robotdata->task_card_set[i]->priority = _task_array_object.value().toInt();
                            }

                            if(_task_array_object.key() == "task_number")
                            {
                                robotdata->task_card_set[i]->task_id = _task_array_object.value().toInt();
                            }

                            if(_task_array_object.key() == "control_target")
                            {
                                if(_task_array_object.value().toString() == "world")
                                {
                                    robotdata->task_card_set[i]->frame = 1;
                                }else if(_task_array_object.value().toString() == "local")
                                {
                                    robotdata->task_card_set[i]->frame = 0;
                                }else{
                                    std::cout<<"no such wold frame!"<<std::endl;
                                    return;
                                }
                            }

                            if(_task_array_object.key() == "RotArray")
                            {
                                robotdata->task_card_set[i]->T_offset(0,0) = _task_array_object.value().toArray().at(0).toDouble();
                                robotdata->task_card_set[i]->T_offset(0,1) = _task_array_object.value().toArray().at(1).toDouble();
                                robotdata->task_card_set[i]->T_offset(0,2) = _task_array_object.value().toArray().at(2).toDouble();

                                robotdata->task_card_set[i]->T_offset(1,0) = _task_array_object.value().toArray().at(3).toDouble();
                                robotdata->task_card_set[i]->T_offset(1,1) = _task_array_object.value().toArray().at(4).toDouble();
                                robotdata->task_card_set[i]->T_offset(1,2) = _task_array_object.value().toArray().at(5).toDouble();

                                robotdata->task_card_set[i]->T_offset(2,0) = _task_array_object.value().toArray().at(6).toDouble();
                                robotdata->task_card_set[i]->T_offset(2,1) = _task_array_object.value().toArray().at(7).toDouble();
                                robotdata->task_card_set[i]->T_offset(2,2) = _task_array_object.value().toArray().at(8).toDouble();
                            }

                            if(_task_array_object.key() == "TransArray")
                            {
                                robotdata->task_card_set[i]->T_offset(0,3) = _task_array_object.value().toArray().at(0).toDouble();
                                robotdata->task_card_set[i]->T_offset(1,3) = _task_array_object.value().toArray().at(1).toDouble();
                                robotdata->task_card_set[i]->T_offset(2,3) = _task_array_object.value().toArray().at(2).toDouble();
                            }

                            if(_task_array_object.key() == "solid_num")
                            {
                                int _joint_num = _task_array_object.value().toInt();
                                if(robotdata->robottype == robot_type::Fixed_Base_Open_Chain)
                                {
                                    robotdata->task_card_set[i]->joint_id = robotdata->id_body[_joint_num - 1];
                                }else if(robotdata->robottype == robot_type::Float_Base_Open_Chain || robotdata->robottype == robot_type::Mobile_Wheel_Open_Chain)
                                {
                                    robotdata->task_card_set[i]->joint_id = robotdata->id_body[_joint_num];
                                }else
                                {
                                    std::cout<<"no matching robot type!"<<std::endl;
                                    return;
                                }
                            }

                            _task_array_object++;
                        }

                        // update task controller and constraints
                        _task_array_object = t_task_array_object.begin();
                        while(_task_array_object != t_task_array_object.end())
                        {
                            if(_task_array_object.key() == "controller_para")
                            {
                                robotdata->task_card_set[i]->controller = new Controller_Lib();
                                robotdata->task_card_set[i]->controller->dim = robotdata->task_card_set[i]->dim;

                                QJsonObject t_task_array_object_object = _task_array_object.value().toObject();
                                QJsonObject::iterator _task_array_object_object = t_task_array_object_object.begin();
                                // controller type
                                while(_task_array_object_object != t_task_array_object_object.end())
                                {
                                    if(_task_array_object_object.key() == "controller_type")
                                    {
                                        if(_task_array_object_object.value().toString() == "导纳控制器")
                                        {
                                            robotdata->task_card_set[i]->controller->controller_type = basic_controller::Admittance;
                                        }else if(_task_array_object_object.value().toString() == "阻抗控制器")
                                        {
                                            robotdata->task_card_set[i]->controller->controller_type = basic_controller::Impedance;
                                        }else if(_task_array_object_object.value().toString() == "PID控制器")
                                        {
                                            if(robotdata->task_card_set[i]->task_selection_matrix[0] == task_direction::task_x_theta){
                                                robotdata->task_card_set[i]->controller->controller_type = basic_controller::PID_orient;
                                            }else{
                                                robotdata->task_card_set[i]->controller->controller_type = basic_controller::PID;
                                            }
                                            
                                        }else if(_task_array_object_object.value().toString() == "无")
                                        {
                                            robotdata->task_card_set[i]->controller->controller_type = basic_controller::None;
                                        }
                                    }
                                    _task_array_object_object++;
                                }
                                // controller para
                                robotdata->task_card_set[i]->controller->para.setZero(12,1);
                                _task_array_object_object = t_task_array_object_object.begin();
                                while(_task_array_object_object != t_task_array_object_object.end())
                                {
                                    if(robotdata->task_card_set[i]->controller->controller_type == basic_controller::Admittance)
                                    {
                                        if(_task_array_object_object.key() == "K")
                                        {
                                            robotdata->task_card_set[i]->controller->para(0,0) = _task_array_object_object.value().toDouble();
                                        }

                                        if(_task_array_object_object.key() == "M")
                                        {
                                            robotdata->task_card_set[i]->controller->para(1,0) = _task_array_object_object.value().toDouble();
                                        }

                                        if(_task_array_object_object.key() == "B")
                                        {
                                            robotdata->task_card_set[i]->controller->para(2,0) = _task_array_object_object.value().toDouble();
                                        }
                                        robotdata->task_card_set[i]->controller->alter_v.setZero(2,robotdata->task_card_set[i]->controller->dim);
                                        robotdata->task_card_set[i]->controller->input_data_a.setZero(4,robotdata->task_card_set[i]->controller->dim);
                                        robotdata->task_card_set[i]->controller->input_data_d.setZero(4,robotdata->task_card_set[i]->controller->dim);
                                        robotdata->task_card_set[i]->controller->output_data.setZero(4,robotdata->task_card_set[i]->controller->dim);

                                    }

                                    if(robotdata->task_card_set[i]->controller->controller_type == basic_controller::Impedance)
                                    {
                                        if(_task_array_object_object.key() == "K")
                                        {
                                            robotdata->task_card_set[i]->controller->para(0,0) = _task_array_object_object.value().toDouble();
                                        }

                                        if(_task_array_object_object.key() == "M")
                                        {
                                            robotdata->task_card_set[i]->controller->para(1,0) = _task_array_object_object.value().toDouble();
                                        }

                                        if(_task_array_object_object.key() == "B")
                                        {
                                            robotdata->task_card_set[i]->controller->para(2,0) = _task_array_object_object.value().toDouble();
                                        }

                                    }

                                    if(robotdata->task_card_set[i]->controller->controller_type == basic_controller::PID || robotdata->task_card_set[i]->controller->controller_type == basic_controller::PID_orient)
                                    {
                                        if(_task_array_object_object.key() == "k_P_1")
                                        {
                                            robotdata->task_card_set[i]->controller->para(0,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_P_2")
                                        {
                                            robotdata->task_card_set[i]->controller->para(1,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_P_3")
                                        {
                                            robotdata->task_card_set[i]->controller->para(2,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_P_4")
                                        {
                                            robotdata->task_card_set[i]->controller->para(3,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_P_5")
                                        {
                                            robotdata->task_card_set[i]->controller->para(4,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_P_6")
                                        {
                                            robotdata->task_card_set[i]->controller->para(5,0) = _task_array_object_object.value().toDouble();
                                        }                                                                                

                                        if(_task_array_object_object.key() == "k_D_1")
                                        {
                                            robotdata->task_card_set[i]->controller->para(6,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_D_2")
                                        {
                                            robotdata->task_card_set[i]->controller->para(7,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_D_3")
                                        {
                                            robotdata->task_card_set[i]->controller->para(8,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_D_4")
                                        {
                                            robotdata->task_card_set[i]->controller->para(9,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_D_5")
                                        {
                                            robotdata->task_card_set[i]->controller->para(10,0) = _task_array_object_object.value().toDouble();
                                        }
                                        if(_task_array_object_object.key() == "k_D_6")
                                        {
                                            robotdata->task_card_set[i]->controller->para(11,0) = _task_array_object_object.value().toDouble();
                                        }                                                                                                                                                                                                        

                                    }

                                    _task_array_object_object++;
                                }

                                // task sensor id
                                robotdata->task_card_set[i]->sensor_id = 0;
                                _task_array_object_object = t_task_array_object_object.begin();
                                while(_task_array_object_object != t_task_array_object_object.end())
                                {
                                    if(_task_array_object_object.key() == "sensor_type")
                                    {
                                        if(_task_array_object_object.value().toString() == "无")
                                        {
                                            robotdata->task_card_set[i]->sensor_id = 0;
                                        }else if(_task_array_object_object.value().toString() == "估计关节外力")
                                        {
                                            robotdata->task_card_set[i]->sensor_id = _nsensor+1;

                                        }else if(_task_array_object_object.value().toString() == "六维力传感器")
                                        {
                                            std::vector<Sensor*>::iterator _sensor_iter = robotdata->sensor_set.begin();
                                            while(_sensor_iter!=robotdata->sensor_set.end())
                                            {
                                                if(((*_sensor_iter)->link_num == robotdata->task_card_set[i]->joint_id)&&((*_sensor_iter)->type == sensor_type::FT_sensor))
                                                {
                                                    robotdata->task_card_set[i]->sensor_id = (*_sensor_iter)->ID;
                                                }
                                                _sensor_iter++;
                                            }

                                        }else{
                                            robotdata->task_card_set[i]->sensor_id = 0;
                                        }

                                    }
                                    _task_array_object_object++;
                                }



                            }

                            if(_task_array_object.key() == "constraintArray")
                            {
                                // task variables
                                std::vector<double> _A_x;
                                std::vector<double> _b_x;

                                std::vector<double> _A_x_dot;
                                std::vector<double> _b_x_dot;

                                std::vector<double> _A_x_f;
                                std::vector<double> _b_x_f;

                                // constraint_type: 1 线性不等式约束; 2 边界不等式约束; 3 摩擦锥
                                int constraint_type = 1;
                                // task_variable: 1 x; 2 x_dot; 3 f;
                                int task_variable = 1;
                                QJsonArray _task_array_object_array = _task_array_object.value().toArray();
                                for(int constraint_i = 0; constraint_i < _task_array_object_array.size();constraint_i++)
                                {
                                    QJsonObject t_task_array_object_array_object = _task_array_object_array.at(constraint_i).toObject();
                                    QJsonObject::iterator _task_array_object_array_object = t_task_array_object_array_object.begin();

                                    while(_task_array_object_array_object != t_task_array_object_array_object.end())
                                    {
                                        if(_task_array_object_array_object.key() == "constraint_type")
                                        {
                                            if(_task_array_object_array_object.value().toString() == "线性不等式约束")
                                            {
                                                constraint_type = 1;
                                            }else if(_task_array_object_array_object.value().toString() == "边界不等式约束")
                                            {
                                                constraint_type = 2;
                                            }else if(_task_array_object_array_object.value().toString() == "摩擦锥")
                                            {
                                                constraint_type = 3;
                                            }else{
                                                constraint_type = 0;
                                            }
                                        }

                                        if(_task_array_object_array_object.key() == "task_variable")
                                        {
                                            if(_task_array_object_array_object.value().toString() == "x")
                                            {
                                                task_variable = 1;
                                            }else if(_task_array_object_array_object.value().toString() == "xdot")
                                            {
                                                task_variable = 2;
                                            }else if(_task_array_object_array_object.value().toString() == "f")
                                            {
                                                task_variable = 3;
                                            }else{
                                                task_variable = 0;
                                            }
                                        }

                                        _task_array_object_array_object++;
                                    }

                                    _task_array_object_array_object = t_task_array_object_array_object.begin();
                                    while(_task_array_object_array_object != t_task_array_object_array_object.end())
                                    {
                                        //
                                        if(task_variable == 1)
                                        {
                                            if(constraint_type == 1)
                                            {
                                                if(_task_array_object_array_object.key() == "A_Array")
                                                {
                                                    QJsonArray _A_Array = _task_array_object_array_object.value().toArray();
                                                    for(int _A_Array_i = 0; _A_Array_i < _A_Array.size();_A_Array_i++)
                                                    {
                                                        _A_x.push_back(_A_Array.at(_A_Array_i).toDouble());
                                                    }
                                                }
                                                if(_task_array_object_array_object.key() == "b_Array")
                                                {
                                                    QJsonArray _b_Array = _task_array_object_array_object.value().toArray();
                                                    for(int _b_Array_i = 0; _b_Array_i < _b_Array.size();_b_Array_i++)
                                                    {
                                                        _b_x.push_back(_b_Array.at(_b_Array_i).toDouble());
                                                    }
                                                }
                                            }else if(constraint_type == 2){
                                                if(_task_array_object_array_object.key() == "boundary_constraint_LB_Array")
                                                {
                                                    QJsonArray _Array = _task_array_object_array_object.value().toArray();
                                                    for(int j = 0; j< _Array.size();j++)
                                                    {
                                                        _b_x.push_back(-_Array.at(j).toDouble());
                                                        //  unit vector I
                                                        Eigen::VectorXd I = Eigen::VectorXd::Zero(robotdata->task_card_set[i]->dim);
                                                        if(j < robotdata->task_card_set[i]->dim)
                                                        {
                                                            I(j) = -1.0;
                                                        }else{
                                                            std::cout<<"boundary_constraint_LB_Array dimesion is wrong!"<<std::endl;
                                                            return;
                                                        }
                                                        for(int k = 0; k< robotdata->task_card_set[i]->dim; k++)
                                                        {
                                                            _A_x.push_back(I(k));
                                                        }
                                                    }
                                                }
                                                if(_task_array_object_array_object.key() == "boundary_constraint_UB_Array")
                                                {
                                                    QJsonArray _Array = _task_array_object_array_object.value().toArray();
                                                    for(int j = 0; j< _Array.size();j++)
                                                    {
                                                        _b_x.push_back(_Array.at(j).toDouble());
                                                        //  unit vector I
                                                        Eigen::VectorXd I = Eigen::VectorXd::Zero(robotdata->task_card_set[i]->dim);
                                                        if(j < robotdata->task_card_set[i]->dim)
                                                        {
                                                            I(j) = 1.0;
                                                        }else{
                                                            std::cout<<"boundary_constraint_UB_Array dimesion is wrong!"<<std::endl;
                                                            return;
                                                        }
                                                        for(int k = 0; k< robotdata->task_card_set[i]->dim; k++)
                                                        {
                                                            _A_x.push_back(I(k));
                                                        }
                                                    }
                                                }
                                            }else if(constraint_type == 3){
                                                // friction cone

                                            }else{
                                                std::cout<<" no such constraint_type!"<<std::endl;
                                                return;
                                            }
                                        }else if(task_variable == 2)
                                        {
                                            if(constraint_type == 1)
                                            {
                                                if(_task_array_object_array_object.key() == "A_Array")
                                                {
                                                    QJsonArray _A_Array = _task_array_object_array_object.value().toArray();
                                                    for(int _A_Array_i = 0; _A_Array_i < _A_Array.size();_A_Array_i++)
                                                    {
                                                        _A_x_dot.push_back(_A_Array.at(_A_Array_i).toDouble());
                                                    }
                                                }
                                                if(_task_array_object_array_object.key() == "b_Array")
                                                {
                                                    QJsonArray _b_Array = _task_array_object_array_object.value().toArray();
                                                    for(int _b_Array_i = 0; _b_Array_i < _b_Array.size(); _b_Array_i++)
                                                    {
                                                        _b_x_dot.push_back(_b_Array.at(_b_Array_i).toDouble());
                                                    }
                                                }
                                            }else if(constraint_type == 2){
                                                if(_task_array_object_array_object.key() == "boundary_constraint_LB_Array")
                                                {
                                                    QJsonArray _Array = _task_array_object_array_object.value().toArray();
                                                    for(int j = 0; j < _Array.size();j++)
                                                    {
                                                        _b_x_dot.push_back(-_Array.at(j).toDouble());
                                                        //  unit vector I
                                                        Eigen::VectorXd I = Eigen::VectorXd::Zero(robotdata->task_card_set[i]->dim);
                                                        if(j < robotdata->task_card_set[i]->dim)
                                                        {
                                                            I(j) = -1.0;
                                                        }else{
                                                            std::cout<<"boundary_constraint_LB_Array dimesion is wrong!"<<std::endl;
                                                            return;
                                                        }
                                                        for(int k = 0; k< robotdata->task_card_set[i]->dim; k++)
                                                        {
                                                            _A_x_dot.push_back(I(k));
                                                        }
                                                    }
                                                }
                                                if(_task_array_object_array_object.key() == "boundary_constraint_UB_Array")
                                                {
                                                    QJsonArray _Array = _task_array_object_array_object.value().toArray();
                                                    for(int j = 0; j < _Array.size();j++)
                                                    {
                                                        _b_x_dot.push_back(_Array.at(j).toDouble());
                                                        //  unit vector I
                                                        Eigen::VectorXd I = Eigen::VectorXd::Zero(robotdata->task_card_set[i]->dim);
                                                        if(j < robotdata->task_card_set[i]->dim)
                                                        {
                                                            I(j) = 1.0;
                                                        }else{
                                                            std::cout<<"boundary_constraint_UB_Array dimesion is wrong!"<<std::endl;
                                                            return;
                                                        }
                                                        for(int k = 0; k< robotdata->task_card_set[i]->dim; k++)
                                                        {
                                                            _A_x_dot.push_back(I(k));
                                                        }
                                                    }
                                                }
                                            }else if(constraint_type == 3){
                                                // friction cone

                                            }else{
                                                std::cout<<" no such constraint_type!"<<std::endl;
                                                return;
                                            }
                                        }else if(task_variable == 3)
                                        {
                                            if(constraint_type == 1)
                                            {
                                                if(_task_array_object_array_object.key() == "A_Array")
                                                {
                                                    QJsonArray _A_Array = _task_array_object_array_object.value().toArray();
                                                    for(int _A_Array_i = 0; _A_Array_i < _A_Array.size();_A_Array_i++)
                                                    {
                                                        _A_x_f.push_back(_A_Array.at(_A_Array_i).toDouble());
                                                    }
                                                }
                                                if(_task_array_object_array_object.key() == "b_Array")
                                                {
                                                    QJsonArray _b_Array = _task_array_object_array_object.value().toArray();
                                                    for(int _b_Array_i =0; _b_Array_i<_b_Array.size();_b_Array_i++)
                                                    {
                                                        _b_x_f.push_back(_b_Array.at(_b_Array_i).toDouble());
                                                    }
                                                }
                                            }else if(constraint_type == 2){
                                                if(_task_array_object_array_object.key() == "boundary_constraint_LB_Array")
                                                {
                                                    QJsonArray _Array = _task_array_object_array_object.value().toArray();
                                                    for(int _Array_i = 0; _Array_i < _Array.size();_Array_i++)
                                                    {
                                                        _b_x_f.push_back(-_Array.at(_Array_i).toDouble());
                                                        //  unit vector I
                                                        Eigen::VectorXd I = Eigen::VectorXd::Zero(robotdata->task_card_set[i]->dim);
                                                        if(_Array_i < robotdata->task_card_set[i]->dim)
                                                        {
                                                            I(_Array_i) = -1.0;
                                                        }else{
                                                            std::cout<<"boundary_constraint_LB_Array dimesion is wrong!"<<std::endl;
                                                            return;
                                                        }
                                                        for(int k = 0; k< robotdata->task_card_set[i]->dim; k++)
                                                        {
                                                            _A_x_f.push_back(I(k));
                                                        }
                                                    }
                                                }
                                                if(_task_array_object_array_object.key() == "boundary_constraint_UB_Array")
                                                {
                                                    QJsonArray _Array = _task_array_object_array_object.value().toArray();
                                                    for(int _Array_i = 0;_Array_i<_Array.size();_Array_i++)
                                                    {
                                                        _b_x_f.push_back(_Array.at(_Array_i).toDouble());
                                                        //  unit vector I
                                                        Eigen::VectorXd I = Eigen::VectorXd::Zero(robotdata->task_card_set[i]->dim);
                                                        if(_Array_i < robotdata->task_card_set[i]->dim)
                                                        {
                                                            I(_Array_i) = 1.0;
                                                        }else{
                                                            std::cout<<"boundary_constraint_UB_Array dimesion is wrong!"<<std::endl;
                                                            return;
                                                        }
                                                        for(int k = 0; k< robotdata->task_card_set[i]->dim; k++)
                                                        {
                                                            _A_x_f.push_back(I(k));
                                                        }
                                                    }
                                                }
                                            }else if(constraint_type == 3){
                                                // friction cone

                                            }else{
                                                std::cout<<" no such constraint_type!"<<std::endl;
                                                return;
                                            }
                                        }else{
                                            std::cout<<"no such task_variable!"<<std::endl;
                                            return;
                                        }

                                        _task_array_object_array_object++;
                                    }
                                }

                                // set constraints in the task
                                // set A_ineq_x and b_ineq_x
                                int rows = _b_x.size();
                                int cols = robotdata->task_card_set[i]->dim;
                                robotdata->task_card_set[i]->A_ineq_x.setZero(rows, cols);
                                robotdata->task_card_set[i]->b_ineq_x.setZero(rows);
                                for(int m = 0;m < rows; m++)
                                {
                                    robotdata->task_card_set[i]->b_ineq_x(m) = _b_x[m];
                                    for(int n = 0;n<cols;n++)
                                    {
                                        robotdata->task_card_set[i]->A_ineq_x(m,n) = _A_x[m*cols + n];
                                    }
                                }
                                //set A_ineq_x_dot b_ineq_x_dot
                                rows = _b_x_dot.size();
                                cols = robotdata->task_card_set[i]->dim;
                                robotdata->task_card_set[i]->A_ineq_x_dot.setZero(rows, cols);
                                robotdata->task_card_set[i]->b_ineq_x_dot.setZero(rows);
                                for(int m = 0;m < rows; m++)
                                {
                                    robotdata->task_card_set[i]->b_ineq_x_dot(m) = _b_x_dot[m];
                                    for(int n = 0;n<cols;n++)
                                    {
                                        robotdata->task_card_set[i]->A_ineq_x_dot(m,n) = _A_x_dot[m*cols + n];
                                    }
                                }
                                // set A_ineq_F b_ineq_F
                                rows = _b_x_f.size();
                                cols = robotdata->task_card_set[i]->dim;
                                robotdata->task_card_set[i]->A_ineq_F.setZero(rows, cols);
                                robotdata->task_card_set[i]->b_ineq_F.setZero(rows);
                                for(int m = 0;m < rows; m++)
                                {
                                    robotdata->task_card_set[i]->b_ineq_F(m) = _b_x_f[m];
                                    for(int n = 0;n<cols;n++)
                                    {
                                        robotdata->task_card_set[i]->A_ineq_F(m,n) = _A_x_f[m*cols + n];
                                    }
                                }
                            }
                            _task_array_object++;
                        }
                    }
                }

                it++;
            }
            std::vector<Task*>::iterator _iter;
            // update task priority
            // reset the min priority to 1

            int _min = (*robotdata->task_card_set.begin())->priority;
            for(_iter = robotdata->task_card_set.begin();_iter!=robotdata->task_card_set.end();_iter++)
            {
                if(_min > (*_iter)->priority)
                {
                    _min = (*_iter)->priority;
                }
            }
            if(_min<1)
            {
                std::cout<<"priority setting wrong!"<<std::endl;
            }
            for(_iter = robotdata->task_card_set.begin();_iter!=robotdata->task_card_set.end();_iter++)
            {
                (*_iter)->priority = (*_iter)->priority - (_min - 1);
            }
            // from 1 ++ iterate compute _npriority
            // count label
            int pre_count = 0;
            int _count = 0;
            int prio = 1;
            while(_count!=(robotdata->ntask))
            {
                pre_count = _count;
                for(_iter = robotdata->task_card_set.begin();_iter!=robotdata->task_card_set.end();_iter++)
                {
                    if((*_iter)->priority == prio)
                    {
                        _count++;
                    }
                }
                while(_count == pre_count)
                {
                    for(_iter = robotdata->task_card_set.begin();_iter!=robotdata->task_card_set.end();_iter++)
                    {
                        if((*_iter)->priority > prio)
                        {
                            (*_iter)->priority = (*_iter)->priority -1;
                        }
                    }
                    for(_iter = robotdata->task_card_set.begin();_iter!=robotdata->task_card_set.end();_iter++)
                    {
                        if((*_iter)->priority == prio)
                        {
                            _count++;
                        }
                    }
                }
                prio++;
            }
            _npriority = prio - 1;
            robotdata->npriority = _npriority;
            // for temp

            robotdata->contactforce.resize(12,1);
            robotdata->contactforce.setZero();

            robotdata->mpcforceref.resize(12,1);
            robotdata->mpcforceref.setZero();
            robotdata->imu_init.setIdentity();
            robotdata->WF1 = Eigen::MatrixXd::Identity(12,12);
            robotdata->WF2 = Eigen::MatrixXd::Identity(12,12);
            // std::cout<<"!"<<std::endl;
            // read  wbic qp weights
            it = object.begin();
            while(it!=object.end())
            {
                if(it.key() == "wbctype")
                {
                    if(it.value().toString() == "WQP")
                    {
                        robotdata->wbcsolver = Wbc_Solver_type::WQP;
                    }else if(it.value().toString() == "WBIC")
                    {
                        robotdata->wbcsolver = Wbc_Solver_type::WBIC;
                    }else{
                        robotdata->wbcsolver = Wbc_Solver_type::WBIC;
                        std::cout<<"Default WBC has been set"<<std::endl;
                    }
                }
                if(it.key() == "wq1")
                {
                    robotdata->wq1 = it.value().toDouble();
                }
                if(it.key() == "wq2")
                {
                    robotdata->wq2 = it.value().toDouble();
                }
                if(it.key() == "wq3")
                {
                    robotdata->wq3 = it.value().toDouble();
                }
                if(it.key() == "vq1")
                {
                    robotdata->vq1 = it.value().toDouble();
                }
                if(it.key() == "vq2")
                {
                    robotdata->vq2 = it.value().toDouble();
                }
                if(it.key() == "vq3")
                {
                    robotdata->vq3 = it.value().toDouble();
                }

                if(it.key() == "mq1")
                {
                    robotdata->mq1 = it.value().toDouble();
                }
                if(it.key() == "mq2")
                {
                    robotdata->mq2 = it.value().toDouble();
                }
                if(it.key() == "mq3")
                {
                    robotdata->mq3 = it.value().toDouble();
                }
                if(it.key() == "fq1")
                {
                    robotdata->fq1 = it.value().toDouble();
                }
                if(it.key() == "fq2")
                {
                    robotdata->fq2 = it.value().toDouble();
                }
                if(it.key() == "fq3")
                {
                    robotdata->fq3 = it.value().toDouble();
                }
                if(it.key() == "imu_init")
                {
                    robotdata->imu_init(0,0) = it.value().toArray().at(0).toDouble();
                    robotdata->imu_init(1,0) = it.value().toArray().at(1).toDouble();
                    robotdata->imu_init(2,0) = it.value().toArray().at(2).toDouble();
                    robotdata->imu_init(0,1) = it.value().toArray().at(3).toDouble();
                    robotdata->imu_init(1,1) = it.value().toArray().at(4).toDouble();
                    robotdata->imu_init(2,1) = it.value().toArray().at(5).toDouble();
                    robotdata->imu_init(0,2) = it.value().toArray().at(6).toDouble();
                    robotdata->imu_init(1,2) = it.value().toArray().at(7).toDouble();
                    robotdata->imu_init(2,2) = it.value().toArray().at(8).toDouble();
                }
                if(it.key() == "WF1")
                {
                    for(int i = 0; i<12;i++)
                    {
                        robotdata->WF1(i,i) = it.value().toArray().at(i).toDouble();
                    }
                }
                if(it.key() == "WF2")
                {
                    for(int i = 0; i<12;i++)
                    {
                        robotdata->WF2(i,i) = it.value().toArray().at(i).toDouble();
                    }
                }
                it++;
            }            
        }
    }
}
