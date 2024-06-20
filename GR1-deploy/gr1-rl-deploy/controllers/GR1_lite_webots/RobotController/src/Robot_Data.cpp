#include "../include/Robot_Data.h"
// #include "Robot_Data.h"
Robot_Data::Robot_Data()
{
}

Robot_Data::~Robot_Data()
{
    for(std::vector<Task*>::iterator iter = task_card_set.begin(); iter != task_card_set.end(); iter++)
    {
        delete (*iter);
    }
}
