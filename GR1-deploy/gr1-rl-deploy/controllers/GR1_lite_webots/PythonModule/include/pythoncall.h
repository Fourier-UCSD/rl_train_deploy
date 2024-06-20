#include<Python.h>
#include<string>
#include<iostream>
#include<Eigen/Dense>
// #include<thread.h>
class pythoncall
{
    public:
    pythoncall();
    ~pythoncall();
    // init 
    void init();
    // close
    void close();
    // set path
        //todo
    // function
    void fun(double _input_1,
            double _input_2,
            double &output,
            std::string function_name);
    private:
    PyObject * pModule = NULL;
    PyObject * pDict = NULL; 
    PyObject * pFunc = NULL; 
    PyObject * pClass = NULL;
    PyObject * pConstruct = NULL;
    PyObject * pInstance = NULL;
    // python envs path
    std::string _scripts_python_envs;
    // python scripts location and name
    std::string _scripts_location;
    // class name string
    std::string _class_name;
};