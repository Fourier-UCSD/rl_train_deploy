#include"../include/pythoncall.h"
pythoncall::pythoncall(){}

pythoncall::~pythoncall(){}

void pythoncall::init()
{
    // temp
    _scripts_location = "test";
    // 添加的python的整个完成的环境包的路劲，必须添加。
    Py_SetPythonHome(L"/home/rxy/anaconda3/envs/python38");
    // 进行初始化
    Py_Initialize();
    if (Py_IsInitialized())
        std::cout << "Init Success" << std::endl;

    PyRun_SimpleString("import sys");             // 在python初始化之后
    PyRun_SimpleString("sys.path.append('./')");   // ""里面填写的是python的语言
    PyRun_SimpleString("print(sys.version)\n");

    // PyObject * pModule = NULL;
    // PyObject * pDict = NULL; 
    // PyObject * pFunc = NULL; 
    // PyObject * pClass = NULL;
    // PyObject * pConstruct = NULL;
    // PyObject * pInstance = NULL;

    //这里是要调用的文件名
    pModule = PyImport_ImportModule(_scripts_location.c_str());

    //加载文件中的函数名、类名
    pDict = PyModule_GetDict(pModule);
    if (!pDict)
    {
        printf("Cant find dictionary./n");
    }

    // 根据类名获取该类
    pClass = PyDict_GetItemString(pDict, _class_name.c_str());
    if (!pClass) {
        printf("Can't find Student class.\n");
        return ;
    }

    // 得到类的构造函数
    pConstruct = PyInstanceMethod_New(pClass);    //python3的
    if (!pConstruct) {
        printf("Can't create Student instance.\n");
        return ;
    }
    // 类的实例化
    pInstance = PyObject_CallObject(pConstruct, NULL);

    // PyObject_CallMethod(pInstance, "SetName", "s", "1111");
    // PyObject_CallMethod(pInstance, "PrintName", NULL, NULL);

    // {
    //     //调用python脚本中的函数 并执行
    //     pFunc = PyObject_GetAttrString(pModule, "hello");
    //     //调用函数
        // PyEval_CallObject(pFunc, NULL);
    //     Py_DECREF(pFunc);

    //     pFunc = PyObject_GetAttrString(pModule, "world");
        // PyObject_CallFunction(pFunc, "s", "zhengji");
    //     Py_DECREF(pFunc);
    // }

    //调用Py_Finalize，这个根Py_Initialize相对应的。
    // Py_Finalize();

}

void pythoncall::close()
{
    Py_DECREF(pModule);
    Py_DECREF(pDict);
    Py_DECREF(pFunc);
    Py_DECREF(pClass);
    Py_DECREF(pConstruct);
    Py_DECREF(pInstance);
    Py_Finalize();
}

void pythoncall::fun(double _input_1,
            double _input_2,
            double &output,
            std::string function_name)
            {
                if(function_name == "Add")
                {
                    PyObject * res = PyObject_CallMethod(pInstance, "Add", "ff", _input_1, _input_2);
                    output = PyFloat_AsDouble(res);
                    std::cout<<"test add res: "<< res<< std::endl;
                }else{
                    std::cout<<"no such function!"<<std::endl;
                }
            }