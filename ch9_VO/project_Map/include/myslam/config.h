//
// Created by chen-tian on 8/6/17.
//
//! config 类负责参数文件的读取， 并在程序任意地方都可以随时提供参数的值
//! 所以把文件写成单件（Singleton）模式. 它只有一个全局对象， 当我们设置参数文件时，
//! 创建该对象并读取参数文件，随后就可以在任意地方访问参数值， 最后在程序结束时自动销毁。
//! 单件模式是一种用于确保整个应用程序中只有一个类实例且这个实例所占资源在整个应用程序中是共享时的程序设计方法.

#ifndef PROJECT_MAP_CONFIG_H
#define PROJECT_MAP_CONFIG_H

#include "common_include.h"

namespace myslam
{

class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;  //read yaml files

    Config () {}  //private constructor makes a singleton

public:
    ~Config();

    //set a new config file
    static void setParameterFile( const std::string& filename );

    //access the parameter values
    template < typename T >
    static T get( const std::string& key )
    {
        return T( Config::config_ -> file_[key] );
    }
};
}

#endif //PROJECT1_CONFIG_H
