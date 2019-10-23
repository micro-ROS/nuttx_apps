#ifndef __ROS_UTIL_H
#define __ROS_UTIL_H

#include <exception>
#include <string>

namespace kobuki 
{

class RCLException : public std::exception {
public:
    RCLException(const std::string& msg) : _msg(msg) {
    }

    virtual const char* what() const noexcept override {
        return _msg.c_str();
    }
private:
    const std::string _msg;
};

}

#endif