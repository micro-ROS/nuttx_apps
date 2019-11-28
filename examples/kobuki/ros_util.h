#ifndef __ROS_UTIL_H
#define __ROS_UTIL_H

#include <exception>
#include <string>
#include <syslog.h>

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

// TODO replace this with calls to rcutil_logging
#define ROS_DEBUG(fmt, ...)     /*syslog(LOG_DEBUG, fmt, __VA_ARGS__);*/
#define ROS_INFO(fmt, ...)      syslog(LOG_INFO, fmt, __VA_ARGS__);
#define ROS_ERROR(fmt, ...)     syslog(LOG_ERR, fmt, __VA_ARGS__);
#endif