#ifndef __ROS_UTIL_H
#define __ROS_UTIL_H

#include <exception>
#include <string>
#include <rcl/error_handling.h>
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

#define PRINT_RCL_ERROR(func) \
  do { \
    ROS_ERROR("error in " #func ": %s\n", rcutils_get_error_string().str); \
    rcl_reset_error(); \
  } while (0)

#define CHECK_RET(FUNC) { \
    rcl_ret_t macro_rc = FUNC ; \
    if(macro_rc != RMW_RET_OK) { \
        PRINT_RCL_ERROR(FUNC); \
        return -1; \
    } else { \
        /* fprintf(stdout, "OK on " #FUNC "\n"); */\
    } \
}
#define WARN_RET(FUNC) { rcl_ret_t macro_rc = FUNC ; if(macro_rc != RMW_RET_OK) { PRINT_RCL_ERROR(FUNC); } }
#define THROW_RET(FUNC) { rcl_ret_t macro_rc = FUNC ; if(macro_rc != RMW_RET_OK) { throw RCLException(rcutils_get_error_string().str); } }

#endif
