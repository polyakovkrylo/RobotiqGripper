#pragma once

#include <QtCore/qglobal.h>

#if defined(ROBOTIQGRIPPER_LIBRARY)
#  define ROBOTIQGRIPPERSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ROBOTIQGRIPPERSHARED_EXPORT Q_DECL_IMPORT
#endif

