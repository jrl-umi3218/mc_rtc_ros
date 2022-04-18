/*
 * Copyright 2016-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*
 * This header avoids spurious warnings from Qt libraries
 */

#pragma GCC system_header

#include <QtGlobal>
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#  include <QtGui>
#else
#  include <QtWidgets>
#endif
