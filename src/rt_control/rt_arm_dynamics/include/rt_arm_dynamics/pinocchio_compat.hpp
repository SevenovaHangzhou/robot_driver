#ifndef RT_ARM_DYNAMICS__PINOCCHIO_COMPAT_HPP_
#define RT_ARM_DYNAMICS__PINOCCHIO_COMPAT_HPP_

#if __has_include(<pinocchio/multibody/model.hpp>)
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#else
#include <pinocchio/multibody.hpp>
#endif

#endif // RT_ARM_DYNAMICS__PINOCCHIO_COMPAT_HPP_
