
#ifndef C_CCM_H
#define C_CCM_H

#include "C_std.h"
#include "C_cs.h"

typedef enum {
  CCMatrixOK           =  OK,
  CCMatrixDimError     = -1,
  CCMatrixKindError    = -2,
  CCMatrixInitError    = -3,
  CCMatrixNotEnoughRWS = notEnoughRWS,
  CCMatrixNotEnoughIWS = notEnoughIWS
} CCMatrixStatus;

#endif
