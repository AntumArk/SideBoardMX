  /* USER CODE BEGIN 7 */
#include "bldc.h"



inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:
      *q = u - v;
      // *u = 0;
      // *v = pwm;
      // *w = -pwm;
      break;
    case 1:
      *q = u;
      // *u = -pwm;
      // *v = pwm;
      // *w = 0;
      break;
    case 2:
      *q = u;
      // *u = -pwm;
      // *v = 0;
      // *w = pwm;
      break;
    case 3:
      *q = v;
      // *u = 0;
      // *v = -pwm;
      // *w = pwm;
      break;
    case 4:
      *q = v;
      // *u = pwm;
      // *v = -pwm;
      // *w = 0;
      break;
    case 5:
      *q = -(u - v);
      // *u = pwm;
      // *v = 0;
      // *w = -pwm;
      break;
    default:
      *q = 0;
      // *u = 0;
      // *v = 0;
      // *w = 0;
  }
}
 /* USER CODE END 7 */