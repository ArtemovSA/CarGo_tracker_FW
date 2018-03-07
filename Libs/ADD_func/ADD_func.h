
#ifndef ADD_FUNC_H
#define ADD_FUNC_H

#define HI(x) (x & 0xFF00)>>8
#define LO(x) (x & 0x00FF)
#define ADD(x,y) (x)|(y<<8)
#define max(a,b) ((a) > (b) ? (a) : (b))

#endif