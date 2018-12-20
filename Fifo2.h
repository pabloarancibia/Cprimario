#ifndef FIFO_H
#define FIFO_H



//direcciones de operaciones de control
//tomando como base la direccion 0x0D00
#define STSFIFO 0x0310
#define CLRWDT1 0x0320
#define FIFO    0x0330
#define AZT     0x0340
#define CNTRL   0x0350

//banderas
#define FULL     0x0002
#define HALF     0x0003
//#define EMPTY    0x0005
#define EMPTY    0x0001

//datos de control
#define RSTFIFO 0x00fe
#define ENDATOS 0x00fd






#endif