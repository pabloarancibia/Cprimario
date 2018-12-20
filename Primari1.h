#ifndef PRIMARIO_H
#define PRIMARIO_H

#define MAXDIFAZ     2

#define tolerancia   9//7
#define max_vuelta 4096
#define max_aus_cons 5 /*Determina el fin del blanco*/
#define blancocorto 3

/******************************************************************/


///void far trans(void);  /*rutina de interrupcion*/
void comset(void);
void inicializa(void);
int centro_blanco(void);
void cabecera(char sector);
void blancos(unsigned int azmt,unsigned int rango,unsigned int altura);
void rut_tx(void);
int ind_fifo;
char control;


/**********************************************************/
/*variables globales correspondientes a la rutina de 
calculo de centro de blanco*/

struct presencia
{
 int rangoinic;
 int aztinic;
 char auscons;
 unsigned char Altura;
} prs[1024];
/**********************************************************************
Variables, constantes y demas utilizadas para 
el procesamiento de Altura:
********************************************************************** */
unsigned char PrsAltura; /*Indica que ha ingresado un dato de altura   */
unsigned int AlturaTemp; /*Lleva el dato de Altura ingresado           */

/***********************************************************************/
int aztant;
int aztprom;
int indice;
int rangoactual;
unsigned int sectoractual;
unsigned int sectoranterior;
int bandera_azt;
unsigned int azimut_anterior;
int bandera;
int rangoinic;
int banderainic;
unsigned int azimuth;
unsigned int lectura;
unsigned int rango;
unsigned char indato;

/**********************************************************/

unsigned int az_ant;
unsigned char sec_ant;
unsigned char header[4];



#endif