//*/ PROGRAMA NUEVO PRIMARIO EN case /* value */:
///*******************Puente********************************///
///Inicio Includes y Defines de Puente///
#include "primari1.h"
#include "Fifo2.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"
#define REG_BASE 0xFF200000
#define REG_SPAN 0x00200000
///Fin Includes y Defines de Puente///
///Inicio Variables globales de Puente///
void* virtual_base;
void* D_addr;		//Dirección del vector D de lectura-escritura
void* A_addr;		//Dirección del vector A de lectura-escritura
int fd;
int D;				//Valor del vector D
int A;				//Valor del vector A
int cambio;
///Fin Variables globales de Puente///
///********************************************************///
///Inicio Includes y defines viejos///
///Fin Includes y defines viejos///
///Inicio Variables globales viejas///

unsigned char  Buff[1024];
unsigned char  CabBuffUDP[4],iwr_u=0,PlotBuffUDP[6];  /// //
int iSendSock; ///int iSendSock;
int UDP_PORT;  //Direccion puerto remoto///

unsigned char  iwr=0,ird=0;
unsigned FlagIni = 0;
unsigned int DatoAnt, AzAnt;
unsigned char SectorActual;//version WP101
unsigned int AREA_INHIBICION=0;
unsigned char InBuff[256];
unsigned int IndWr, IndRd;
unsigned char bu1, bu2, bu3, bu4, bu5, bu6, bu7, bu8, bu9, bu10;
int ok1, ok2, ok3;
int Flag_alt;
float X_alt, Y_alt;
float X_bco, Y_bco;
unsigned int sector_alt;
unsigned int Dist, Dist_max=3;//En MN. Valor estimado aproximado
unsigned int alt_alt;
float PI=3.1415;
unsigned int eco=0;
///Fin Variables globales viejas///
///Inicio declaracion funciones viejas///
void ResetFifo(void);
void InicializaPr(void);
int ChkFlag(void);
int GetDato(unsigned int* );
int CtrlDato(unsigned int );
int Primario(unsigned int );
int Primario_eco(unsigned int );
void cabecera(char );
void blancos(unsigned int , unsigned int , unsigned int );
///Fin declaracion funciones viejas///

////////DECLARACIONES SOCKETE//////////
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<arpa/inet.h>
#include<sys/socket.h>

#define SERVER "127.0.0.1"
#define BUFLEN 1024  //Max length of buffer
#define PORT 113113   //The port on which to listen for incoming data

//////////FIN DECLARACIONES SOCKETE///////////

void ResetFifo()
{
  *(uint32_t *)A_addr=0x4350;//-/ outpw(0x350,RSTFIFO); //enable datos
  usleep(1000);
     *(uint32_t *)D_addr=RSTFIFO;  /// no hace falta porque permanece A en 0x2350 ///*(uint32_t *)A_addr=0x2350;//-/outpw(0x350,ENDATOS); //reset fifo
	 usleep(1000);
  *(uint32_t *)D_addr=ENDATOS;
  usleep(1000);
   *(uint32_t *)D_addr=RSTFIFO;
   usleep(1000);

}

void InicializaPr()
{

 indice=0;
 sec_ant=0;

 header[0]=0xff;
 header[1]=0xff;
 header[2]=0x00;
 header[3]=0x00;

 bandera=0;
 banderainic=1;
 azimuth=0;
 lectura=0;
 rango=0;
 sectoranterior=0;

 bandera_azt=0;
 azimut_anterior=0xf000;
 PrsAltura=0;
 AlturaTemp=0;

}

int ChkFlag()
{
  unsigned int Flag;

  *(uint32_t *)A_addr=0x2310;///leemos en STSFIFO///STSFIFO; //-/ Flag = inp(STSFIFO);
  Flag = *(uint32_t *)D_addr;

  if(Flag == 0x02)
  	{
	  //printf("%x ",Flag);
	  ResetFifo();
	  FlagIni = 0;
	  return -1;
  	}
  //if(Flag == 0x0C)//FULL
  //    {
  //     ResetFifo();
  //     FlagIni = 0;
       //iwr = ird = 0;
  //     printf("R1 ");
  //     return -1;
  //    }

  //if((Flag&0x0003) != EMPTY) return 0;
  if(Flag == 0x00) return 0; //Hay Dato
  return -1;
}

int GetDato(unsigned int* Dato )
{
  unsigned int Aux;
  *(uint32_t *)A_addr=0x2330; ///leemos en FIFO/// FIFO;//-/Aux = inpw(FIFO);
  Aux = *(uint32_t *)D_addr;///
  if(CtrlDato(Aux))
    return -1;
  *Dato = Aux;

 return 0;
}

int CtrlDato(unsigned int Dato){
  //CONTROLA QUE EL DATO RECIBIDO PERTENEZCA A ALGUNO DE LOS ESPERADOS
  if(((Dato&0xf000) != 0x8000)&&
     ((Dato&0xf000) != 0x3000)&&
     ((Dato&0xf000) != 0xC000))
     {
      printf("ERROR0   %x   ",Dato);
      ResetFifo();
      FlagIni = 0;
      return -1;
     }
  //ESPERA EL PRIMER DATO PARA LUEGO EN EL PROXIMO , HACER EL CONTROL
  if(!FlagIni)
    {
     if((Dato&0xf000) == 0x8000) AzAnt = Dato;
     DatoAnt = Dato;
     FlagIni = 1;
     return 0;
    }
  //CONTROLA UN DATO DE AZIMUT
  if((Dato&0xf000)  == 0x8000)
    {
	    AzAnt = AzAnt & 0x8FFF;
      //SI EL DATO ES MENOR O IGUAL AL DATO DE AZIMUT ANT
      //Y NO ES UN CAMBIO DE SECTOR
      if(Dato <= AzAnt)
        {
		if(((AzAnt&0xff00) != 0x8F00) ||
	   	((Dato&0xff00)  != 0x8000))
	    	{
		 	if(AzAnt!=Dato){
		    	printf("%x   %x    ",AzAnt,Dato);
		    	printf("ERROR1                            ");
	        	}
		    AzAnt=Dato;
	     	ResetFifo();//<------------
	     	FlagIni = 0;
	     	return -1;
	    	}
		}
	//SI ES CAMBIO DE SECTOR, MANDA UN CARACTER
	//else  Buff[iwr++]= 'T';
      DatoAnt = AzAnt = Dato;
    }
   //SI EL DATO ANTERIOR ES UNA ALTURA, ESTE DEBE SER UN RANGO
   if(((DatoAnt&0xF000) == 0xC000)&&((Dato&0xF000) != 0x3000))
       {
	       printf("ERROR2   ");
          ResetFifo();
          //Buff[iwr++]= '5'; //error 5
          //iwr = ird = 0;
          FlagIni = 0;
          banderainic = 1;    //modif R01!!!
          return -1;
       }
    else DatoAnt = Dato;		//<--Agregado 10/03/2010

  return 0;

}

int Primario_eco(unsigned int lectura){
   //unsigned int retorno;

   int aux;
   int j;
   int dif;
   int modulo;
   int i;
   int aztfinal;
   int RangoAltura=0;
//printf(". ");
   if(((lectura & 0xf000)==0x8000)||((lectura & 0xf000)==0x3000)||((lectura & 0xf000)==0xC000)) /*Es lectura un dato valido?*/
   		{ /* lectura si es dato valido*/
    	if((lectura & 0xf000)==0x8000) /* es lectura un dato de azimuth?*/
    		{  /* DATO ES AZIMUTH */
    		//printf("%x  ",lectura);
    		aztprom=lectura;
    		sectoractual= ( lectura & 0x0f00 ) >> 8;
            if( sectoractual != sectoranterior )
      			{
	      			//printf("%x   ",sectoractual);
       			sectoranterior=sectoractual;
       			cabecera(sectoractual);
      			}
       		}/*fin de "lectura es un azt"*/
    	else /* es un rango o altura*/
    		{
     		if ((lectura & 0xf000)==0x3000) /*Si es rango*/
     			{
	     			lectura = ((lectura & 0x3FFF) + 19);   //1-abr-12   //Eliminar al corregir error de FPGA
	     			if(lectura <= 0x3FFF){    //Procesa si esta dentro del rango   //1-abr-12

	     		blancos(aztprom,lectura,0);
          			}
	          	}
	        }
    	}

 return 0;
}

void cabecera(char sect){
  int indloc;
   unsigned char aux;
	/*
	Buff[iwr++] = 0xff;
	Buff[iwr++] = 0xff;
	Buff[iwr++] = (header[2] & 0xf0)|sect;
	Buff[iwr++] = 0x0;
	*/

   header[2]=(header[2] & 0xf0)|sect;

	for(indloc=0;indloc<=3;indloc++)
      {
   		Buff[iwr++] = header[indloc];
   		//++++++++++++++++++++++++++++++++++++++
      	CabBuffUDP[iwr_u++]=header[indloc];
      	//++++++++++++++++++++++++++++++++++++++
      }
   SectorActual = (unsigned char)sect;

       /* outbyte(ier,txen);      */

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

       //reordenamiento de bytes en el buffer
       aux=CabBuffUDP[3];
       CabBuffUDP[3]=CabBuffUDP[2];
       CabBuffUDP[2]=aux;
       //envio por UDP
       //if (WriteSocket(iSendSock, CabBuffUDP, 4, NET_FLG_BROADCAST) < 0)//error
	     //printf("Error on NetWrite %d bytes: %s\n", 4, Err(iNetErrNo));
       iwr_u=0;

}

void blancos(unsigned int az, unsigned int rng, unsigned int Altr){
 unsigned char Sector = (unsigned char) ((az&0x0f00)>>8);
 unsigned char SectAnt = (unsigned char) (abs(SectorActual - 1));
 unsigned char aux;
//cprintf("1=%x   2=%x   3=%x   ",Sector,SectAnt,SectorActual);
 //cprintf("A=%x  ",Altr);
 //Version WP101
 if((Sector == SectorActual)||(Sector == SectAnt))
  {
   Buff[iwr++] = az & 0x00ff;
   //cprintf("%x  ",Buff[iwr]);
   Buff[iwr++] = (az & 0xff00)>>8;
   //cprintf("%x  ",Buff[iwr]);
   Buff[iwr++] = rng & 0x00ff;
   //cprintf("%x  ",Buff[iwr]);
   Buff[iwr++] = (rng & 0xff00)>>8;
   //cprintf("%x  ",Buff[iwr]);
   Buff[iwr++] = Altr & 0x00ff;
   //cprintf("%x  ",Buff[iwr]);
   Buff[iwr++] = ((Altr & 0x0F00)>>8)+ 0xC0;
   //cprintf("%x  ",Buff[iwr]);
//++++++++++++++++++++++++++++++++++++++++++++++++++++

   PlotBuffUDP[iwr_u++] = az & 0x00ff;
   PlotBuffUDP[iwr_u++] = (az & 0xff00)>>8;
   PlotBuffUDP[iwr_u++] = rng & 0x00ff;
   PlotBuffUDP[iwr_u++] = (rng & 0xff00)>>8;
   PlotBuffUDP[iwr_u++] = Altr & 0x00ff;
   PlotBuffUDP[iwr_u++] = ((Altr & 0x0F00)>>8)+ 0xC0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++
  }
//++++++++++++++++++++++++++++++++++++++++++++++++++++
//reordenamiento de bytes en el buffer

for(iwr_u=0;iwr_u<=5;iwr_u=iwr_u+2)
{
aux=PlotBuffUDP[iwr_u];
PlotBuffUDP[iwr_u]=PlotBuffUDP[iwr_u+1];
PlotBuffUDP[iwr_u+1]=aux;
}

/////////////////////////////////////////////////////
//////////////////////////SOCKETE////////////////////
void die(char *s)
{
    perror(s);
    exit(1);
}
    struct sockaddr_in si_other;

    int s, i, slen = sizeof(si_other);
    char buf[BUFLEN];
    char *message="hola from server";//[BUFLEN];
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }

    // zero out the structure
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);

    if (inet_aton(SERVER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }

    //bind socket to port
    /*if( bind(s , (struct sockaddr*)&si_other, sizeof(si_other) ) == -1)
    {
        die("bind");
    }
    else
    {
        printf ("Success Bind");
    }*/


  //  while(1)
    //{
        //printf("Enter message : ");
        //gets(message);
        //message[0] = "tu pendex";

        //send the message
        //if (sendto(s, (const char *)message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen)==-1)
        if (sendto(s, PlotBuffUDP, 6 , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
        else
        {
            printf ("Success Send");
        }

    //}

//close(s);
    //return 0;
    close(s);





////////////////////FIN SOCKETE//////////////////////
/////////////////////////////////////////////////////

//envio por UDP
//if (WriteSocket(iSendSock, PlotBuffUDP, 6, NET_FLG_BROADCAST) < 0)//error
//	    printf("Error on NetWrite %d bytes: %s\n", 4, Err(iNetErrNo));
       iwr_u=0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}

int Primario(unsigned int lectura){
   //unsigned int retorno;

   int ss,tt,uu;
   int aux;
   int j;
   int dif;
   int modulo;
   int i;
   int aztfinal;
   int RangoAltura=0;

   dif=(tolerancia>>1)+1;   /*dif = maxima distancia entre dos ecos para considerarlo un mismo blanco*/
   //retorno=0;
   //lectura=inword(fifo);   /*lectura carga el primer dato disponible en fifo*/

   if(((lectura & 0xf000)==0x8000)||((lectura & 0xf000)==0x3000)||((lectura & 0xf000)==0xC000)) /*Es lectura un dato valido?*/
   { /* lectura si es dato valido*/
    if((lectura & 0xf000)==0x8000) /* es lectura un dato de azimuth?*/
    {  /* DATO ES AZIMUTH */
     PrsAltura=0;
     if(((lectura < (azimuth+4))&&((lectura > azimuth)||(lectura < 0x8003)))||(banderainic == 1))
     { /*Esta el azimut actual en un entorno del azimut anterior?*/
      banderainic=0;
      azimuth=lectura;
      bandera_azt=1;
      sectoractual= ( lectura & 0x0f00 ) >> 8;
      if( sectoractual != sectoranterior )
      {
       sectoranterior=sectoractual;
       cabecera(sectoractual);
      }
      if (bandera==0)
      {
       for(j=0;j<indice;j++)
       {
	modulo=(rangoinic-prs[j].rangoinic);
	if (modulo<0)
	{
	 modulo=~modulo;
	 modulo++;
	}
	if(dif>modulo)
	{
	 dif=modulo;       /*encuentra la menor distancia*/
	 aux=j;
	}
       }
       if(dif<=(tolerancia>>1))  /*hay correlacion*/
       {
	prs[aux].auscons=0;
       }
       else /*crear nuevo blanco*/
       {
			//cprintf("dif=%x   ",dif);
	       //cprintf("crea nvo bco.2 Azt_inic=%x   ",aztant);
	prs[indice].rangoinic=rangoinic;
	prs[indice].aztinic=aztant;
	prs[indice].auscons=0;
	prs[indice].Altura=0;
	indice++;
       }
      } /*fin bandera=0*/

      bandera=1;
      aztant=lectura&0x0fff;
      for(j=0;j<indice;j++)
      {
       prs[j].auscons++;
       if(prs[j].auscons>=max_aus_cons)
       {
	aztfinal=aztant-max_aus_cons;
	aztprom=(prs[j].aztinic+aztfinal)>>1;
	if(aztfinal<prs[j].aztinic)
	{
	 aztprom=(aztprom+(max_vuelta>>1) & 0x0fff);
	 aztfinal=aztfinal+4096;
	}
	//cprintf("-----%x  %x      -----",aztfinal,prs[j].aztinic);
	if((aztfinal-prs[j].aztinic)>=blancocorto)
	{
	 aztprom=aztprom|0x8000;
//	 blancos(aztprom,prs[j].rangoinic,prs[j].Altura);	//HABILITAR PARA WESTINGHOUSE????????????????????
//	}													//HABILITAR PARA WESTINGHOUSE????????????????????

	//SOLO PARA 113 -----------------------------------------------------------------------------------------------
	//Antes de poner el blanco en el buffer de transmisi�n
	 //se verifica si hay dato de altura y si �ste es asociable
	 //al dato a transmitir
	 if(Flag_alt){//Hay dato de altura
	 //cprintf("AP=%x  ",(aztprom & 0x0F00)>>8);
	 			//Se compara con datos en el sector anterior, actual y posterior al de altura
	 			if((sector_alt==((aztprom & 0x0F00)>>8))|                //sector actual
	 			   (sector_alt==((((aztprom & 0x0F00)>>8)-1) & 0x0F))|   //sector anterior
	 			   (sector_alt==((((aztprom & 0x0F00)>>8)+1) & 0x0F))){  //sector posterior

	 					//Se calcula la posici�n X-Y del blanco
	 					X_bco =  (float)(prs[j].rangoinic & 0x0fff)*0.238356 * sin((float)(aztprom)*2*PI/4096.0);//En MN
  						Y_bco =  (float)(prs[j].rangoinic & 0x0fff)*0.238356 * cos((float)(aztprom)*2*PI/4096.0);//En MN
  						//cprintf("Xb=%x  ",prs[j].rangoinic  & 0x0fff);
  						//cprintf("X=%f  Y=%f  ",X_bco,Y_bco);
  						//cprintf("X=%f  Y=%f  ",X_alt,Y_alt);
  						//Se calcula la distancia entre blanco y dato-altura
  						Dist =  sqrt((X_bco - X_alt)*(X_bco - X_alt) + (Y_bco - Y_alt)*(Y_bco - Y_alt));//En MN
  						printf("Dist=%d  ",Dist);
  						//Se verifica si est� dentro de la distancia aceptable para su asociaci�n
 						if(Dist<Dist_max){//Se asocia dato de altura a plot y se manda a Buffer de tx
	 							blancos(aztprom,prs[j].rangoinic,alt_alt);
	 							printf("EXITO  ");
	 							Flag_alt=0;
	 							}
 					else{	//No se asocia dato de altura a plot (fuera de distancia); se manda a Buffer de tx
	 					blancos(aztprom,prs[j].rangoinic,prs[j].Altura);
	 					Flag_alt=0;
 						}
	 				}
	 			else //No se asocia dato de altura a plot (fuera de sector); se manda a Buffer de tx
	 				blancos(aztprom,prs[j].rangoinic,prs[j].Altura);
	 				//Flag_alt=0;
 				}
	 else ////No hay dato de altura; se manda a Buffer de tx
	 	blancos(aztprom,prs[j].rangoinic,prs[j].Altura);

	}
	//FIN SOLO PARA 113------------------------------------------------------------------------------------------------------------------

	for (i=j;i<indice;i++)
	{
	 prs[i]=prs[i+1];
	}
	indice--;
	j--;
       }
      }
     }
     else
     {
      banderainic = 1;
      if(lectura==azimuth)
      {
       bandera_azt=0;
      }
      else
      {
       indice=0;
      }
     }
    }/*fin de "lectura es un azt"*/
    else /* es un rango o altura*/
    {
     if ((lectura & 0xf000)==0x3000) /*Si es rango y esta fuera de las 15 MN aproxim*/
     {
	     //Agregado el 4-oct-12   NO PROCESA DENTRO DE LAS 15 MN
	     //if((lectura & 0x0fff)>=0x41)
	     //Modificado el 3-jun-14
	     if((lectura & 0x0fff)>=AREA_INHIBICION)
	     {
	     /*
	     //Incrementa celda correspondiente del mapa de clutter siempre que el rango leiodo de la FIFO no sea el que corresponde al dato de altura
	     if(!PrsAltura){
	     	Mapa_Clutter[vuelta_mapa][(azimuth & 0x0FFF)/64][(lectura & 0x0FFF)/8]+=1;
	     	//printf("[%d][%d][%d]= %d --> %d   MCT= %d \n",vuelta_mapa,(azimuth & 0x0FFF)/32,(lectura & 0x0FFF)/8,Mapa_Clutter[vuelta_mapa][(azimuth & 0x0FFF)/32][(lectura & 0x0FFF)/8],Mapa_Umbral[(azimuth & 0x0FFF)/32][(lectura & 0x0FFF)/8],Mapa_Clutter_Total[(azimuth & 0x0FFF)/32][(lectura & 0x0FFF)/8]);
	     	}
	     */

	     //lectura = (lectura & 0x0FFF);  //4-abr-12
	     //Si el rango es menor a 5 MN (cuenta RC=21) no procesa!!!!   //4-abr-12
	//SOLO PARA 113-->     if(((lectura & 0x0FFF) < 21)||((lectura & 0x0FFF) > 0x0FFF)){}  //Verifica, adem�s, que este dentro del rango  //1-abr-12
	//SOLO PARA 113 --> else{
	     //cprintf("%x  ",lectura);
	     //Si es dato de altura y no excedi� el limite de clutter
      //if((PrsAltura==1)&&(Mapa_Umbral[(azimuth & 0x0FFF)/64][(lectura & 0x0FFF)/8]))
      if(PrsAltura==1)
        {
        PrsAltura=0;
	    bandera=1;
        RangoAltura=lectura;
        //cprintf("%x  ",RangoAltura);
        dif=(tolerancia>>1)+1;
        for(j=0;j<indice;j++)
           {
	       modulo=(RangoAltura-prs[j].rangoinic);
	       if (modulo<0)
	         {
	         modulo=~modulo;
	         modulo++;
	         }
	       if(dif>modulo)
	         {
	         dif=modulo;       /*encuentra la menor distancia*/
	         aux=j;
	         }
           }
        if(dif<=(tolerancia>>1))  /*hay correlacion de rango altura*/
           {
	       prs[aux].Altura=AlturaTemp;
	       //cprintf("xx   ");
           }
        return(0);
        }
        //Si es rango y no excedi� el limite de clutter
       //if ((bandera_azt==1)&&(Mapa_Umbral[(azimuth & 0x0FFF)/64][(lectura & 0x0FFF)/8]))
       if (bandera_azt==1)
        {
        rangoactual= lectura;
        if (bandera==1)
          {
	      rangoinic=rangoactual;
	      bandera=0;
          }
        else /* si bandera=0*/
          {
	      if(rangoactual-rangoinic<=tolerancia)
	        {
	        }
	      else /*si esta fuera de la tolerancia ==> otro blanco*/
	        {
	        dif=(tolerancia>>1)+1;
	        for(j=0;j<indice;j++)
	          {
	           modulo=(rangoinic-prs[j].rangoinic);
	           if (modulo<0)
	             {
	             modulo=~modulo;
	             modulo++;
	             }
	           if(dif>modulo)
	             {
	             dif=modulo;       /*encuentra la menor distancia*/
	             aux=j;
	             }
	          }
	        if(dif<=( tolerancia>>1))  /*hay correlacion*/
	          {
	          rangoinic=rangoactual;
	          prs[aux].auscons=0;
	          }
	        else /*crear nuevo blanco*/
	          {
		          //cprintf("crea nvo bco.1 Azt_inic=%x   ",aztant);
	          prs[indice].rangoinic=rangoinic;
	          prs[indice].aztinic=aztant;
	          prs[indice].auscons=0;
	          prs[indice].Altura=0;
	          rangoinic=rangoactual;
	          indice++;
	          }
	        }
          }
        }
    	}
       //SOLO PARA 113 --> }
       } /* fin if 'es un rango'*/		//<-------------------------------------------------------------------------------
     else  /*es dato de altura*/
       {
	       //No excedio limite clutter

//	 if (Mapa_Umbral[(azimuth & 0x0FFF)/64][(lectura & 0x0FFF)/8]){
        	PrsAltura=1;
        	AlturaTemp=lectura&0x00ff;
        	//cprintf("%x  ",AlturaTemp);
//    		}

        }
       } /*fin else rango-altura*/
   }
 return 0;/////!!!! ver
}






int main (){
///Inicio declaración de variables del main:///
  unsigned int Dato,xxx,xxx_ant;
  unsigned long kk;
  unsigned char jumper,swich,inhibicion;

//Apertura de la memoria del HPS, el mapeo y el direccionamiento de los vectores D y A
  fd=open("/dev/mem",(O_RDWR|O_SYNC));
  virtual_base=mmap(NULL,REG_SPAN,(PROT_READ|PROT_WRITE),MAP_SHARED,fd,REG_BASE);
  D_addr=virtual_base+LECTURAESC_BASE;
  A_addr=virtual_base+ESCRITURA_BASE;

	printf("\nSe realizo exitosamente la apertura del puente\n");

  //llamada a Funciones
  ResetFifo();
  printf("\nFifo reseteada\n");
  InicializaPr();
	printf("Parametros inicializados\n");
	printf("Comienza lectura de jumpers\n");
  //Lee estado de jumpers

 	*(uint32_t *)A_addr=0x2300; //-/ jumper=inp(0x300);
  usleep(1000);
  jumper=*(uint32_t *)D_addr;
 	 printf("%x  ",jumper);
	// *(uint32_t *)A_addr=0x2310;
	// usleep(1000);
	// printf("\n%i\n",*(uint32_t *)D_addr);
	// *(uint32_t *)A_addr=0x2330;
	// usleep(1000);
	// printf("\n%i\n",*(uint32_t *)D_addr);
 	 //JMP0
 	eco=jumper & 0x01;
 	if(eco)
  {
  *(uint32_t *)A_addr=0x4354; //-/ outp(0x354,0x01);//Anula Detector Ventana
  *(uint32_t *)D_addr=0x01;
    }
  else
  {
  *(uint32_t *)A_addr=0x4354;
  *(uint32_t *)D_addr=0x00;  //-/ outp(0x354,0x00);//Habilita Detector Ventana
  printf("A=%x y D=%x \n",A_addr,D_addr);
  }
 	//JMP1
 	//Manejo de umbral del detector ventana
 	swich=(jumper & 0x02)>>1;
 	if(swich)
  {
  *(uint32_t *)A_addr=0x4352; //-/ outp(0x352,0x01);//Manejo del Umbral a traves del DipSwitch
  *(uint32_t *)D_addr=0x01;
  printf("A=%x y D=%x ",A_addr,D_addr);
  }
  else
  {
     *(uint32_t *)A_addr=0x4352; //-/ outp(0x352,0x30);//Manejo del Umbral a partir del valor cargado en el nibble superior; en este caso 0x3.
     *(uint32_t *)D_addr=0x30;
  }
 	 //JMP5
 	 //Manejo del area de inhibicion
 	 inhibicion=(jumper & 0x10)>>4;
 	 if(inhibicion)
   {
      AREA_INHIBICION = 0x1E; //7 MN
   }
 	 else
   {
     AREA_INHIBICION = 0x0D; //3 MN
   }
  	printf("ECO=%x  SWITCH=%x  \n",eco,swich);

      while(1)              //Corazón del programa
      {
        if(!ChkFlag())
        {
          if(!GetDato(&Dato))
          {
            if(eco)
            {//Procesa ecos. Detector ventana anulado
    	   				Primario_eco(Dato);
                printf("E  ");
       					}
    	   		else
            {              //Procesa plots. Detector ventana habilitado
    	   			Primario(Dato);
    	   			printf("P  ");
          }
        }


      }
return 0;
}
}
