/**
  ******************************************************************************
  * @file    sh32f2xx_macp.h
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provides MACP APIs
  ******************************************************************************
  * @attention
  *
  * SINOWEALTH IS SUPPLYING THIS SOFTWARE FOR USE EXCLUSIVELY ON SINOWEALTH'S 
  * MICROCONTROLLER PRODUCTS. IT IS PROTECTED UNDER APPLICABLE COPYRIGHT LAWS. 
  * THIS SOFTWARE IS FOR GUIDANCE IN ORDER TO SAVE TIME. AS A RESULT, SINOWEALTH 
  * SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
  * WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH FIRMWARE AND/OR
  * THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN 
  * CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2017 Sinowealth</center></h2>
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SH32F2xx_MACP_H
#define __SH32F2xx_MACP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_sa0_lib.h"

/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/** @addtogroup MACP_MODULE
  * @{
  */ 

/** @defgroup MACP_Group_Constant  Public Constants
  * @{
  */ 

/*! IQ26_1_K constant */     
#define IQ26_1_K          ((IQ26)(0x026DD3B6))          
/*! IQ24_1_K constant */     
#define IQ24_1_K          ((IQ24)(0x009B74EE))
/*! IQ15_1_K constant */     
#define IQ15_1_K          ((IQ15)(0x004DBA))

/*! IQ26_1 constant */     
#define IQ26_1            ((IQ26)(0x3FFFFFF))          
/*! IQ24_1 constant */     
#define IQ24_1            ((IQ24)(0xFFFFFF))
/*! IQ15_1 constant */     
#define IQ15_1            ((IQ15)(0x7FFF))


/*! PI CONSTANT */
#define PI  3.141593

     
/*!  CORDIC K Adjust*/   
typedef enum
{
   CORDIC_KADJ_NONE = 0,  /*!< Result do nothing */
   CORDIC_KADJ_DIVK  = 1  /*!< Result /= K */   
}CORDIC_KADJ_Type;
/*! check CORDIC K adjust type */
#define IS_CORDIC_KADJ_TYPE(type) (((type) == CORDIC_KADJ_NONE) || ((type) == CORDIC_KADJ_DIVK))
     
/*!  CORDIC XY Adjust */   
typedef enum
{
   CORDIC_XYMRS_NONE = 0, /*!< X,Y do nothing */
   CORDIC_XYMRS_DIV2 = 1  /*!< X/=2 Y/=2 */   
}CORDIC_XYMRS_Type;
/*! check CORDIC XY adjust type */
#define IS_CORDIC_XYMRS_TYPE(type) (((type) == CORDIC_XYMRS_NONE) || ((type) == CORDIC_XYMRS_DIV2))

/*!  CORDIC data Format */   
typedef enum
{
   CORDIC_FMT_IQ26 = 0, /*!< IQ26 Format */
   CORDIC_FMT_IQ24 = 1, /*!< IQ24 Format */
   CORDIC_FMT_IQ15 = 2  /*!< IQ15 Format */
}CORDIC_FMT_Type;
/*! check CORDIC data format */
#define IS_CORDIC_FMT_TYPE(type) (((type) == CORDIC_FMT_IQ26) \
                               || ((type) == CORDIC_FMT_IQ24) \
                               || ((type) == CORDIC_FMT_IQ15))

/*!  CORDIC MODE */   
typedef enum
{
   CORDIC_MODE_ROTATE = 0, /*!< rotate conversion*/
   CORDIC_MODE_VECTOR = 1  /*!< vector conversion*/
}CORDIC_MODE_Type;
/*! check CORDIC transform mode */
#define IS_CORDIC_MODE_TYPE(type) (((type) == CORDIC_MODE_ROTATE) || ((type) == CORDIC_MODE_VECTOR))



/*!  IQDIV sign flag */   
typedef enum
{
   IQDIV_UNSIGNED = 0, /*!< unsigned IQ data*/
   IQDIV_SIGNED   = 1  /*!< signed IQ data is unsigned*/   
}IQDIV_Sign_Type;
/*! check IQDIV sign type */
#define IS_IQDIV_SIGN_TYPE(type) (((type) == IQDIV_UNSIGNED) || ((type) == IQDIV_SIGNED))

/*!  IQDIV approximate method */   
typedef enum
{
   IQDIV_CMOD_TRUNC = 0, /*!< use trucation method of last data */
   IQDIV_CMOD_ROUND = 1  /*!< use roung method of last data */
}IQDIV_CMOD_Type;
/*! check IQDIV CMOD type */
#define IS_IQDIV_CMOD_TYPE(type) (((type) == IQDIV_CMOD_TRUNC) || ((type) == IQDIV_CMOD_ROUND))


/*!  SVPWM  Type */   
typedef enum
{
    SVPWM_SVTYPE_7 = 0, /*!< 7 segment type */
    SVPWM_SVTYPE_5 = 1  /*!< 5 segment type */  
}SVPWM_SVTYPE_Type;
/*! check SVPWM  Type */
#define IS_SVPWM_SVTYPE_TYPE(type) (((type) == SVPWM_SVTYPE_7) || ((type) == SVPWM_SVTYPE_5))


/**
  * @}
  */ 

/** @defgroup MACP_Group_Types  Public Types
  * @{
  */   

/*! define datatype IQ30 */
typedef int32_t   IQ30;  
/*! define datatype IQ26 */
typedef int32_t   IQ26;
/*! define datatype IQ24 */
typedef int32_t   IQ24;
/*! define datatype IQ15 */
typedef int32_t   IQ15;
/*! define datatype IQN */
typedef int32_t   IQN;


/*! @struct  CORDIC_TypeDef
  *  this structure for CORDIC device registers
  */      
typedef struct {
    union {
        __IO  uint32_t  V32;                                /*!< 0000H */
        struct {
            __IO  uint32_t  RUN       : 1;  /*!< run control bit */
            __I   uint32_t  OVF       : 1;  /*!< overflow flag */
            __IO  uint32_t  KADJ      : 1;  /*!< K value adjust @ref CORDIC_KADJ_Type*/
            __IO  uint32_t  XYMRS     : 1;  /*!< result adjust @ref CORDIC_XYMRS_Type*/
                  uint32_t  rev0      : 1;  /*!< reserved*/
            __IO  uint32_t  FORMAT    : 2;  /*!< Data format: @ref CORDIC_FMT_Type*/
            __IO  uint32_t  MODE      : 1;  /*!< Mode rotation or vector @ref CORDIC_MODE_Type*/
                  uint32_t  rev1      :24;  /*!< reserved*/
        }BIT;
    }CSR;                               /*!< 0000H */
    __IO uint32_t  OPRDX;               /*!< operation data X */
    __IO uint32_t  OPRDY;               /*!< operation data Y */
    __IO uint32_t  OPRDZ;               /*!< operation data Z */
}CORDIC_TypeDef;
/*! define CORDIC0 device pointer*/
#define CORDIC0  ((CORDIC_TypeDef *)&(MACP->CORDCSR0.V32))
/*! define CORDIC1 device pointer*/
#define CORDIC1  ((CORDIC_TypeDef *)&(MACP->CORDCSR1.V32))
/*! check CORDIC device pointer */
#define IS_CORDIC_MODULE(CORDICx) ((CORDICx == CORDIC0) || (CORDICx == CORDIC1))

/*! @struct  CORDIC_InitTypeDef
  *  CORDIC initial structure
  */  
typedef struct {
    uint32_t  RunCtrl          : 1;  /*!< run control bit : ENABLE or DISABLE */
    uint32_t  rev0             : 1;  /*!< reserved */
    uint32_t  KAdjust          : 1;  /*!< K value adjust @ref CORDIC_KADJ_Type*/
    uint32_t  XYAmplitude      : 1;  /*!< X,Y amplitude adjustment @ref CORDIC_XYMRS_Type*/
    uint32_t  rev1             : 1;  /*!< reserved*/
    uint32_t  Format           : 2;  /*!< Data format: @ref CORDIC_FMT_Type*/
    uint32_t  TransformMode    : 1;  /*!< Transform method: rotation or vector @ref CORDIC_MODE_Type*/
}CORDIC_InitTypeDef;

/*! @struct  CORDIC_DataTypeDef
  *  this structure for IQDIV device registers
  */  
typedef struct{
    int32_t X; /*!< Cordic opeartion data or result X */
    int32_t Y; /*!< Cordic opeartion data or result Y */
    int32_t Z; /*!< Cordic opeartion data or result Z */
}CORDIC_DataTypeDef;

/*! @struct  IQRotate_TypeDef
  *  this structure for IQRotate parameters
  */  
typedef struct{
    IQN    X;      /*!< X axis value */
    IQN    Y;      /*!< Y axis value */
    IQN    Angle;  /*!< rotate angle */
}IQRotate_TypeDef;

/*! @struct  IQVector_TypeDef
  *  this structure for IQVector parameters
  */ 
typedef struct{
    IQN    X;    /*!< X axis value */
    IQN    Y;    /*!< Y axis value */
    IQN    MOD;  /*!< root mean square */
    IQN    ATAN; /*!< direction of vector */
}IQVector_TypeDef;

/*! @struct  IQSinCos_TypeDef
  *  this structure for IQSinCos parameters
  */ 
typedef struct{
    IQN    Theta;   /*!< operation data: theta */
    IQN    SIN;     /*!< result: sine */
    IQN    COS;     /*!< result: cosine */
}IQSinCos_TypeDef; 

/*! @struct  PARK_TypeDef
  *  Park transform parameters
  */  
typedef struct {
	IQN  Alpha; /*!< operation data: Ialpha */
	IQN  Beta;  /*!< operation data: Ibeta  */
	IQN  Theta; /*!< operation data: theta  */   
    IQN  Id;	/*!< output : Id */		
    IQN  Iq;	/*!< output : Iq */		
}PARK_TypeDef;	

/*! @struct  IPARK_TypeDef
  *  IPark transform parameters
  */  
typedef struct {
    IQN  Ud;	/*!< operation data: Ud*/		
    IQN  Uq;	/*!< operation data: Uq */		
	IQN  Theta; /*!< operation data: theta */   
	IQN  Alpha; /*!< output : Ualpha */
	IQN  Beta;  /*!< output : Ubeta */
}IPARK_TypeDef;	


/*! @struct  IQDiv_DataTypeDef
  *  IQ division parameters
  */  
typedef struct {
    IQN Dividend;  /*!< operation data : Dividend*/
    IQN Divisor;   /*!< operation data : Divisor*/
    IQN Quotient;  /*!< result: quotient*/
}IQDiv_DataTypeDef;	

/*! @struct  IQDIV_TypeDef
  *  this structure for IQDIV device registers
  */      
typedef struct {
    union {
        __IO  uint32_t  V32;                /*!< 0000H */
        struct {
            __IO  uint32_t  RUN       : 1;  /*!< run control bit */
            __IO  uint32_t  SIGN      : 1;  /*!< sign mode select bit */
            __IO  uint32_t  SAT       : 1;  /*!< saturated flag */
            __IO  uint32_t  Q         : 5;  /*!< Q format selection */
            __IO  uint32_t  CMOD      : 1;  /*!< approximate method for result */
                  uint32_t  rev       :23;  /*!< reserved */
        }BIT;
    }CSR;                             /*!< Control and status */
    __IO uint32_t  DIVDND;            /*!< Dividend */
    __IO uint32_t  DIVSOR;            /*!< Divisor */
    __IO uint32_t  RLT;               /*!< Result */
}IQDIV_TypeDef;
/*! define IQDIV0 device pointer*/
#define IQDIV0  ((IQDIV_TypeDef *)&(MACP->IQDIVCSR0.V32))
/*! define IQDIV1 device pointer*/
#define IQDIV1  ((IQDIV_TypeDef *)&(MACP->IQDIVCSR1.V32))
/*! check IQDIV device pointer*/
#define IS_IQDIV_MODULE(DIVx) ((DIVx == IQDIV0) || (DIVx == IQDIV1))

/*! @struct  IQDIV_InitTypeDef
  *  IQDIV initial value structure
  */ 
typedef struct{
    uint32_t  RunCtrl           : 1;  /*!< run control bit */
    uint32_t  SignMode          : 1;  /*!< sign mode select bit @ref IQDIV_Sign_Type */
    uint32_t  SaturateFlag      : 1;  /*!< Saturated flag : SET or RESET */
    uint32_t  QFormat           : 5;  /*!< Q format selection 0~31 */
    uint32_t  ApproximateMethod : 1;  /*!< approximate method for result @ref IQDIV_CMOD_Type */
}IQDIV_InitTypeDef;    

/*! @struct  SVPWM_DataTypeDef
  *  this structure for IQDIV device registers
  */  
typedef struct {
    SVPWM_SVTYPE_Type SVType; /*!< Input SVType :SVPWM_SVTYPE_7 or SVPWM_SVTYPE_5*/
    IQ24  UAlpha;	 /*!< Input uAlpha */
	IQ24  UBeta;	 /*!< Input uBeta */		
	IQ24  Ta;        /*!< transform result Ta */
	IQ24  Tb;        /*!< transform result Tb */ 
	IQ24  Tc;        /*!< transform result Tc */ 
    uint8_t SVIQN;   /*!< IQ data format 0~31 */
    uint8_t Sector;  /*!< transform result sector */ 
    uint8_t rev[2];  /*!< reserved for word alignment */
}SVPWM_DataTypeDef;



/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/** @defgroup MACP_Group_Macro  Public Macros
  * @{
  */ 

/** @defgroup MACP_IQMATH_Macro  IQ Match Macros
  * @{
  */ 

/** 
  *@brief  convert data to IQ26
  */
#define IQ26(X)   ((IQ26)(X*67108864))
/** 
  *@brief  convert data to IQ24
  */
#define IQ24(X)   ((IQ24)(X*16777216))
/** 
  *@brief  convert data to IQ15
  */
#define IQ15(X)   ((IQ15)(X*32768))

/**
  * @verbatim   Example
       convert float (0.4) to IQ24 format data
       IQ24 X = IQn(0x666666,24);
       printf("Float:0.4 to IQ24:%X\n",(unsigned int)X);
  * @endverbatim  
  *@brief  convert data to IQn q=0~31
  */
#define IQn(x,q)  ((IQN)(x*(1<<(q))))


/**
  * @verbatim   Example
       convert IQ24 format data to float data
       float f = IQnToFloat(0x666666,24);
       printf("IQ24:0x666666 to float:%f\n",f);
  * @endverbatim  
  *@brief  convert IQn data to float
  */
#define IQnToFloat(x,q) (((float)(((int32_t)((x)<<(31-q)))>>(31-q)))/((int32_t)1<<(q)))

/** 
  *@brief  IQ30 format data multiplication
  */
#define IQ30mpy(X, Y)   ((IQ30)(((int64_t)(((int64_t)(X))*((int64_t)(Y)))) >> 30))

/** 
  *@brief  IQ26 format data multiplication
  */
#define IQ26mpy(X, Y)   ((IQ26)(((int64_t)(((int64_t)(X))*((int64_t)(Y)))) >> 26))

/** 
  *@brief  IQ24 format data multiplication
  */
#define IQ24mpy(X, Y)   ((IQ24)(((int64_t)(((int64_t)(X))*((int64_t)(Y)))) >> 24))

/** 
  *@brief  IQ15 format data multiplication
  */
#define IQ15mpy(X, Y)   ((IQ24)(((int64_t)(((int64_t)(X))*((int64_t)(Y)))) >> 15))



/** 
  *@brief  IQ Rotate function
  */
#define IQRotate(CORDICx,SinCosData) CORDIC_IQRotate(CORDICx,SinCosData)

/** 
  *@brief  IQ Vector function
  */
#define IQVector(CORDICx,SinCosData) CORDIC_IQVector(CORDICx,SinCosData)

/** 
  *@brief  IQ Sin Cos function
  */
#define IQSinCos(CORDICx,SinCosData) CORDIC_IQSinCos(CORDICx,SinCosData)

/** 
  *@brief  IQ IPark Conversation
  */
#define IQIPark(CORDICx,IParkData) CORDIC_IQIPark(CORDICx,IParkData)

/** 
  *@brief  IQ Park Conversation
  */
#define IQPark(CORDICx,ParkData) CORDIC_IQPark(CORDICx,ParkData)


/** 
  *@brief  IQ arctan = arctan(X);
  */
#define IQAtan(CORDICx,X) CORDIC_IQAtan(CORDICx, (X))

/** 
  *@brief  IQ arctan = arctan(Y/X);
  */
#define IQAtan2(CORDICx,X,Y) CORDIC_IQAtan2(CORDICx, (X), (Y))


/** 
  *@brief  IQ division = X / Y;
  */
#define IQDiv(CORDICx,X,Y) IQDIV_IQDiv(CORDICx, (X), (Y))


/** 
  *@brief  IQ SVPWM  Input: alpha, beta  Output:Ta,Tb,Tc,Sector
  *        svdata : SVPWM_DataTypeDef
  */
#define IQSvpwm(svdata) SVPWM_Transform(svdata)



/**
  * @}
  */ 

/** 
  *@brief  set CORDIC format use CORDIC_FMT_Type
  *   @arg CORDIC_FMT_IQ26
  *   @arg CORDIC_FMT_IQ24
  *   @arg CORDIC_FMT_IQ15
  */
#define CORDIC_SetFormat(CORDICx,fmt)     ((CORDICx)->CSR.BIT.FORMAT = fmt)

/** 
  *@brief set CORDIC mode use CORDIC_MODE_Type
  *   @arg CORDIC_MODE_ROTATE
  *   @arg CORDIC_MODE_VECTOR  
  */
#define CORDIC_SetMode(CORDICx,mode)     ((CORDICx)->CSR.BIT.MODE = mode)

/** 
  *@brief set CORDIC XYMRS use CORDIC_XYMRS_Type
  *   @arg CORDIC_XYMRS_NONE
  *   @arg CORDIC_XYMRS_DIV2  
  */
#define CORDIC_SetXYMRS(CORDICx,xymrs)     ((CORDICx)->CSR.BIT.XYMRS = xymrs)

/** 
  *@brief set CORDIC mode use CORDIC_KADJ_Type
  *   @arg CORDIC_KADJ_NONE
  *   @arg CORDIC_KADJ_DIVK  
  */
#define CORDIC_SetKADJ(CORDICx,kadj)     ((CORDICx)->CSR.BIT.KADJ = kadj)


/** 
  *@brief  check CORDIC busy flag
  */
#define CORDIC_IS_BUSY(CORDICx)     ((CORDICx)->CSR.BIT.RUN)

/** 
  *@brief  check CORDIC Overflow flag
  */
#define CORDIC_IS_OVERFLOW(CORDICx)  ((CORDICx)->CSR.BIT.OVF)

/** 
  *@brief  start CORDIC
  */
#define CORDIC_START(CORDICx)     (CORDICx)->CSR.BIT.RUN = 1


/** 
  *@brief  check iqdiv busy flag
  */
#define IQDIV_IS_BUSY(DIVx)     ((DIVx)->CSR.BIT.RUN )

/** 
  *@brief  start IQDIV
  */
#define IQDIV_START(DIVx)     (DIVx)->CSR.BIT.RUN = 1


/** 
  *@brief check IQDIV saturate flag  
  */
#define IQDIV_IS_SATURATE(DIVx) ((DIVx)->CSR.BIT.SAT)

/** 
  *@brief  clear IQDIV saturate flag
  */
#define IQDIV_CLEAR_SAT(DIVx)     (DIVx)->CSR.BIT.SAT = 0


/** 
  *@brief  get IQDIV result
  */
#define IQDIV_GET_RESULT(DIVx)  ((DIVx)->RLT)


/** 
  *@brief  start IQDIV
  */
#define SVPWM_IS_BUSY()     (MACP_SVCON_RUN_BIT == 1)


/** 
  *@brief  start IQDIV
  */
#define SVPWM_START()        (MACP_SVCON_RUN_BIT = 1)

      
/**
  * @}
  */ 

/* Exported functions --------------------------------------------------------*/ 
/** @addtogroup MACP_Group_Pub_Funcs
  * @{
  */     


/* CORDIC functions --------------------------------------------------------*/ 
/** @addtogroup  MACP_Group_Cordic_Funcs
  * @{
  */

/* CORDIC Module Init */
void CORDIC_Init(CORDIC_TypeDef* CORDICx, const CORDIC_InitTypeDef* InitCfg);

/*Fills each InitStruct member with its default value*/
void CORDIC_StructInit(CORDIC_InitTypeDef* InitStruct);

/* Deinitializes the CORDICx registers to their default reset */
void CORDIC_Reset(CORDIC_TypeDef* CORDICx);

/*  CORDIC transform then return the result */
bool_t CORDIC_Transform(CORDIC_TypeDef* CORDICx, CORDIC_DataTypeDef* CordicData);

/* start CORDIC transform then return immediately */
void CORDIC_Run(CORDIC_TypeDef* CORDICx, CORDIC_DataTypeDef* CordicData);
   
/* get CORDIC result */
bool_t CORDIC_GetResult(CORDIC_TypeDef* CORDICx, CORDIC_DataTypeDef* CordicData);

/* coordinate(X,Y) rotate  transform */
bool_t CORDIC_IQRotate(CORDIC_TypeDef* CORDICx, IQRotate_TypeDef* RotateData);

/* coordinate(X,Y) vector  transform */
bool_t CORDIC_IQVector(CORDIC_TypeDef* CORDICx, IQVector_TypeDef* VectorData);


/* caculate IQ24 format sin and cos */
bool_t CORDIC_IQSinCos(CORDIC_TypeDef* CORDICx,IQSinCos_TypeDef* SinCosData);


/*  Park transform */  
bool_t CORDIC_IQPark(CORDIC_TypeDef* CORDICx,PARK_TypeDef* ParkData);


/*  IPark transform */  
bool_t CORDIC_IQIPark(CORDIC_TypeDef* CORDICx,IPARK_TypeDef* IParkData);

/* = arctan(X) */
IQN CORDIC_IQAtan(CORDIC_TypeDef* CORDICx, IQN X);

/* = arctan(Y/X) */
IQN CORDIC_IQAtan2(CORDIC_TypeDef* CORDICx, IQN X, IQN Y);


/**
  * @}
  */

/* CORDIC functions --------------------------------------------------------*/ 
/** @addtogroup  MACP_Group_IQDiv_Funcs
  * @{
  */



/* IQDIV Module Init */
void IQDIV_Init(IQDIV_TypeDef* IQDIVx, const IQDIV_InitTypeDef* InitCfg);

/*Fills each InitStruct member with its default value*/
void IQDIV_StructInit(IQDIV_InitTypeDef* InitStruct);

/* Deinitializes the IQDIVx registers to their default reset */
void IQDIV_Reset(IQDIV_TypeDef* IQDIVx);


/* transform then return the result */
IQN IQDIV_IQDiv(IQDIV_TypeDef* IQDIVx, IQN Dividend, IQN Divisor);

/* start IQDIV then return immediately */
void IQDIV_Div_Run(IQDIV_TypeDef* IQDIVx,IQN Dividend, IQN Divisor);
   

/* IQ division then return the result */
bool_t IQDIV_IQDiv2(IQDIV_TypeDef* IQDIVx, IQDiv_DataTypeDef* DivData);

/**
  * @}
  */

/** @addtogroup  MACP_Group_SVPWM_Funcs 
  * @{
  */

/* SVPWM transform */
void SVPWM_Transform(SVPWM_DataTypeDef* SVPWMData);

/**
  * @}
  */
     
     
/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /*__SH32F2xx_MACP_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/
