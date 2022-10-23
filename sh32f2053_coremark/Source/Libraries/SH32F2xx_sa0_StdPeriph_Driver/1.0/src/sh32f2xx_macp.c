/**
  ******************************************************************************
  * @file    sh32f2xx_macp.c
  * @author  sinowealth
  * @version V1.0.0
  * @date    01-March-2017 
  * @brief   This file provide MACP APIs
  *         
  *  @verbatim
  *
  *          ===================================================================
  *                                  How to use this driver
  *          ===================================================================
  *          
  *
  *  @endverbatim
  *
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
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sh32f2xx_macp.h"
      
/** @addtogroup SH32F2xx_sa0_StdLib_Driver
  * @{
  */

/* MACP  Module----------------------------------------------------------*/
/** @defgroup MACP_MODULE  MACP 
*  MACP Module: Include Cordic, IQDIV and SVPWM units
  * @{
  */ 
        
/** @defgroup  MACP_Group_Pub_Funcs  Public Functions
 *  @brief   MACP Public Functions
 *
  * @{
  */
  

/** @defgroup  MACP_Group_Cordic_Funcs  CORDIC Functions
 *  @brief   Cordic Public Functions
 *
  * @{
  */



/**
  *@code Example
      const CORDIC_InitTypeDef CORDIC_CFG = {
                                            DISABLE,           // run control bit : ENABLE or DISABLE 
                                            0,                 // reserved
                                            CORDIC_KADJ_DIVK,  // K value adjust @ref CORDIC_KADJ_Type
                                            CORDIC_XYMRS_NONE, // X,Y amplitude adjustment @ref CORDIC_XYMRS_Type
                                            0,                 // reserved
                                            CORDIC_FMT_IQ24,   // Data format: @ref CORDIC_FMT_Type
                                            CORDIC_MODE_ROTATE,// Transform method: rotation or vector @ref CORDIC_MODE_Type
                                            };
       CORDIC_DataTypeDef CordicData;
       CORDIC_InitTypeDef InitCfg;
       CORDIC_StructInit(&InitCfg);
       InitCfg.KADJ   = CORDIC_KADJ_DIVK;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       InitCfg.MODE   = CORDIC_MODE_ROTATE;
       CORDIC_Init(CORDIC0, &InitCfg);
       CordicData.X = IQ24(0.4);
       CordicData.Y = IQ24(-0.4);
       CordicData.Z = IQ24(0.3);       
       if(CORDIC_transform(CORDIC0,&CordicData) == TRUE)
       {
            printf("X=%f  Y=%f  Z=%f\n",IQnToFloat(CordicData.X,24),IQnToFloat(CordicData.Y,24),IQnToFloat(CordicData.Z,24));
       }       
       CORDIC_Init(CORDIC1, &CORDIC_CFG);
       CordicData.X = IQ24(0.4);
       CordicData.Y = IQ24(-0.4);
       CordicData.Z = IQ24(0.3);       
       if(CORDIC_transform(CORDIC1,&CordicData) == TRUE)
       {
            printf("X=%f  Y=%f  Z=%f\n",IQnToFloat(CordicData.X,24),IQnToFloat(CordicData.Y,24),IQnToFloat(CordicData.Z,24));
       }       
  *@endcode     
  *@brief   CORDIC Module Init
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param InitCfg  Input CORDIC init values
  *@li BIT.KADJ
  *     @arg @b CORDIC_KADJ_NONE
  *     @arg @b CORDIC_KADJ_DIVK
  *@li BIT.XYMRS
  *     @arg @b CORDIC_XYMRS_NONE
  *     @arg @b CORDIC_XYMRS_DIV2
  *@li BIT.FORMAT
  *     @arg @b CORDIC_FMT_IQ26
  *     @arg @b CORDIC_FMT_IQ24
  *     @arg @b CORDIC_FMT_IQ15
  *@li BIT.MODE
  *     @arg @b CORDIC_MODE_ROTATE
  *     @arg @b CORDIC_MODE_VECTOR  
  *@retval  None
  */
void CORDIC_Init(CORDIC_TypeDef* CORDICx, const CORDIC_InitTypeDef* InitCfg)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_CORDIC_KADJ_TYPE(InitCfg->KAdjust));
    assert_param(IS_CORDIC_XYMRS_TYPE(InitCfg->XYAmplitude));
    assert_param(IS_CORDIC_FMT_TYPE(InitCfg->Format));
    assert_param(IS_CORDIC_MODE_TYPE(InitCfg->TransformMode));
    
    /* Initialize CORDIC Module*/
    CORDICx->CSR.V32 = *((uint32_t*)InitCfg);
}

/**
  * @brief  Fills each InitStruct member with its default value.
  * @param  InitStruct : pointer to a CORDIC_InitTypeDef structure which will
  *   be initialized.
  * @retval None
  */
void CORDIC_StructInit(CORDIC_InitTypeDef* InitStruct)
{
    const CORDIC_InitTypeDef CORDIC_DEFAULT_CFG = {0};
    
    assert_param(IS_IN_CRAM(InitStruct) || IS_IN_SRAM(InitStruct));
    
    *InitStruct = CORDIC_DEFAULT_CFG;    
}

/**@brief  Deinitializes the CORDICx registers to their default reset
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@retval None
  */
void CORDIC_Reset(CORDIC_TypeDef* CORDICx)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    
    /* Initialize CORDIC Module*/
    CORDICx->CSR.V32 = 0;
    CORDICx->OPRDX = 0;
    CORDICx->OPRDY = 0;
    CORDICx->OPRDZ = 0;
}


/**
  *@brief   CORDIC transform
  *@details
  *\n        === Rotate Mode ===
  *\n        X' = K[Xcos(Z) - Ysin(Z)]  
  *\n        Y' = K[Xsin(Z) + Ycos(Z)]
  *\n        Z' = 0
  *\n        K = 1.6467602579
  *\n        === Vector Mode ===
  *\n        X' = K * sqrt(X^2 + Y^2)
  *\n        Y' = 0
  *\n        Z' = Z + arctan(Y/X)
  *\n        K = 1.6467602579  
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param CordicData  input data to transform then get result from it
  *@retval  bool_t transform result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE operation overflow
  */
bool_t CORDIC_Transform(CORDIC_TypeDef* CORDICx, CORDIC_DataTypeDef* CordicData)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_IN_CRAM(CordicData) || IS_IN_SRAM(CordicData));
    
    /* set CORDIC parameters */
    CORDICx->OPRDX = CordicData->X;
    CORDICx->OPRDY = CordicData->Y;
    CORDICx->OPRDZ = CordicData->Z;
    
    /* start transform */
    CORDIC_START(CORDICx);    
    
    /* wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));
    
    /* get result */
    CordicData->X = CORDICx->OPRDX;
    CordicData->Y = CORDICx->OPRDY;
    CordicData->Z = CORDICx->OPRDZ;    
    
    /* return transform error flag */
    return (CORDICx->CSR.BIT.OVF ? FALSE : TRUE);
}


/**
  * @code   Example
       CORDIC_DataTypeDef CordicData;
       CORDIC_InitTypeDef InitCfg;
       InitCfg.KADJ   = CORDIC_KADJ_DIVK;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       InitCfg.MODE   = CORDIC_MODE_ROTATE;
       CORDIC_Init(CORDIC0, &InitCfg);
       CordicData.X = IQ24(0.4);
       CordicData.Y = IQ24(-0.4);
       CordicData.Z = IQ24(0.3);       
       CORDIC_Run(CORDIC0,&CordicData);
       
       while(CORDIC_IS_BUSY(CORDIC0));  
        
       if(CORDIC_GetResult(CORDIC0,&CordicData))
       {
            printf("X=%f  Y=%f  Z=%f\n",IQnToFloat(CordicData.X,24),IQnToFloat(CordicData.Y,24),IQnToFloat(CordicData.Z,24));
       }

  * @endcode 
  *@brief  start CORDIC transform then return immediately
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param CordicData  input data to transform then get result from it
  *@retval   None
  */ 
void CORDIC_Run(CORDIC_TypeDef* CORDICx,CORDIC_DataTypeDef* CordicData)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    
    /* set CORDIC parameters */
    CORDICx->OPRDX = CordicData->X;
    CORDICx->OPRDY = CordicData->Y;
    CORDICx->OPRDZ = CordicData->Z;
    
    /* start transform */
    CORDIC_START(CORDICx);    
}

/**
  *@brief  get CORDIC result
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param CordicData  output the transform result
  *@retval  bool_t transform result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE operation overflow
  */
bool_t CORDIC_GetResult(CORDIC_TypeDef* CORDICx, CORDIC_DataTypeDef* CordicData)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_IN_CRAM(CordicData) || IS_IN_SRAM(CordicData));
        
    /* get transform result */
    CordicData->X = CORDICx->OPRDX;
    CordicData->Y = CORDICx->OPRDY;
    CordicData->Z = CORDICx->OPRDZ;    

    /* return transform error flag */
    return (CORDICx->CSR.BIT.OVF ? FALSE : TRUE);
}
   

/**
  * @code   Example
       IQRotate_TypeDef RotateData;
       CORDIC_InitTypeDef InitCfg;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       CORDIC_Init(CORDIC0, &InitCfg);
       RotateData.X = IQ24(0.3);
       RotateData.Y = IQ24(0.2);
       RotateData.Angle = IQ24(0.5);       
       if(CORDIC_IQRotate(CORDIC0, &RotateData, CORDIC_FMT_IQ24))
       {
           printf("new coordinate (%f, %f)\n",IQnToFloat(RotateData.X,24),IQnToFloat(RotateData.Y,24));
       }
  * @endcode 
  *@brief   coordinate(X,Y) rotate transform. 
  *@details
  *\n        input coordinate(X,Y) and rotate Angle.
  *\n        output is new coordinate(x1,y1)
  *\n        X' = K[Xcos(Z) - Ysin(Z)]  
  *\n        Y' = K[Xsin(Z) + Ycos(Z)]
  *\n        Z' = 0
  *\n        K = 1.6467602579
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param RotateData  input data to transform then get new coordinate
  *@retval  bool_t result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE operation overflow
  */
bool_t CORDIC_IQRotate(CORDIC_TypeDef* CORDICx, IQRotate_TypeDef* RotateData)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_IN_CRAM(RotateData) || IS_IN_SRAM(RotateData));

    /* select KADJ mode */
    CORDIC_SetKADJ(CORDICx,CORDIC_KADJ_DIVK);
    
    /* select rotate mode */
    CORDIC_SetMode(CORDICx,CORDIC_MODE_ROTATE);
    
    
    /* set orignal coordinate (X,Y)*/
    CORDICx->OPRDX = RotateData->X;
    CORDICx->OPRDY = RotateData->Y;
    
    /* set rotate Angle */
    CORDICx->OPRDZ = RotateData->Angle;
    
    /* start transform */
    CORDIC_START(CORDICx);    
    
    /* wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));
    
    /* get new coordinate */
    RotateData->X = CORDICx->OPRDX;
    RotateData->Y = CORDICx->OPRDY;
    
    /* return transform error flag */
    return (CORDICx->CSR.BIT.OVF ? FALSE : TRUE);
}


/**
  * @code   Example
       IQVector_TypeDef VectorData;
       CORDIC_InitTypeDef InitCfg;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       CORDIC_Init(CORDIC0, &InitCfg);
       VectorData.X = IQ24(0.3);
       VectorData.Y = IQ24(0.2);
       if(CORDIC_IQVector(CORDIC0, &VectorData, CORDIC_FMT_IQ24))
       {
            printf("vecotr RMS:%f  direction:%f\n",IQnToFloat(VectorData.RMS,24),IQnToFloat(VectorData.ATAN,24));
       }
 * @endcode 
  *@brief coordinate(X,Y) vector transform \n
  *@details
  *      input  VectorData.X, VectorData.Y \n
  *      output VectorData.RMS  = sqrt(X^2 + Y^2) \n
  *      output VectorData.ATAN = ATAN(Y/X) \n
  *        X' = K * sqrt(X^2 + Y^2)\n
  *        Y' = 0 \n
  *        Z' = Z + arctan(Y/X) \n
  *        K = 1.6467602579  \n
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param VectorData  input coordinate data to transform then get the vector
  *@retval  bool_t result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE operation overflow
  */
bool_t CORDIC_IQVector(CORDIC_TypeDef* CORDICx, IQVector_TypeDef* VectorData)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_IN_CRAM(VectorData) || IS_IN_SRAM(VectorData));
    
    /* select KADJ mode. */
    CORDIC_SetKADJ(CORDICx,CORDIC_KADJ_DIVK);
    
    /* select vector transform mode */
    CORDIC_SetMode(CORDICx,CORDIC_MODE_VECTOR);
      
    /* set (X,Y),Z=0 to transform*/
    CORDICx->OPRDX = VectorData->X;
    CORDICx->OPRDY = VectorData->Y;
    CORDICx->OPRDZ = 0;
    
    /* start transform */
    CORDIC_START(CORDICx);    
    
    /* wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));
    
    /* get result */        
    VectorData->MOD  = CORDICx->OPRDX;
    VectorData->ATAN  = CORDICx->OPRDZ;          
    
    /* return transform error flag */
    return (CORDICx->CSR.BIT.OVF ? FALSE : TRUE);
}

/**
  * @code   Example
       IQSinCos_TypeDef SinCosData;
       CORDIC_InitTypeDef InitCfg;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       CORDIC_Init(CORDIC0, &InitCfg);
       SinCosData.Theta = IQ24(0.3/PI);
       if(CORDIC_IQSinCos(CORDIC0, &SinCosData))
       {
            printf("SIN:%f  COS:%f\n",IQnToFloat(SinCosData.SIN,24),IQnToFloat(SinCosData.COS,24));
       }
  * @endcode 
  *@brief caculate IQ format SIN and COS
  *@details
  *    @arg @b input SinCosData.Theta
  *    @arg @b output SinCosData.COS = COS(SinCosData.Theta) 
  *    @arg @b output SinCosData.SIN = SIN(SinCosData.Theta)
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param SinCosData  input Theta data to transform then get sine and cosine value
  *@retval bool_t result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE operation overflow
  */
bool_t CORDIC_IQSinCos(CORDIC_TypeDef* CORDICx,IQSinCos_TypeDef* SinCosData )
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_IN_CRAM(SinCosData) || IS_IN_SRAM(SinCosData));  
    
     /* select KADJ mode. Since X is 1/K, so result is KADK_NONE */
    CORDIC_SetKADJ(CORDICx,CORDIC_KADJ_NONE);
   
    /* select roate mode */
    CORDIC_SetMode(CORDICx,CORDIC_MODE_ROTATE);
    
    /* set parameter X=1.0 */
    switch(CORDICx->CSR.BIT.FORMAT)
    {
        case CORDIC_FMT_IQ26:
             CORDICx->OPRDX = IQ26_1_K;
            break;
        case CORDIC_FMT_IQ24:
             /* in cordic IQ24 treated as IQ26, so use IQ26_1_K */
             CORDICx->OPRDX = IQ26_1_K;
            break;
        case CORDIC_FMT_IQ15:
             CORDICx->OPRDX = IQ15_1_K;
            break;
        default:
            break;
    }
    
    /* set parameter Y=0 */    
    CORDICx->OPRDY = 0x0;
    /* set parameter Z=Theta */    
    CORDICx->OPRDZ = SinCosData->Theta;
    
    /* start transform */
    CORDIC_START(CORDICx);    
    
    /* wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));
    
    /* get result */        
    if(CORDICx->CSR.BIT.FORMAT == CORDIC_FMT_IQ24)
    {
        /* since IQ24 treated as IQ26, need adjust the result must */
        SinCosData->COS = CORDICx->OPRDX>>2;
        SinCosData->SIN = CORDICx->OPRDY>>2;          			
    }
    else
    {
        SinCosData->COS = CORDICx->OPRDX;
        SinCosData->SIN = CORDICx->OPRDY;          
    }
    /* return transform error flag */
    return (CORDICx->CSR.BIT.OVF ? FALSE : TRUE);    
}

/**
  * @code   Example
       PARK_TypeDef ParkData;
       CORDIC_InitTypeDef InitCfg;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       CORDIC_Init(CORDIC0, &InitCfg);
       ParkData.Alpha = IQ24(0.3);
       ParkData.Beta = IQ24(0.3);
       ParkData.Theta = IQ24(0.5);
       CORDIC_IQPark(CORDIC0, &ParkData);
       printf("Iq:%f  Id:%f\n",IQnToFloat(ParkData.Id,24),IQnToFloat(ParkData.Iq,24));
  * @endcode 
  *@brief Park conversation
  *@details
  *    @arg @b input ParkData.Alpha, ParkData.Beta and ParkData.Theta
  *    @arg @b output ParkData.Iq = ParkData.Beta * COS(ParkData.Theta) - ParkData.Alpha * SIN(ParkData.Theta)
  *    @arg @b output ParkData.Id = ParkData.Alpha* COS(ParkData.Theta) + ParkData.Beta  * SIN(ParkData.Theta)
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param ParkData  input Theta data to transform then get SIN and COS value
  *@retval bool_t result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE operation overflow
  */
bool_t CORDIC_IQPark(CORDIC_TypeDef* CORDICx, PARK_TypeDef* ParkData)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_IN_CRAM(ParkData) || IS_IN_SRAM(ParkData));  
 
    /* select KADJ mode. */
    CORDIC_SetKADJ(CORDICx,CORDIC_KADJ_DIVK);
    
    /* select rotate mode */
    CORDIC_SetMode(CORDICx,CORDIC_MODE_ROTATE);
        
    /* set input data */
    CORDICx->OPRDX = ParkData->Beta;
    CORDICx->OPRDY = ParkData->Alpha;    
	  CORDICx->OPRDZ = ParkData->Theta;			            

    /* start transform */
    CORDIC_START(CORDICx);

    /*wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));  
        
    /* get result */        
    ParkData->Iq = CORDICx->OPRDX;
    ParkData->Id = CORDICx->OPRDY;
    
    /* return transform error flag */
    return (CORDICx->CSR.BIT.OVF ? FALSE : TRUE);        
}


/**
  * @code   Example
       IPARK_TypeDef IParkData;
        CORDIC_InitTypeDef InitCfg;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       CORDIC_Init(CORDIC0, &InitCfg);
       IParkData.Ud = IQ24(0.3);
       IParkData.Uq = IQ24(0.3);
       IParkData.Theta = IQ24(0.5);
       CORDIC_IQIPark(CORDIC0, &IParkData);
       printf("Alpha:%f  Beta:%f\n",IQnToFloat(IParkData.Alpha,24),IQnToFloat(IParkData.Beta,24));
  * @endcode 
  *@brief IPark conversation
  *@details
  *   @arg @b input  IParkData.Ud, IParkData.Uq and IParkData.Theta
  *   @arg @b output IParkData.Alpha = IParkData.Ud * COS(IParkData.Theta)- IParkData.Uq * SIN(IParkData.Theta)
  *   @arg @b output IParkData.Beta  = IParkData.Ud * SIN(ParkData.Theta) + IParkData.Uq * COS(IParkData.Theta)
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param IParkData  input Theta data to transform then get SIN and COS value
  *@retval bool_t result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE operation overflow
  */
bool_t CORDIC_IQIPark(CORDIC_TypeDef* CORDICx, IPARK_TypeDef* IParkData)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
    assert_param(IS_IN_CRAM(IParkData) || IS_IN_SRAM(IParkData));  
 
    /* select KADJ mode. */
    CORDIC_SetKADJ(CORDICx,CORDIC_KADJ_DIVK);
    
    /* select rotate mode */
    CORDIC_SetMode(CORDICx,CORDIC_MODE_ROTATE);
    
    
    /* set input data */
    CORDICx->OPRDX = IParkData->Ud;
    CORDICx->OPRDY = IParkData->Uq;
	CORDICx->OPRDZ = IParkData->Theta;			            

    /* start transform */
    CORDIC_START(CORDICx);
    
    /*wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));  

    /* get result */        
    IParkData->Alpha = CORDICx->OPRDX;
    IParkData->Beta  = CORDICx->OPRDY;
    
    /* return transform error flag */
    return (CORDICx->CSR.BIT.OVF ? FALSE : TRUE);        
}


/**
  * @code   Example
        IQ24 atanv;
       CORDIC_InitTypeDef InitCfg;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       CORDIC_Init(CORDIC0, &InitCfg);
       atanv = CORDIC_IQAtan(CORDIC0, IQ24(0.3));
       printf("arctan:%f \n",IQnToFloat(atanv,24));
  * @endcode 
  *@brief ARCTAN transform
  *@details
  *\n     input  X 
  *\n     output arctan(X);
  *@param CORDICx  CORDIC device selection
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param X input X to transform then get arctan value
  *@retval IQN arctan(X)
  */
IQN CORDIC_IQAtan(CORDIC_TypeDef* CORDICx, IQN X)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));
 
    /* select KADJ mode. */
    CORDIC_SetKADJ(CORDICx,CORDIC_KADJ_DIVK);
    
    /* select vector mode */
    CORDIC_SetMode(CORDICx,CORDIC_MODE_VECTOR);

    /* set parameter X=1.0 */
    switch(CORDICx->CSR.BIT.FORMAT)
    {
        case CORDIC_FMT_IQ26:
             CORDICx->OPRDX = IQ26_1;
            break;
        case CORDIC_FMT_IQ24:
             CORDICx->OPRDX = IQ24_1;
            break;
        case CORDIC_FMT_IQ15:
             CORDICx->OPRDX = IQ15_1;
            break;
        default:
            break;
    }
    
    /* set input data */
    CORDICx->OPRDY = X;
	CORDICx->OPRDZ = 0;			            

    /* start transform */
    CORDIC_START(CORDICx);
    
    /*wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));  

    /* get result */        
    return ((IQN)CORDICx->OPRDZ);    
}


/**
  * @code   Example
        ** Result is normalization to -1~1. you can *PI if needed.
        IQ24 atanv;
       CORDIC_InitTypeDef InitCfg;
       InitCfg.XYMRS  = CORDIC_XYMRS_NONE;
       InitCfg.FORMAT = CORDIC_FMT_IQ24;
       CORDIC_Init(CORDIC0, &InitCfg);
       atanv = CORDIC_IQAtan2(CORDIC0, IQ24(0.3),IQ24(0.4));
       printf("arctan(Y/X):%f \n",IQnToFloat(atanv,24)*PI);
  * @endcode 
  *@brief ARCTAN transform
  *\n         input  X,Y 
  *\n         output arctan(Y/X);
  *@param CORDICx  CORDIC device selection
  *@details
  *     @arg @b CORDIC0 
  *     @arg @b CORDIC1 
  *@param X  input X to transform then get arctan value
  *@param Y  input Y to transform then get arctan value
  *@retval IQN arctan(Y/X)
  */
IQN CORDIC_IQAtan2(CORDIC_TypeDef* CORDICx, IQN X,IQN Y)
{
    assert_param(IS_CORDIC_MODULE(CORDICx));

    /* select KADJ mode. */
    CORDIC_SetKADJ(CORDICx,CORDIC_KADJ_DIVK);
    
    /* select vector mode */
    CORDIC_SetMode(CORDICx,CORDIC_MODE_VECTOR);
    
    /* set input data */
    CORDICx->OPRDX = X;
    CORDICx->OPRDY = Y;
	CORDICx->OPRDZ = 0;			            

    /* start transform */
    CORDIC_START(CORDICx);
    
    /*wait transform done */
    while(CORDIC_IS_BUSY(CORDICx));  

    /* get result */        
    return ((IQN)CORDICx->OPRDZ);    
}


/**
  * @}
  */

/** @defgroup  MACP_Group_IQDiv_Funcs  IQDIV Functions
 *  @brief   IQDIV Public Functions
 *
  * @{
  */

/**
  * @code   Example
       const IQDIV_InitTypeDef IQDIV_CFG = {{0,IQDIV_SIGNED,0,24,IQDIV_CMOD_ROUND}};
       uint32_t rlt;
       IQDIV_Init(IQDIV0,&IQDIV_CFG);
       rlt = IQDIV_IQDiv(IQDIV0,IQ24(0.4),IQ24(0.2));
       printf("result = %f\n",IQnToFloat(rlt,24));
  *@endcode 
  *@brief   IQDIV Module Init
  *@param IQDIVx  IQDiv Module selection
  *     @arg @b IQDIV0
  *     @arg @b IQDIV1
  *@param InitCfg  Input IQDIV module init value
  *     @arg @b --- BIT.SIGN IQDiv data format
  *     @arg @b IQDIV_UNSIGNED
  *     @arg @b IQDIV_SIGNED
  *     @arg @b --- BIT.Q IQ format(0~31)
  *     @arg @b --- BIT.CMOD result approximate method
  *     @arg @b IQDIV_CMOD_TRUNC
  *     @arg @b IQDIV_CMOD_ROUND
  *@retval  None
  */
void IQDIV_Init(IQDIV_TypeDef* IQDIVx, const IQDIV_InitTypeDef* InitCfg)
{
    assert_param(IS_IQDIV_MODULE(IQDIVx));
    assert_param(IS_IQDIV_SIGN_TYPE(InitCfg->SignMode));
    assert_param((InitCfg->QFormat <= 31));
    assert_param(IS_IQDIV_CMOD_TYPE(InitCfg->ApproximateMethod));
    
    IQDIVx->CSR.V32 = *((uint32_t*)InitCfg);
}


/**
  * @brief  Fills each InitStruct member with its default value.
  * @param  InitStruct : pointer to a IQDIV_InitTypeDef structure which will
  *   be initialized.
  * @retval None
  */
void IQDIV_StructInit(IQDIV_InitTypeDef* InitStruct)
{
    const IQDIV_InitTypeDef IQDIV_DEFAULT_CFG = {0};
    
    assert_param(IS_IN_CRAM(InitStruct) || IS_IN_SRAM(InitStruct));
    
    *InitStruct = IQDIV_DEFAULT_CFG;    
}

/**@brief Deinitializes the IQDIVx registers to their default reset 
  *@param IQDIVx  IQDiv Module selection
  *     @arg @b IQDIV0
  *     @arg @b IQDIV1
  *@retval  None
  */
void IQDIV_Reset(IQDIV_TypeDef* IQDIVx)
{
    assert_param(IS_IQDIV_MODULE(IQDIVx));
    
    IQDIVx->CSR.V32 = 0;
    IQDIVx->DIVDND  = 0;
    IQDIVx->DIVSOR  = 0;
    IQDIVx->RLT  = 0;
}


/**
  *@brief IQDIV and return the result
  *@param IQDIVx  IQDiv Module selection
  *     @arg @b IQDIV0
  *     @arg @b IQDIV1
  *@param Dividend  Q Format Dividend data
  *@param Divisor   Q Format Divisor data
  *@retval IQN IQDIV result
  */
IQN IQDIV_IQDiv(IQDIV_TypeDef* IQDIVx, IQN Dividend, IQN Divisor)
{
    assert_param(IS_IQDIV_MODULE(IQDIVx));

    /* set division parameters */
    IQDIVx->DIVDND = Dividend;                
    IQDIVx->DIVSOR = Divisor;
    /* clear saturate flag */
    IQDIV_CLEAR_SAT(IQDIVx);
    
    /*start calculate */
    IQDIV_START(IQDIVx);

    /* wait calcuate done */
    while(IQDIV_IS_BUSY(IQDIVx));
    
    /* return the result */
    return ((IQN)IQDIVx->RLT);
}

/**
  * @code   Example
       uint32_t rlt;
       IQDIV_Init(IQDIV0,IQDIV_SIGNED,23,IQDIV_CMOD_ROUND);
       IQDIV_Div_Run(IQDIV0,0x240000,0x200000);
       while(IQDIV_IS_BUSY(IQDIV0));
       rlt = GET_IQDIV_RESULT(IQDIV0);    
  *@endcode 
  *@brief start IQDIV then return immediately
  *@param IQDIVx  IQDiv Module selection
  *     @arg @b IQDIV0
  *     @arg @b IQDIV1
  *@param Dividend  Q Format Dividend data
  *@param Divisor  Q Format Divisor data
  *@retval None
  */
void IQDIV_Div_Run(IQDIV_TypeDef* IQDIVx, IQN Dividend, IQN Divisor)
{
    assert_param(IS_IQDIV_MODULE(IQDIVx));

    /* set division parameters */
    IQDIVx->DIVDND = Dividend;                
    IQDIVx->DIVSOR = Divisor;
    
    /* clear saturate flag */
    IQDIV_CLEAR_SAT(IQDIVx);
    
    /*start calculate */
    IQDIV_START(IQDIVx);
}

/**
  * @code   Example
       IQDiv_DataTypeDef DivData;   
       IQDIV_Init(IQDIV0,IQDIV_SIGNED,24,IQDIV_CMOD_ROUND);
       DivData.Dividend = IQ24(0.6);
       DivData.Divisor  = IQ24(0.2);
       IQDIV_IQDiv2(IQDIV0,&DivData);
       printf("result=%f\n",IQnToFloat(DivData.Quotient,24));
  * @endcode 
  *@brief   IQDIV Module Init
  *@param IQDIVx  IQDiv Module selection
  *     @arg @b IQDIV0
  *     @arg @b IQDIV1
  *@param DivData  input data to division then get result
  *@retval bool_t result is legal or not
  *    @arg @b TRUE  result is right
  *    @arg @b FALSE result is saturate
  */
bool_t IQDIV_IQDiv2(IQDIV_TypeDef* IQDIVx, IQDiv_DataTypeDef* DivData)
{
    assert_param(IS_IQDIV_MODULE(IQDIVx));
    assert_param(IS_IN_CRAM(DivData) || IS_IN_SRAM(DivData));
    
    /* set division parameters */
    IQDIVx->DIVDND = DivData->Dividend;                
    IQDIVx->DIVSOR = DivData->Divisor;
    
    /* clear saturate flag */
    IQDIV_CLEAR_SAT(IQDIVx);
    
    /* start calculate */
    IQDIV_START(IQDIVx);
    
    /* wait calculate done */
    while(IQDIV_IS_BUSY(IQDIVx));
    
    /* get result */
    DivData->Quotient = IQDIV_GET_RESULT(IQDIVx);
    
    /* return result flag */
    return (IQDIV_IS_SATURATE(IQDIVx) ? FALSE : TRUE);
}

/**
  * @}
  */

/** @defgroup  MACP_Group_SVPWM_Funcs  SVPWM Functions
 *  @brief   SVPWM Public Functions
 *
  * @{
  */

/**
  * @code   Example
       SVPWM_DataTypeDef SVPWMData;
       SVPWMData.UAlpha = IQ24(0.3);
       SVPWMData.UBeta  = IQ24(0.2);
       SVPWMData.SVType = SVPWM_SVTYPE_7;
       SVPWMData.SVIQN  = 24;
       SVPWM_Transform(&SVPWMData);
       printf("Ta:%X Tb:%X Tc:%X Sector:%X\n",SVPWMData.Ta,SVPWMData.Tb,SVPWMData.Tc,SVPWMData.Sector);
  *@endcode 
  *@brief Initial SVPWM Module
  *@param SVPWMData  Input SVPWM Alpha and Beta value then get result from Ta,Tb,Tc,Sector.
  *@retval None
  */
void SVPWM_Transform(SVPWM_DataTypeDef* SVPWMData)
{
    assert_param(IS_SVPWM_SVTYPE_TYPE(SVPWMData->SVType));
    assert_param(SVPWMData->SVIQN < 32);
    
    /* set parameters */
    MACP->SVUALPHA = SVPWMData->UAlpha;
    MACP->SVUBETA  = SVPWMData->UBeta;
    MACP->SVIQN.BIT.SVTYP = SVPWMData->SVType;
    MACP->SVIQN.BIT.SVIQN = SVPWMData->SVIQN;

    /* start transform */
    SVPWM_START();
    
    /* wait transform done */
    while(SVPWM_IS_BUSY());
    
    /* get ransform result */
    SVPWMData->Ta = MACP->SVTA;
    SVPWMData->Tb = MACP->SVTB;
    SVPWMData->Tc = MACP->SVTC;
    SVPWMData->Sector = MACP->SVSECTOR;    
}


/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT Sinowealth *****END OF FILE****/


