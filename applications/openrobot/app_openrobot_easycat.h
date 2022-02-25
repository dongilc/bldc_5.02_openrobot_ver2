//********************************************************************************************    
//                                                                                           *
// This software is distributed as an example, "AS IS", in the hope that it could            *
// be useful, WITHOUT ANY WARRANTY of any kind, express or implied, included, but            *
// not limited,  to the warranties of merchantability, fitness for a particular              *
// purpose, and non infringiment. In no event shall the authors be liable for any            *    
// claim, damages or other liability, arising from, or in connection with this software.     *
//                                                                                           *
//******************************************************************************************** 

#ifndef EASYCAT_H
#define EASYCAT_H

//                                                         SPI1        SPI2/3
#define SPI_BaudRatePrescaler_2         ((uint16_t)0x0000) //  42 MHz      21 MHZ
#define SPI_BaudRatePrescaler_4         ((uint16_t)0x0008) //  21 MHz      10.5 MHz
#define SPI_BaudRatePrescaler_8         ((uint16_t)0x0010) //  10.5 MHz    5.25 MHz
#define SPI_BaudRatePrescaler_16        ((uint16_t)0x0018) //  5.25 MHz    2.626 MHz
#define SPI_BaudRatePrescaler_32        ((uint16_t)0x0020) //  2.626 MHz   1.3125 MHz
#define SPI_BaudRatePrescaler_64        ((uint16_t)0x0028) //  1.3125 MHz  656.25 KHz

#ifdef HW_SPI_DEV
//LAN9252 max clk freq: 80MHz 
// in case of PIC clock FOSC/16  = 64MHz/16 = 4MHz 
// in case of arduino #define SpiSpeed        8000000 
// common setting : CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
static const SPIConfig easycat_spi_cfg = {
		SPI_MASTER,	//openrobot-spi role selection required
		NULL,
		HW_SPI_PORT_NSS,
		HW_SPI_PIN_NSS,
		SPI_BaudRatePrescaler_4}; // SPI_BaudRatePrescaler_2 is not working
#endif

//--- SPI macro ---------------------------------------------------------------------------------
#define SCS_Low_macro	      palClearPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS)
#define SCS_High_macro	    palSetPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS)    
          
#define SPI_TransferTx      SPI1Transfer         
#define SPI_TransferTxLast  SPI1Transfer 
#define SPI_TransferRx      SPI1Transfer    

//--- global functions --------------------------------------------------------------------------
unsigned char EasyCAT_MainTask(void);   // EtherCAT main task   
unsigned char EasyCAT_Init(void);       // EasyCAT board initialization 
//----------------------------------------------------------------------------------------------- 

//*********************************************************************************************
#define BYTE_NUM  128               // Standard mode. support 32, 64, 128

//#define CUSTOM                      // Custom mode
//#include "TestEasyCAT_Custom.h"     // This file has been created by the Easy Configurator 
                                    // and must be located in the Arduino project folder
                                    //
                                    // There are two others files created by the Easy Configurator:
                                    // TestEasyCAT_Custom.bin that must be loaded into the EEPROM.
                                    // TestEasyCAT_Custom.xml that must be used by the EtherCAT master.                                 
//*********************************************************************************************

#if (!defined BYTE_NUM && !defined CUSTOM)			// if BYTE_NUM is not declared 
  #define BYTE_NUM 32                    			// set it to the 32+32 byte default 
#endif                                    			// 
 
//-- the preprocessor calculates the parameters necessary to transfer out data --- 

                                                    // define TOT_BYTE_NUM_OUT as the
                                                    // total number of byte we have to
                                                    // transfer in output
                                                    // this must match the ESI XML
                                                    //
#ifdef  BYTE_NUM                                    // in Standard Mode
  #define TOT_BYTE_NUM_OUT  BYTE_NUM                // 16, 32, 64 or 128
                                                    //
#else                                               // in Custom Mode  
  #define TOT_BYTE_NUM_OUT  CUST_BYTE_NUM_OUT       // any number between 0 and 128  
#endif                                              //


#if TOT_BYTE_NUM_OUT > 64                           // if we have more then  64 bytes
                                                    // we have to split the transfer in two
                                                        
  #define SEC_BYTE_NUM_OUT  (TOT_BYTE_NUM_OUT - 64) // number of bytes of the second transfer
                                        
  #if ((SEC_BYTE_NUM_OUT & 0x03) != 0x00)           // number of bytes of the second transfer
                                                    // rounded to long
    #define SEC_BYTE_NUM_ROUND_OUT  ((SEC_BYTE_NUM_OUT | 0x03) + 1)  
  #else                                             // 
    #define SEC_BYTE_NUM_ROUND_OUT  SEC_BYTE_NUM_OUT//
  #endif                                            //

  #define FST_BYTE_NUM_OUT  64                      // number of bytes of the first transfer     
  #define FST_BYTE_NUM_ROUND_OUT  64                // number of bytes of the first transfer
                                                    // rounded to 4 (long)

#else                                               // if we have max 64 bytes we transfer
                                                    // them in just one round
                                                        
  #define FST_BYTE_NUM_OUT  TOT_BYTE_NUM_OUT        // number of bytes of the first and only transfer 
  
  #if ((FST_BYTE_NUM_OUT & 0x03) != 0x00)           // number of bytes of the first and only transfer  
                                                    // rounded to 4 (long)   
    #define FST_BYTE_NUM_ROUND_OUT ((FST_BYTE_NUM_OUT | 0x03) + 1)
  #else                                             //
    #define FST_BYTE_NUM_ROUND_OUT  FST_BYTE_NUM_OUT//   
  #endif                                            //

  #define SEC_BYTE_NUM_OUT  0                       // we don't use the second round
  #define SEC_BYTE_NUM_ROUND_OUT  0                 //
  
#endif


//-- the preprocessor calculates the parameters necessary to transfer in data --- 
 
                                                    // define TOT_BYTE_NUM_IN as the
                                                    // total number of byte we have to
                                                    // transfer in input
                                                    // this must match the ESI XML
                                                    //     
#ifdef  BYTE_NUM                                    // in Standard Mode
  #define TOT_BYTE_NUM_IN  BYTE_NUM                 // 16, 32, 64 or 128
                                                    //
#else                                               // in Custom Mode
  #define TOT_BYTE_NUM_IN  CUST_BYTE_NUM_IN         // any number between 0 and 128  
#endif                                              //


#if TOT_BYTE_NUM_IN > 64                            // if we have more then  64 bytes
                                                    // we have to split the transfer in two
                                                        
  #define SEC_BYTE_NUM_IN  (TOT_BYTE_NUM_IN - 64)   // number of bytes of the second transfer
 
  #if ((SEC_BYTE_NUM_IN & 0x03) != 0x00)            // number of bytes of the second transfer 
                                                    // rounded to 4 (long)          
    #define SEC_BYTE_NUM_ROUND_IN  ((SEC_BYTE_NUM_IN | 0x03) + 1)  
  #else                                             //
    #define SEC_BYTE_NUM_ROUND_IN  SEC_BYTE_NUM_IN  //
  #endif                                            //

  #define FST_BYTE_NUM_IN  64                       // number of bytes of the first transfer     
  #define FST_BYTE_NUM_ROUND_IN  64                 // number of bytes of the first transfer
                                                    // rounded to 4 (long)

#else                                               // if we have max 64 bytes we transfer
                                                    // them in just one round
                                                        
  #define FST_BYTE_NUM_IN  TOT_BYTE_NUM_IN          // number of bytes of the first and only transfer  

  #if ((FST_BYTE_NUM_IN & 0x03) != 0x00)            // number of bytes of the first and only transfer
                                                    // rounded to 4 (long)
    #define FST_BYTE_NUM_ROUND_IN ((FST_BYTE_NUM_IN | 0x03) + 1)
  #else                                             //
    #define FST_BYTE_NUM_ROUND_IN  FST_BYTE_NUM_IN  // 
  #endif                                            //

  #define SEC_BYTE_NUM_IN  0                        // we don't use the second round
  #define SEC_BYTE_NUM_ROUND_IN  0                  //

#endif
 

//----------------- sanity check -------------------------------------------------------                                 
      

#ifdef BYTE_NUM                     // STANDARD MODE and CUSTOM MODE
                                    // cannot be defined at the same time
  #ifdef CUST_BYTE_NUM_OUT 
    #error "BYTE_NUM and CUST_BYTE_NUM_OUT cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
    #endif
  
  #ifdef CUST_BYTE_NUM_IN 
    #error "BYTE_NUM and CUST_BYTE_NUM_IN cannot be defined at the same time !!!!"
    #error "define them correctly in file EasyCAT.h"
  #endif
#endif 
      
#ifdef BYTE_NUM                     //--- for BYTE_NUM we accept only 16  32  64  128 --
                                  
  #if ((BYTE_NUM !=16) && (BYTE_NUM !=32) && (BYTE_NUM !=64)  && (BYTE_NUM !=128))
    #error "BYTE_NUM must be 16, 32, 64 or 128 !!! define it correctly in file EasyCAT.h"
  #endif 
  
#else
                                   //--- CUST_BYTE_NUM_OUT and CUST_BYTE_NUM_IN --------
                                   //    must be max 128
  #if (CUST_BYTE_NUM_OUT > 128)
    #error "CUST_BYTE_NUM_OUT must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif 
  
  #if (CUST_BYTE_NUM_IN > 128)
    #error "CUST_BYTE_NUM_IN must be max 128 !!! define it correctly in file EasyCAT.h"
  #endif 
  
#endif 
           

//---- LAN9252 registers --------------------------------------------------------------------------

                                            //---- access to EtherCAT registers -------------------

#define ECAT_CSR_DATA           0x0300      // EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD            0x0304      // EtherCAT CSR Interface Command Register


                                            //---- access to EtherCAT process RAM ----------------- 

#define ECAT_PRAM_RD_ADDR_LEN   0x0308      // EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD        0x030C      // EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN   0x0310      // EtherCAT Process RAM Write Address and Length Register 
#define ECAT_PRAM_WR_CMD        0x0314      // EtherCAT Process RAM Write Command Register

#define ECAT_PRAM_RD_DATA       0x0000      // EtherCAT Process RAM Read Data FIFO
#define ECAT_PRAM_WR_DATA       0x0020      // EtherCAT Process RAM Write Data FIFO

                                            //---- EtherCAT registers -----------------------------

#define AL_CONTROL              0x0120      // AL control                                             
#define AL_STATUS               0x0130      // AL status
#define AL_STATUS_CODE          0x0134      // AL status code
#define AL_EVENT                0x0220      // AL event request
#define AL_EVENT_MASK           0x0204      // AL event interrupt mask

#define WDOG_STATUS             0x0440      // watch dog status

#define SM0_BASE                0x0800      // SM0 base address (output)
#define SM1_BASE                0x0808      // SM1 base address (input) 


                                            //---- LAN9252 registers ------------------------------    

#define HW_CFG                  0x0074      // hardware configuration register
#define BYTE_TEST               0x0064      // byte order test register
#define RESET_CTL               0x01F8      // reset register       
#define ID_REV                  0x0050      // chip ID and revision
#define IRQ_CFG                 0x0054      // interrupt configuration
#define INT_EN                  0x005C      // interrupt enable


//---- LAN9252 flags ------------------------------------------------------------------------------

#define ECAT_CSR_BUSY     		0x80
#define PRAM_ABORT        		0x40000000
#define PRAM_BUSY         		0x80
#define PRAM_AVAIL        		0x01
#define READY             		0x08
#define DIGITAL_RST       		0x00000001


//---- EtherCAT flags -----------------------------------------------------------------------------

#define ALEVENT_CONTROL         0x0001
#define ALEVENT_SM              0x0010
 
 
//----- state machine ------------------------------------------------------------

#define ESM_INIT                0x01          // state machine control
#define ESM_PREOP               0x02          // (state request)
#define ESM_BOOT                0x03          // 
#define ESM_SAFEOP              0x04          // safe-operational
#define ESM_OP                  0x08          // operational
    
    
//--- ESC commands --------------------------------------------------------------------------------

#define ESC_WRITE 		   		0x80
#define ESC_READ 		   		  0xC0


//---- SPI ----------------------------------------------------------------------------------------

#define COMM_SPI_READ    		0x03
#define COMM_SPI_WRITE   		0x02

#define DUMMY_BYTE       		0xFF


//---- typedef ------------------------------------------------------------------------------------

typedef union
{
    unsigned short  Word;
    unsigned char   Byte[2];
} UWORD;

typedef union
{
    unsigned long   Long;
    unsigned short  Word[2];
    unsigned char   Byte[4];
} ULONG;


//-------------------------------------------  Input/Output buffers for Standard Mode -----------
#ifdef BYTE_NUM                                 
                                            
  typedef struct						      //-- output buffer -----------------
  {		 									  //	
    uint8_t  Byte [BYTE_NUM];                 //    
  } PROCBUFFER_OUT;							  //
                                            
  typedef struct                              //-- input buffer ------------------
  {											  //
    uint8_t  Byte [BYTE_NUM];                 //     
  } PROCBUFFER_IN;                            //

#endif

 
#endif








