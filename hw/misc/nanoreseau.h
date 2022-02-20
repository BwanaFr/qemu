#ifndef _NANORESEAU_H__
#define _NANORESEAU_H__

#include <stdint.h>

//Registers definition

typedef union NECTRLReg0
{
    struct{
        uint8_t regPointer : 3;
        uint8_t command : 3;
        uint8_t crcCtrlCmd : 2;
    }fields;
    uint8_t value;
}NECTRLReg0;

typedef union NECTRLReg1
{
    struct{
        uint8_t extIntEnable : 1;           //Ext/Status INT enable
        uint8_t txIntEnable : 1;            //Transmitter interrupt enable
        uint8_t statusVector : 1;           //Conditions affects vector Channel B only
        uint8_t rcvInterruptMode : 2;       //Receiver interrupt mode
        uint8_t waitRcvTrans : 1;           //Wait on receive transmitter
        uint8_t txByteCountModeEnable : 1;  //TX byte count mode enable
        uint8_t waitFunctionEnable : 1;     //Wait function enable
    }fields;
    uint8_t value;
}NECTRLReg1;

typedef union NECTRLReg2
{
    union {
        struct{
            uint8_t dmaModeSelect : 2;  //DMA mode select
            uint8_t priority : 1;       //Priority
            uint8_t intVector : 3;      //Interrupt vector
            uint8_t rxIntMask : 1;      //RX INT mask
            uint8_t pin10SYNCB_RTSB : 1;//Pin 10 ~SYNCB/~RTSB
        }fieldsChanA;
        struct{
            uint8_t intVector0 : 1;  //Interrupt vector 0
            uint8_t intVector1 : 1;  //Interrupt vector 1
            uint8_t intVector2 : 1;  //Interrupt vector 2
            uint8_t intVector3 : 1;  //Interrupt vector 3
            uint8_t intVector4 : 1;  //Interrupt vector 4
            uint8_t intVector5 : 1;  //Interrupt vector 5
            uint8_t intVector6 : 1;  //Interrupt vector 6
            uint8_t intVector7 : 1;  //Interrupt vector 7            
        }fieldsChanB;
    }fields;
    uint8_t value;
}NECTRLReg2;

typedef union NECTRLReg3
{
    struct{
        uint8_t receiverEnable : 1;         //Receiver enabled
        uint8_t syncCharLoadInh : 1;        //Sync character Load inhibit
        uint8_t addrSearchMode : 1;         //Address search mode
        uint8_t receiverCRCEnable : 1;      //Receiver CRC enable
        uint8_t enterHuntPhase : 1;         //Enter in hunt phase
        uint8_t autoEnables : 1;            //Auto Enables
        uint8_t nbRecvBitsPerChar : 2;      //Number of received bits per character
    }fields;
    uint8_t value;
}NECTRLReg3;

typedef union NECTRLReg4
{
    struct{
        uint8_t parityEnable : 1;           //Parity enabled
        uint8_t parityEvenOdd : 1;          //Parity even/odd
        uint8_t numberOfStopBits : 2;       //Number of stops bits per sync Mode
        uint8_t syncMode : 2;               //Sync mode
        uint8_t clockRate : 2;              //Clock rate
    }fields;
    uint8_t value;
}NECTRLReg4;

typedef union NECTRLReg5
{
    struct{
        uint8_t transmitterCRCEnable : 1;   //Transmitter CRC enable
        uint8_t rts : 1;                    //RTS
        uint8_t crcPolynomial : 1;          //CRC polynomial select
        uint8_t transmitterEnable : 1;      //Transmitter enable
        uint8_t sendBreak : 1;              //Send break
        uint8_t nbBitsPerChar : 2;          //Number of bits per characters
        uint8_t dtr : 1;                    //DTR
    }fields;
    uint8_t value;
}NECTRLReg5;

typedef union NECTRLReg6
{
    uint8_t syncByte1;      //Sync byte 1   
    uint8_t value;
}NECTRLReg6;

typedef union NECTRLReg7
{
    uint8_t syncByte2;      //Sync byte 2  
    uint8_t value;
}NECTRLReg7;

typedef union NECStatusReg0
{
    struct{
        uint8_t recvCharAvailable : 1;  //Rec'd char available
        uint8_t intPending : 1;         //Pending interrupt
        uint8_t txBufferEmpty : 1;      //TX buffer empty
        uint8_t dcd : 1;                //DCD
        uint8_t syncStatus : 1;         //Sync status
        uint8_t cts : 1;                //CTS
        uint8_t idle_crc : 1;           //Idle/CRC
        uint8_t break_abort : 1;        //Break/Abort
    }fields;
    uint8_t value;
}NECStatusReg0;

typedef union NECStatusReg1
{
    struct{
        uint8_t allSent : 1;            //All sent
        uint8_t sdlcResidueCode : 3;    //SDLC residue code
        uint8_t parityError : 1;        //Parity error
        uint8_t overRunError : 1;       //Overrun error
        uint8_t crcFramingError : 1;    //CRC framing error
        uint8_t endOfSDLCFrame : 1;     //End of SDLC frame
    }fields;
    uint8_t value;
}NECStatusReg1;

typedef union NECStatusReg2B
{
    struct{
        uint8_t intVector0 : 1;  //Interrupt vector 0
        uint8_t intVector1 : 1;  //Interrupt vector 1
        uint8_t intVector2 : 1;  //Interrupt vector 2
        uint8_t intVector3 : 1;  //Interrupt vector 3
        uint8_t intVector4 : 1;  //Interrupt vector 4
        uint8_t intVector5 : 1;  //Interrupt vector 5
        uint8_t intVector6 : 1;  //Interrupt vector 6
        uint8_t intVector7 : 1;  //Interrupt vector 7   
    }fields;
    uint8_t value;
}NECStatusReg2B;

typedef union NECStatusReg3
{
    uint8_t txByteCountLow; //TX byte count lower bytes
    uint8_t value;
}NECStatusReg3;

typedef union NECStatusReg4
{
    uint8_t txByteCountHigh; //TX byte count higher bytes
    uint8_t value;
}NECStatusReg4;

typedef union NECAddressing
{
    struct{
        uint8_t controlNotData : 1;     //Control or data
        uint8_t channelBNotA : 1;       //Channel B selected
    }fields;
    uint8_t value;
}NECAddressing;

#endif

