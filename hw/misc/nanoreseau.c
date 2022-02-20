/*
 * Nanoreseau poste maitre emulation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "sysemu/reset.h"
#include "chardev/char-fe.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "nanoreseau.h"

#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KBNRM  "\x1B[40m"
#define KBRED  "\x1B[41m"
#define KBGRN  "\x1B[42m"
#define KBYEL  "\x1B[43m"
#define KBBLU  "\x1B[44m"
#define KBMAG  "\x1B[45m"
#define KBCYN  "\x1B[46m"
#define KBWHT  "\x1B[47m"
#define KRESET "\x1B[0m"

#define TYPE_ISA_NANORESEAU_DEVICE "isa-nanoreseau"
OBJECT_DECLARE_SIMPLE_TYPE(ISANanoreseau, ISA_NANORESEAU_DEVICE)


typedef struct NECRegisters{
    NECTRLReg0 ctrlReg0;
    NECTRLReg1 ctrlReg1;
    NECTRLReg2 ctrlReg2;
    NECTRLReg3 ctrlReg3;
    NECTRLReg4 ctrlReg4;
    NECTRLReg5 ctrlReg5;
    NECTRLReg6 ctrlReg6;
    NECTRLReg7 ctrlReg7;
    NECStatusReg0 statusReg0;
    NECStatusReg1 statusReg1;
    NECStatusReg2B statusReg2B;
    NECStatusReg3 statusReg3;
    NECStatusReg4 statusReg4;
}NECRegisters;

typedef struct ISANanoreseauState {
    MemoryRegion io;
    qemu_irq pic;
    IsaDma *isa_dma;
    CharBackend chr;
    QEMUTimer* clkTimer;
    NECRegisters regsChanA;
    NECRegisters regsChanB;
    uint32_t writeCount;

    bool clkDetected;       //Clock detected (DCD input)
    bool clkSilence;        //Clock silence detected (CTS input)
} ISANanoreseauState;

struct ISANanoreseau {
    ISADevice parent_obj;

    uint32_t iobase;
    uint32_t iosize;
    uint32_t irq;
    uint32_t dma;
    
    ISANanoreseauState state;
};

static void printValue(const char* value)
{
    printf(" \x1B[47;1m\x1B[30;1m%s" KRESET " ", value);
}

static void printCTRLReg0(NECTRLReg0* reg)
{
    printf("\tCTRLReg0 -> Reg pointer : %u command :", reg->fields.regPointer);
    switch(reg->fields.command){
        case 0b000:
            printValue("Null");
            break;
        case 0b001:
            printValue("Send_abort");
            break;
        case 0b010:
            printValue("Reset_external_status_interrupt");
            break;
        case 0b011:
            printValue("Channel_reset");
            break;
        case 0b100:
            printValue("Enable_INT_on_next_char");
            break;
        case 0b101:
            printValue("DMA_request");
            break;
        case 0b110:
            printValue("Error_reset");
            break;
        case 0b111:
            printValue("End_of_interrupt");
            break;
        default:
            printValue("Unknown");
    }
    printf("CRC control command :");
    switch(reg->fields.crcCtrlCmd){
        case 0b00:
            printValue("Null");
            break;
        case 0b01:
            printValue("Reset_receiver_CRC_checker");
            break;
        case 0b10:
            printValue("Reset_transmitter_CRC_generator");
            break;
        case 0b11:
            printValue("Reset_idle/CRC_latch");
            break;
        default:
            printValue("Unknown");
    }
    printf("\n");
}

static void printCTRLReg5(NECTRLReg5* reg)
{
    printf("\tCTRLReg5 ->");
    if(reg->fields.transmitterCRCEnable){
        printValue("transmitter_CRC_enabled");
    }else{
        printValue("transmitter_CRC_disabled");
    }
    if(reg->fields.rts){
        printValue("RTS_low");
    }else{
        printValue("RTS_high");
    }
    if(reg->fields.crcPolynomial){
        printValue("CRC-16");
    }else{
        printValue("CRC-CCITT");
    }
    if(reg->fields.transmitterEnable){
        printValue("transmitter_enabled");
    }else{
        printValue("transmitter_disabled");
    }
    if(reg->fields.sendBreak){
        printValue("send_break");
    }else{
        printValue("no_send_break");
    }
    char str[64];
    sprintf(str, "Bit per char %u", reg->fields.nbBitsPerChar);
    printValue(str);
    if(reg->fields.dtr){
        printValue("DTR");
    }else{
        printValue("no_DTR");
    }
    printf("\n");
}

static void printCTRLReg4(NECTRLReg4* reg)
{
    printf("\tCTRLReg4 ->");
    if(reg->fields.parityEnable){
        printf(" parity ");
    }else{
        printf(" no_parity ");
    }
    if(reg->fields.parityEvenOdd){
        printf(" even ");
    }else{
        printf(" odd ");
    }
    printf(" %u stop bits ", reg->fields.numberOfStopBits);
    switch(reg->fields.syncMode){
        case 0b00:
            printf(" monosync ");
            break;
        case 0b01:
            printf(" bisync ");
            break;
        case 0b10:
            printf(" SDLC/HDLC ");
            break;
        case 0b11:
            printf(" ext_sync ");
            break;
    }
    switch(reg->fields.clockRate){
        case 0b00:
            printf(" clock_x1 ");
            break;
        case 0b01:
            printf(" clock_x16 ");
            break;
        case 0b10:
            printf(" clock_x32 ");
            break;
        case 0b11:
            printf(" clock_x64 ");
            break;
    }
    printf("\n");
}

static void printCTRLReg3(NECTRLReg3* reg)
{
    printf("\tCTRLReg3 ->");
    if(reg->fields.receiverEnable){
        printf(" receiver_enabled ");
    }
    if(reg->fields.syncCharLoadInh){
        printf(" sync_char_inhib ");
    }
    if(reg->fields.addrSearchMode){
        printf(" addr_search_mode ");
    }
    if(reg->fields.receiverCRCEnable){
        printf(" receiver_crc_enabled ");
    }
    if(reg->fields.enterHuntPhase){
        printf(" enter_hunt_phase ");
    }
    if(reg->fields.autoEnables){
        printf(" auto_enables ");
    }
    printf(" bits_per_char_%u ", reg->fields.nbRecvBitsPerChar);

    printf("\n");
}

static void nanoreseau_set_clock(ISANanoreseauState* state, bool active)
{
    state->clkDetected = active;
    if(!state->clkDetected){
        state->clkSilence = false;
        //Silence will be active 
        timer_mod(state->clkTimer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200000);
    }
}

static uint64_t nanoreseau_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t ret = 0;
    ISANanoreseauState* state = opaque;
    if(addr < 4){
        NECAddressing necAddr;
        necAddr.value = addr & 0x3;
        if(necAddr.fields.controlNotData){
            //Select register set
            NECRegisters* reg = &state->regsChanA;
            if(necAddr.fields.channelBNotA){
                reg = &state->regsChanB;
            }
            uint8_t regPointer = reg->ctrlReg0.fields.regPointer;
            printf("Read channel %c status register %u ", 
                    (necAddr.fields.channelBNotA ? 'B' : 'A'),
                    regPointer);
            switch(regPointer){
                case 0:
                {
                    ret = reg->statusReg0.value;
                }
                break;
                case 1:
                {
                    ret = reg->statusReg1.value;
                }
                break;
                case 2:
                {
                    if(necAddr.fields.channelBNotA)
                    {
                        ret = reg->statusReg2B.value;
                    }
                }
                break;
                case 3:
                {
                    ret = reg->statusReg3.value;
                }
                break;
                case 4:
                {
                    ret = reg->statusReg4.value;
                }
                break;
            }
            if(regPointer != 0){
                reg->ctrlReg0.fields.regPointer = 0x0;
            }
            printf(" --> 0x%x\n", (uint8_t)(ret & 0xFF));
        }else{
            //Read data
            printf("Read data\n");
        }
    }else{
        printf("Read addr > 4!\n");
    }
    return ret;
}

static void nanoreseau_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned width)
{
    ISANanoreseauState* state = opaque;
    ++state->writeCount;
    if(addr < 4){
        NECAddressing necAddr;
        necAddr.value = addr & 0x3;
        if(necAddr.fields.controlNotData){
            //Select register set
            NECRegisters* reg = &state->regsChanA;
            if(necAddr.fields.channelBNotA){
                reg = &state->regsChanB;
            }
            uint8_t regPointer = reg->ctrlReg0.fields.regPointer;
            printf("Write[%u] 0x%x to channel %s control register %u\n",
                state->writeCount,
                ((uint8_t)val), (necAddr.fields.channelBNotA ? KBLU "B" KRESET : KYEL "A" KRESET),
                regPointer);
            switch(regPointer){
                case 0:
                {
                    reg->ctrlReg0.value = val & 0xFF;
                    printCTRLReg0(&reg->ctrlReg0);
                }
                break;
                case 1:
                {
                    reg->ctrlReg1.value = val & 0xFF;
                }
                break;
                case 2:
                {
                    reg->ctrlReg2.value = val & 0xFF;
                    if(necAddr.fields.channelBNotA)
                    {
                        reg->statusReg2B.value = val & 0xFF;
                    }
                }
                break;
                case 3:
                {
                    reg->ctrlReg3.value = val & 0xFF;
                    printCTRLReg3(&reg->ctrlReg3);
                }
                break;
                case 4:
                {
                    reg->ctrlReg4.value = val & 0xFF;
                    printCTRLReg4(&reg->ctrlReg4);
                }
                break;
                case 5:
                {
                    reg->ctrlReg5.value = val & 0xFF;
                    printCTRLReg5(&reg->ctrlReg5);
                    if(reg->ctrlReg5.fields.rts){
                        //RTS active it means we activate the TX
                        nanoreseau_set_clock(state, true);
                    }else{
                        //Disable the TX
                        nanoreseau_set_clock(state, false);
                    }
                    if(reg->ctrlReg5.fields.dtr){
                        //DTR active means the clock active detection is activated

                    }else{
                        //DTR not active, clock detection not activated
                    }
                }
                break;
                case 6:
                {
                    reg->ctrlReg6.value = val & 0xFF;
                }
                break;
                case 7:
                {
                    reg->ctrlReg7.value = val & 0xFF;
                }
                break;
            }
            if(regPointer != 0){
                reg->ctrlReg0.fields.regPointer = 0x0;
            }
        }else{
            //Write data
        }
    }else{
        printf("Addr > 4!\n");
    }
}

static int nanoreseau_can_receive(void *opaque)
{
    ISANanoreseauState *s = opaque;
    //printf("nanoreseau_can_receive\n");
    return 0;
}

static void nanoreseau_receive(void *opaque, const uint8_t *buf, int size)
{
    printf("nanoreseau_receive -> ");
    for(int i=0;i<size;++i){
        printf("%c", buf[i]);
    }
    printf("\n");
}

static int nanoreseau_dma_read(void *opaque, int nchan, int dma_pos, int dma_len)
{
    printf("DMA read!");
    return dma_pos;
}

static void nanoreseau_reset(void *opaque)
{
    ISANanoreseauState *s = opaque;
    //Default registers value
    s->regsChanA.statusReg0.fields.txBufferEmpty = 1;
    s->regsChanA.statusReg0.fields.idle_crc = 1;
    s->regsChanA.statusReg0.fields.dcd = 1;
    s->regsChanB.statusReg0.value = s->regsChanA.statusReg0.value = 0x28;
}

static void nanoreseau_clk_timer_cb(void *opaque)
{
    ISANanoreseauState *s = opaque;
    printf("Silence timer triggered\n");
    s->clkSilence = true;
}

static const MemoryRegionOps nanoreseau_ops = {
    .read = nanoreseau_read,
    .write = nanoreseau_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nanoreseau_realizefn(DeviceState *d, Error **errp)
{
    ISADevice *dev = ISA_DEVICE(d);
    ISANanoreseau *isa = ISA_NANORESEAU_DEVICE(d);
    ISANanoreseauState *s = &isa->state;
    IsaDmaClass *k;

    //Maybe the chardev is not mandatory
    if (!qemu_chr_fe_backend_connected(&s->chr)) {
        error_setg(errp, "Can't create nanoreseau device, empty char device");
        return;
    }
    //Initialize ISA io region
    memory_region_init_io(&s->io, OBJECT(dev), &nanoreseau_ops, s,
                          TYPE_ISA_NANORESEAU_DEVICE, isa->iosize);
    memory_region_add_subregion(isa_address_space_io(dev),
                                isa->iobase, &s->io);                        
    //Initialize DMA channel for nanoreseau
    s->isa_dma = isa_get_dma(isa_bus_from_device(dev), isa->dma);
    if (!s->isa_dma) {
        error_setg(errp, "ISA controller does not support DMA");
        return;
    }
    k = ISADMA_GET_CLASS(s->isa_dma);
    k->register_channel(s->isa_dma, isa->dma, nanoreseau_dma_read, s);

    //Initialize IRQ line
    isa_init_irq(dev, &s->pic, isa->irq);
    
    //Register handler for chardev
    qemu_chr_fe_set_handlers(&s->chr, nanoreseau_can_receive,
                             nanoreseau_receive, NULL, NULL,
                             s, NULL, true);
    //Register machine reset to put default registers value
    qemu_register_reset(nanoreseau_reset, s);

    //Timer to simulate the clock detection
    s->clkTimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, &nanoreseau_clk_timer_cb, s);
}

static Property nanoreseau_properties[] = {
    DEFINE_PROP_UINT32("iobase", ISANanoreseau, iobase, 0x300),
    DEFINE_PROP_UINT32("iosize", ISANanoreseau, iosize, 0x08),
    DEFINE_PROP_UINT32("dma", ISANanoreseau, dma, 3),
    DEFINE_PROP_UINT32("irq", ISANanoreseau, irq, 2),
    DEFINE_PROP_CHR("chardev", ISANanoreseau, state.chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void nanoreseau_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nanoreseau_realizefn;
    device_class_set_props(dc, nanoreseau_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo nanoreseau_info = {
    .name          = TYPE_ISA_NANORESEAU_DEVICE,
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(ISANanoreseau),
    .class_init    = nanoreseau_class_initfn,
};

static void nanoreseau_register_types(void)
{
    type_register_static(&nanoreseau_info);
}

type_init(nanoreseau_register_types)
