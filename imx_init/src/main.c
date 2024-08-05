#include "xparameters.h"
#include "xgpio.h"
#include "sleep.h"
#include "xiltimer.h"
#include "xspi.h"		/* SPI device driver */
#include "xspi_l.h"
#include "xaxivdma.h"

#include "xil_cache.h"

#include "xil_util.h"
#include <xparameters.h>

#include "vdma.h"

#define BUFFER_SIZE	3


static XSpi spi;
static XGpio gpio;




u8 ReadBuffer[BUFFER_SIZE];
u8 WriteBuffer[BUFFER_SIZE];

const u32 DIRECTION_MASK = 0b0000110010110010100;

const u32 CLKGEN_RST_PIN = 1 << 0;
const u32 CLKGEN_OE_PIN = 1 << 1;
const u32 CLKGEN_LOL_PIN = 1 << 2;
const u32 CAMERA_1_EN_PIN = 1 << 3;
const u32 CAMERA_1_PGOOD_PIN = 1 << 4;
const u32 CAMERA_1_XCLR_PIN = 1 << 5;
const u32 CAMERA_1_XMASTER_PIN = 1 << 6;
const u32 CAMERA_1_XHS_PIN = 1 << 7;
const u32 CAMERA_1_XVS_PIN = 1 << 8;
const u32 CAMERA_2_EN_PIN = 1 << 9;
const u32 CAMERA_2_PGOOD_PIN = 1 << 10;
const u32 CAMERA_2_XCLR_PIN = 1 << 11;
const u32 CAMERA_2_XMASTER_PIN = 1 << 12;
const u32 CAMERA_2_XHS_PIN = 1 << 13;
const u32 CAMERA_2_XVS_PIN = 1 << 14;

const u32 CAMERA_1_XTRIG1_PIN = 1 << 15;
const u32 CAMERA_1_XTRIG2_PIN = 1 << 16;
const u32 CAMERA_2_XTRIG1_PIN = 1 << 17;
const u32 CAMERA_2_XTRIG2_PIN = 1 << 18;

const u32 PGOOD_TIMEOUT = 150; // ms
const u32 XCLR_WAIT = 1; // us
const u32 INCK_WAIT = 2; // us

#define ITERS_PER_SEC Xil_GetRISCVFrequency()
#define ITERS_PER_MSEC ((ITERS_PER_SEC + 500U)/1000U)
#define ITERS_PER_USEC ((ITERS_PER_MSEC + 500U)/1000U)


const u8 CAMERA_1_PRESENT = 0;
const u8 CAMERA_2_PRESENT = 1;

void* const SLVS_EC_BASE_0 = XPAR_M_AVALON_0_BASEADDR;
void* const SLVS_EC_BASE_1 = XPAR_M_AVALON_1_BASEADDR;

volatile u32* const CONFIG_REG_0 = (u32*)(SLVS_EC_BASE_0 + 0x20);
volatile u32* const CTRL_REG_0 = (u32*)(SLVS_EC_BASE_0 + 0x34);
volatile u32* const CONFIG_REG_1 = (u32*)(SLVS_EC_BASE_1 + 0x20);
volatile u32* const CTRL_REG_1 = (u32*)(SLVS_EC_BASE_1 + 0x34);

u8 cameraEnable = 1;
u8 cameraPowerGood[2]  = {0, 0};

typedef enum {
    SEQ_STATE_INIT,
    SEQ_STATE_EN_INIT,
    SEQ_STATE_EN_WAIT_POWERON,
    SEQ_STATE_EN_WAIT_XCLR,
    SEQ_STATE_EN_WAIT_INCK,
    SEQ_STATE_EN_WAIT_STANDBY,
    SEQ_STATE_READY,
    SEQ_STATE_DISABLE,
    SEQ_STATE_FAIL
} SequencingState;

SequencingState state = SEQ_STATE_INIT;

u8 nibble_to_hex(u8 c) {
    c &= 0x0f;
    c += (c <= 9) ? '0' : 'A' - 10;
    return c;
}


const u32 RETRY_COUNT = 3;

u8 readRegister(u8 chipId, u8 addr) {
    WriteBuffer[0] = chipId | 0x80;
    WriteBuffer[1] = addr;
    WriteBuffer[2] = 0;
    XSpi_Transfer(&spi, WriteBuffer, ReadBuffer, BUFFER_SIZE);
    XSpi_SetSlaveSelectReg(&spi, 0xffffffff);
    return ReadBuffer[2];
}


// returns old value for IMX547
u8 writeRegisterOneShot(u8 chipId, u8 addr, u8 data) {
    WriteBuffer[0] = chipId;
    WriteBuffer[1] = addr;
    WriteBuffer[2] = data;
    XSpi_Transfer(&spi, WriteBuffer, ReadBuffer, BUFFER_SIZE);
    XSpi_SetSlaveSelectReg(&spi, 0xffffffff);
    return ReadBuffer[2];
}

// XST_SUCCESS/XST_FAILURE
u32 writeRegister(u8 chipId, u8 addr, u8 data) {
    for (u8 i = 0; i < RETRY_COUNT; i++) {
        writeRegisterOneShot(chipId, addr, data);
        u8 new = readRegister(chipId, addr);
        if (new == data) {
            return XST_SUCCESS;
        }
    }
    return XST_FAILURE;
}

#define TWO_LANE


u32 programInitialEC10bit() {
writeRegister(0x2, 0x4, 0xa8);
writeRegister(0x2, 0x5, 0x2);
writeRegister(0x2, 0x14, 0xa);
writeRegister(0x2, 0x15, 0x22);
writeRegister(0x2, 0x16, 0xb1);
writeRegister(0x2, 0x18, 0x40);
writeRegister(0x2, 0x19, 0x04);
writeRegister(0x2, 0x1b, 0x3a);
writeRegister(0x2, 0x3c, 0x3);
writeRegister(0x2, 0xd0, 0xa8);
writeRegister(0x2, 0xd1, 0x9);
writeRegister(0x2, 0xd2, 0xa8);
writeRegister(0x2, 0xd3, 0x9);
writeRegister(0x2, 0xdc, 0x0);
writeRegister(0x2, 0xe5, 0x2);
writeRegister(0x4, 0x0, 0x05);
writeRegister(0x4, 0x1c, 0x80);
writeRegister(0x4, 0x1e, 0xe0);
writeRegister(0x4, 0x1f, 0x00);
writeRegister(0x4, 0x20, 0x80);
writeRegister(0x4, 0x22, 0xe0);
writeRegister(0x4, 0x23, 0x00);
writeRegister(0x4, 0x26, 0x20);
writeRegister(0x4, 0x27, 0xc0);
writeRegister(0x4, 0x33, 0x00);
writeRegister(0x4, 0x3c, 0x19); 
writeRegister(0x4, 0x3e, 0x33); 
//writeRegister(0x4, 0x40, 100);
//writeRegister(0x4, 0x41, 0);
//writeRegister(0x4, 0x42, 0);
writeRegister(0x7, 0x2, 0x08);
writeRegister(0x7, 0x14, 240); // db/10
writeRegister(0x7, 0x15, 0);
writeRegister(0x7, 0x21, 0x3d);
writeRegister(0x7, 0x35, 0x00);
writeRegister(0x7, 0x42, 0x27);
writeRegister(0x7, 0x46, 0xf);
writeRegister(0x7, 0x4a, 0x20);
writeRegister(0x7, 0x9c, 0); // low power consumption vblank disable
writeRegister(0x7, 0x9d, 0x2);
writeRegister(0x7, 0xa4, 0xff); //hsst1 disable
writeRegister(0x7, 0xa5, 0x0f);

writeRegister(0x7, 0xa8, 0xff); //hsst2 disable
writeRegister(0x7, 0xa9, 0x0f);

writeRegister(0x7, 0xce, 0xe);

writeRegister(0x7, 0xec, 0xff); //hsst3 disable
writeRegister(0x7, 0xed, 0x0f);

writeRegister(0x7, 0xf0, 0xfb);
writeRegister(0x7, 0xf1, 0xb);
writeRegister(0x7, 0xf2, 0xfb);
writeRegister(0x7, 0xf3, 0xb);
#ifdef TWO_LANE



writeRegister(0x2, 0xd4, 0xa8); // 2-lane
writeRegister(0x2, 0xd5, 0x08);
writeRegister(0x2, 0xd6, 0x00);
writeRegister(0x2, 0xd8, 0x12); // 2-lane
writeRegister(0x2, 0xd9, 0x01);

writeRegister(0x2, 0xe2, 0x8); // 2-lane

writeRegister(0x2, 0xe3, 0x32); // 2-lane

writeRegister(0x2, 0xe6, 0x08); // 2-lane
writeRegister(0xb, 0x4, 0x3); // 0x3 = 2-lane, 0x4 = 1-lane

writeRegister(0x8, 0x2c, 0x1C);
writeRegister(0x8, 0x2e, 0xEB);
writeRegister(0x8, 0x2f, 0x1F);

writeRegister(0x8, 0x54, 0x1C);
writeRegister(0x8, 0x56, 0xEB);
writeRegister(0x8, 0x57, 0x1F);

writeRegister(0x8, 0x7c, 0x1C);
writeRegister(0x8, 0x7e, 0xEB);
writeRegister(0x8, 0x7f, 0x1F);

writeRegister(0x17, 0xe6, 0x53);
writeRegister(0x17, 0xf0, 0x90);
writeRegister(0x17, 0xf2, 0x8a);
writeRegister(0x17, 0xf8, 0x8e);
writeRegister(0x17, 0xfa, 0x90);


writeRegister(0x18, 0x4, 0x8e);
writeRegister(0x18, 0x6, 0x90);
writeRegister(0x18, 0xc, 0x8a);
writeRegister(0x18, 0xe, 0xbb);
writeRegister(0x18, 0x14, 0x90);
writeRegister(0x18, 0x16, 0x8a);
writeRegister(0x18, 0x34, 0x4a);
writeRegister(0x18, 0x36, 0x90);
writeRegister(0x18, 0x3c, 0x4c);
writeRegister(0x18, 0x3e, 0x92);
writeRegister(0x18, 0x44, 0x4e);
writeRegister(0x18, 0x46, 0x94);
writeRegister(0x18, 0x4c, 0x47);
writeRegister(0x18, 0x4e, 0x4d);
writeRegister(0x18, 0x54, 0x49);
writeRegister(0x18, 0x56, 0x50);
writeRegister(0x18, 0x5c, 0x4b);
writeRegister(0x18, 0x5e, 0x52);
writeRegister(0x18, 0x6a, 0x9e);
writeRegister(0x18, 0x70, 0x98);
writeRegister(0x18, 0x76, 0x96);
writeRegister(0x18, 0x78, 0xba);
writeRegister(0x18, 0x98, 0x93);
writeRegister(0x18, 0x9a, 0xb9);
#else

writeRegister(0x2, 0xd4, 0x92);  // 1-lane
writeRegister(0x2, 0xd5, 0x08);
writeRegister(0x2, 0xd6, 0x00);

writeRegister(0x2, 0xd8, 0x06); // 1-lane
writeRegister(0x2, 0xd9, 0x02);

writeRegister(0x2, 0xe2, 0x4); // 1-lane

writeRegister(0x2, 0xe3, 0x1c); // 1-lane

writeRegister(0x2, 0xe6, 0x0C); // 1-lane

writeRegister(0xb, 0x4, 0x4); // 0x3 = 2-lane, 0x4 = 1-lane

writeRegister(0x8, 0x2e, 0x24);
writeRegister(0x8, 0x56, 0x44);
writeRegister(0x8, 0x7e, 0x24);

writeRegister(0x12, 0x56, 0x23);
writeRegister(0x12, 0x96, 0x23);


#endif
writeRegister(0x8, 0x42, 0x10);
writeRegister(0x8, 0x6a, 0x2e);
writeRegister(0x8, 0x70, 0xc3);
writeRegister(0x8, 0x72, 0x5);
writeRegister(0x8, 0x74, 0xb6);
writeRegister(0x8, 0x75, 0x1);
writeRegister(0x8, 0x76, 0x5);
writeRegister(0x8, 0x92, 0x10);
writeRegister(0x8, 0xe8, 0x11);
writeRegister(0x8, 0xf5, 0xf);
writeRegister(0x9, 0x97, 0x20);
writeRegister(0xc, 0x0, 0b11100001);
writeRegister(0xc, 0xc, 0x18);
writeRegister(0x10, 0x2e, 0x7);
writeRegister(0x10, 0x30, 0x4e);
writeRegister(0x10, 0x6e, 0x7);
writeRegister(0x10, 0x70, 0x35);
writeRegister(0x10, 0x96, 0x1);
writeRegister(0x10, 0x9e, 0x38);
writeRegister(0x10, 0xa0, 0x4c);
writeRegister(0x11, 0x3a, 0x4);
// writeRegister(0x12, 0x56, 0xf);
// writeRegister(0x12, 0x96, 0xf);
writeRegister(0x13, 0x82, 0x0);
writeRegister(0x13, 0xa2, 0x3);
writeRegister(0x14, 0x32, 0x3c);
writeRegister(0x14, 0x35, 0x22);
writeRegister(0x15, 0x6, 0x0);
writeRegister(0x15, 0x7, 0x0);
writeRegister(0x15, 0x8, 0x0);
writeRegister(0x15, 0x9, 0x0);
writeRegister(0x15, 0x10, 0x4);
writeRegister(0x15, 0x11, 0x4);
writeRegister(0x15, 0x12, 0x4);
writeRegister(0x15, 0x13, 0x4);
writeRegister(0x15, 0x1e, 0x16);
writeRegister(0x15, 0x1f, 0x16);
writeRegister(0x15, 0x3c, 0x8a);
writeRegister(0x15, 0x3d, 0x2);
writeRegister(0x15, 0x3e, 0xe8);
writeRegister(0x15, 0x3f, 0x5);
writeRegister(0x15, 0x40, 0x9e);
writeRegister(0x15, 0x41, 0xc);
writeRegister(0x16, 0x60, 0x6c);
writeRegister(0x16, 0x6a, 0x4c);
writeRegister(0x16, 0x6e, 0x51);
writeRegister(0x16, 0x72, 0x57);
writeRegister(0x16, 0x76, 0x79);
writeRegister(0x16, 0x8a, 0x4c);
writeRegister(0x16, 0x8e, 0x51);
writeRegister(0x16, 0x92, 0x57);
writeRegister(0x16, 0x96, 0x79);
writeRegister(0x16, 0xec, 0x3f);
writeRegister(0x16, 0xf0, 0x44);
writeRegister(0x16, 0xf4, 0x4a);
writeRegister(0x17, 0x10, 0x3f);
writeRegister(0x17, 0x14, 0x44);
writeRegister(0x17, 0x18, 0x4a);
writeRegister(0x17, 0x76, 0xbe);
writeRegister(0x17, 0x7a, 0xb1);
writeRegister(0x17, 0x80, 0xbc);
writeRegister(0x17, 0x84, 0xaf);
writeRegister(0x19, 0x28, 0xd4);
writeRegister(0x19, 0x29, 0xe);
writeRegister(0x19, 0x2f, 0x4);
writeRegister(0x19, 0x30, 0x4);
writeRegister(0x19, 0x31, 0x4);
writeRegister(0x19, 0x3c, 0x6);
writeRegister(0x19, 0x3d, 0x6);
writeRegister(0x19, 0x3e, 0x6);
writeRegister(0x19, 0x3f, 0x6);
writeRegister(0x19, 0x49, 0x9f);
writeRegister(0x19, 0x4a, 0x99);
writeRegister(0x19, 0x4b, 0x9);
writeRegister(0x19, 0x53, 0x90);
writeRegister(0x19, 0x54, 0x99);
writeRegister(0x19, 0x55, 0x9);
writeRegister(0x19, 0x88, 0x4);
writeRegister(0x1a, 0x64, 0xdc);
writeRegister(0x1a, 0x68, 0xdc);
writeRegister(0x1a, 0x6c, 0xdc);
writeRegister(0x1a, 0x74, 0xdc);
writeRegister(0x1a, 0x78, 0xdc);
writeRegister(0x1a, 0x7c, 0xdc);
writeRegister(0x1a, 0xa4, 0xf4);
writeRegister(0x1a, 0xa8, 0xf4);
writeRegister(0x1a, 0xac, 0xf4);
writeRegister(0x1a, 0xb4, 0xf4);
writeRegister(0x1a, 0xb8, 0xf4);
writeRegister(0x1a, 0xbc, 0xf4);
writeRegister(0x1b, 0x0, 0x64);
writeRegister(0x1b, 0x1, 0xa);
writeRegister(0x1b, 0x2, 0x1);
writeRegister(0x1b, 0x8, 0x6e);
writeRegister(0x1b, 0x16, 0);
writeRegister(0x1b, 0x17, 0);
writeRegister(0x1b, 0x18, 0xff);
writeRegister(0x1b, 0x19, 0xf);
writeRegister(0x1b, 0x1e, 0xff);
writeRegister(0x1b, 0x1f, 0xf);
writeRegister(0x1b, 0x20, 0);
writeRegister(0x1b, 0x21, 0);
writeRegister(0x1b, 0x26, 0xff);
writeRegister(0x1b, 0x27, 0xf);
writeRegister(0x1b, 0x28, 0);
writeRegister(0x1b, 0x29, 0);
writeRegister(0x1c, 0x34, 0xa);
return XST_SUCCESS;
}


#define DDR_BASE_ADDR		0x80000000
#define DDR_HIGH_ADDR		0xC0000000



/* Memory space for the frame buffers
 *
 * This example only needs one set of frame buffers, because one video IP is
 * to write to the frame buffers, and the other video IP is to read from the
 * frame buffers.
 *
 * For 16 frames of 1080p, it needs 0x07E90000 memory for frame buffers
 */
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x01000000)
#define MEM_HIGH_ADDR		DDR_HIGH_ADDR
#define MEM_SPACE		(MEM_HIGH_ADDR - MEM_BASE_ADDR)





int run_triple_frame_buffer(XAxiVdma* InstancePtr, int DeviceId, int hsize,
		int vsize, int buf_base_addr, int number_frame_count,
		int enable_frm_cnt_intr);

XAxiVdma vdma;
volatile u32 doRead = 0;

int main () {

	XAxiVdma_FrameCounter FrameCfg;

    int Status = XGpio_Initialize(&gpio, XPAR_XGPIO_0_BASEADDR);
    if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    Status = XSpi_Initialize(&spi, XPAR_XSPI_0_BASEADDR);
    if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}




    /*
    gpio_rtl_tri_io(0) => clkgen_rst,
    gpio_rtl_tri_io(1) => clkgen_output_enable,
    gpio_rtl_tri_io(2) => clkgen_lossoflock,
    gpio_rtl_tri_io(3) => en_cam1,
    gpio_rtl_tri_io(4) => pgood_cam1,
    gpio_rtl_tri_io(5) => xclr_cam1,
    gpio_rtl_tri_io(6) => xmaster_cam1,
    gpio_rtl_tri_io(7) => xhs_cam1,
    gpio_rtl_tri_io(8) => xvs_cam1,
    gpio_rtl_tri_io(9) => en_cam2,
    gpio_rtl_tri_io(10) => pgood_cam2,
    gpio_rtl_tri_io(11) => xclr_cam2,
    gpio_rtl_tri_io(12) => xmaster_cam2,
    gpio_rtl_tri_io(13) => xhs_cam2,
    gpio_rtl_tri_io(14) => xvs_cam2,
    */

    XGpio_SetDataDirection(&gpio, 1, DIRECTION_MASK);

    u32 cameraTimeStart = 0;
    u32 cameraTimeEnd = 0;
    XSpi_Stop(&spi);
    XGpio_DiscreteWrite(&gpio, 1, 0);
    *CTRL_REG_0 = 0x0;
    *CTRL_REG_1 = 0x0;

    *CTRL_REG_0 = 0x1;
    *CTRL_REG_0 = 0x0;
    *CTRL_REG_1 = 0x1;
    *CTRL_REG_1 = 0x0;
    usleep(1000*1000);
    XGpio_DiscreteSet(&gpio, 1, CLKGEN_RST_PIN | CLKGEN_OE_PIN);
    #ifdef TWO_LANE
    *CONFIG_REG_0 = 0x09a80212;
    *CONFIG_REG_1 = 0x09a80212;
    #else
    *CONFIG_REG_0 = 0x09a80211;
    *CONFIG_REG_1 = 0x09a80211; // 1 lane
    #endif
    

    XSpi_SetOptions(&spi,XSP_MASTER_OPTION | XSP_CLK_ACTIVE_LOW_OPTION | XSP_CLK_PHASE_1_OPTION | XSP_MANUAL_SSELECT_OPTION);
    



    u32 ctr = 0;
    u8 prevHs = 0;
    u8 prevVs = 0;
    u8 running = 0;
    u8 writeAllowed[1] = {0,0};
    u8 temperature = 0;
    while(1) {
        u32 val = XGpio_DiscreteRead(&gpio, 1);
        if (state == SEQ_STATE_READY && !(val & CAMERA_1_XVS_PIN) && prevVs) {
            prevVs = 0;
            writeAllowed[0] = 1;
            ctr = 0;
        } else if (state == SEQ_STATE_READY && (val & CAMERA_1_XVS_PIN)) {
            prevVs = 1;
        }
        if (state == SEQ_STATE_READY && !(val & CAMERA_1_XHS_PIN) && prevHs) {
            ctr++;
            prevHs = 0;
        } else if (state == SEQ_STATE_READY && (val & CAMERA_1_XHS_PIN)) {
            prevHs = 1;
        }
        if (!prevHs && ctr > 0 && (ctr % 100) == 0) {
            print("ctr");
        }

        if (writeAllowed[0] && ctr > 4) {
            writeAllowed[0] = 0;
            ctr = 0;
        }

        if(state == SEQ_STATE_READY) { //&& writeAllowed[0]) {
            XSpi_SetSlaveSelect(&spi, 0b100);
            writeRegisterOneShot(0x07, 0x96, 1);
            usleep(250);
            temperature = readRegister(0x07, 0x94); 
            writeAllowed[0] = 0; //todo hack (don't reset it, just cba to make gateware for this)
            ctr = 0;
        }

        switch(state) {
            case SEQ_STATE_INIT:
                if (cameraEnable) {
                    state = SEQ_STATE_EN_INIT;
                }
                break;
            case SEQ_STATE_EN_INIT:
                XGpio_DiscreteSet(&gpio, 1, CAMERA_1_EN_PIN | CAMERA_2_EN_PIN);
                state = SEQ_STATE_EN_WAIT_POWERON;
                cameraTimeStart = rdtime();
                cameraTimeEnd = cameraTimeStart + ITERS_PER_MSEC * PGOOD_TIMEOUT;
                break;
            case SEQ_STATE_EN_WAIT_POWERON:
                cameraPowerGood[0] = (val & CAMERA_1_PGOOD_PIN) && 1;
                cameraPowerGood[1] = (val & CAMERA_2_PGOOD_PIN) && 1;

                if ((!CAMERA_1_PRESENT || cameraPowerGood[0]) && (!CAMERA_2_PRESENT || cameraPowerGood[1])) {
                    state = SEQ_STATE_EN_WAIT_XCLR;
                    cameraTimeStart = rdtime();
                    cameraTimeEnd = cameraTimeStart + ITERS_PER_USEC * XCLR_WAIT;

                    XGpio_DiscreteSet(&gpio, 1, CAMERA_1_XTRIG1_PIN | CAMERA_1_XTRIG2_PIN | CAMERA_2_XTRIG1_PIN | CAMERA_2_XTRIG2_PIN);

                    XSpi_Start(&spi);
                    XSpi_IntrGlobalDisable(&spi);
                    u32 ControlReg = XSpi_GetControlReg(&spi);
                    // ControlReg &= ~XSP_CR_LSB_MSB_FIRST_MASK;
                    // XSpi_SetOptions(&spi,XSP_MASTER_OPTION  | XSP_MANUAL_SSELECT_OPTION);
                    // XSpi_SetSlaveSelect(&spi, 1);
                    // writeRegister(0x00, 0x23, 0x00);
                    // writeRegister(0x01, 0x23, 0x01);
                    // writeRegister(0x00, 0x23, 0x00);
                    // writeRegister(0x00, 0x07, 0x00);
                    // XSpi_SetSlaveSelect(&spi, 0);
                    XSpi_SetOptions(&spi,XSP_MASTER_OPTION | XSP_CLK_ACTIVE_LOW_OPTION | XSP_CLK_PHASE_1_OPTION  | XSP_MANUAL_SSELECT_OPTION);
                    ControlReg = XSpi_GetControlReg(&spi);
                    ControlReg |= XSP_CR_LSB_MSB_FIRST_MASK;
                    XSpi_SetControlReg(&spi, ControlReg);
                    
                } else if (cameraTimeEnd < rdtime()) {
                    state = SEQ_STATE_FAIL;
                }
                break;
            case SEQ_STATE_EN_WAIT_XCLR:
                if (cameraTimeEnd < rdtime()) {
                    XGpio_DiscreteSet(&gpio, 1, CAMERA_1_XCLR_PIN|CAMERA_2_XCLR_PIN);
                    state = SEQ_STATE_EN_WAIT_INCK;
                    cameraTimeStart = rdtime();
                    cameraTimeEnd = cameraTimeStart + ITERS_PER_USEC * INCK_WAIT;
                }
                break;
            case SEQ_STATE_EN_WAIT_INCK:
                if (cameraTimeEnd < rdtime() ) {
                    XGpio_DiscreteClear(&gpio, 1, CLKGEN_OE_PIN);

                    usleep(1000);

                    state = SEQ_STATE_EN_WAIT_STANDBY;
                }
                break;
            case SEQ_STATE_EN_WAIT_STANDBY:
                state = SEQ_STATE_READY;
                for(int cam = 0; cam < 2; cam++) {
                    if (!cameraPowerGood[cam]) {
                        continue;
                    }
                    XSpi_SetSlaveSelect(&spi, 0b10 << cam);
                    if (programInitialEC10bit() != XST_SUCCESS) {
                        state = SEQ_STATE_FAIL;
                        break;
                    };
                    usleep(10*1000);
                    writeRegister(2,0,0);
                    usleep(1138*1000);
                    *CTRL_REG_0 = 0x0101;
                    *CTRL_REG_1 = 0x0101;
                    writeRegister(2, 0x10, 0);
                    XSpi_SetSlaveSelect(&spi, 0);
                    usleep(80*1000);
                    Status = run_triple_frame_buffer(&vdma, 0, 2472, 2128, MEM_BASE_ADDR, 3, 0);

                    if (Status != XST_SUCCESS) {
                       state = SEQ_STATE_FAIL;
                       break;
                    }
                }
                break;
            case SEQ_STATE_READY:
                cameraPowerGood[0] = (val & CAMERA_1_PGOOD_PIN) && 1;
                cameraPowerGood[1] = (val & CAMERA_2_PGOOD_PIN) && 1;
                if (!cameraEnable || (CAMERA_1_PRESENT && !cameraPowerGood[0]) 
                    || (CAMERA_2_PRESENT && !cameraPowerGood[1])) {
                    state = SEQ_STATE_DISABLE;
                }

                // if (doRead) {
                //     XAxiVdma_StartReadFrame(&vdma, &vdma_context[0].ReadCfg);
                //     doRead = 0;
                // }
                break;

            case SEQ_STATE_DISABLE:
                XGpio_DiscreteClear(&gpio, 1, CAMERA_1_XCLR_PIN|CAMERA_2_XCLR_PIN|CAMERA_1_XTRIG1_PIN|CAMERA_1_XTRIG2_PIN|CAMERA_2_XTRIG1_PIN|CAMERA_2_XTRIG2_PIN);
                XGpio_DiscreteSet(&gpio, 1, CLKGEN_OE_PIN);
                XSpi_Stop(&spi);
                XGpio_DiscreteClear(&gpio, 1, CAMERA_1_EN_PIN|CAMERA_2_EN_PIN);
                cameraEnable  = 0;
                state = SEQ_STATE_INIT;
                break;
            case SEQ_STATE_FAIL:
                XGpio_DiscreteClear(&gpio, 1, CAMERA_1_XCLR_PIN|CAMERA_2_XCLR_PIN|CAMERA_1_XTRIG1_PIN|CAMERA_1_XTRIG2_PIN|CAMERA_2_XTRIG1_PIN|CAMERA_2_XTRIG2_PIN);
                XGpio_DiscreteSet(&gpio, 1, CLKGEN_OE_PIN);
                XSpi_Stop(&spi);
                XGpio_DiscreteClear(&gpio, 1, CAMERA_1_EN_PIN|CAMERA_2_EN_PIN);
                cameraEnable = 0;
                break;
        }


    }
}