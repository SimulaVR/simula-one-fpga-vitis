#include "xparameters.h"
#include "xgpio.h"
#include "sleep.h"
#include "xiltimer.h"
#include "xspi.h"		/* SPI device driver */
#include "xspi_l.h"

#define BUFFER_SIZE	3


static XSpi spi;
static XGpio gpio;


u8 ReadBuffer[BUFFER_SIZE];
u8 WriteBuffer[BUFFER_SIZE];

const u32 DIRECTION_MASK = 0b110010110010100;

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

const u32 PGOOD_TIMEOUT = 150; // ms
const u32 XCLR_WAIT = 1; // us
const u32 INCK_WAIT = 2; // us

#define ITERS_PER_SEC Xil_GetRISCVFrequency()
#define ITERS_PER_MSEC ((ITERS_PER_SEC + 500U)/1000U)
#define ITERS_PER_USEC ((ITERS_PER_MSEC + 500U)/1000U)


const u8 CAMERA_1_PRESENT = 0;
const u8 CAMERA_2_PRESENT = 1;

u8 cameraEnable = 1;
u8 camera1PowerGood = 0;
u8 camera2PowerGood = 0;

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

void writeRegister(u8 chipId, u8 addr, u8 data) {
    WriteBuffer[0] = chipId;
    WriteBuffer[1] = addr;
    WriteBuffer[2] = data;
    XSpi_Transfer(&spi, WriteBuffer, ReadBuffer, BUFFER_SIZE);
}

void programInitialEC10bit() {
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
writeRegister(0x2, 0xd4, 0xa8);
writeRegister(0x2, 0xd5, 0x08);
writeRegister(0x2, 0xd6, 0x00);
writeRegister(0x2, 0xd8, 0x12);
writeRegister(0x2, 0xd9, 0x01);
writeRegister(0x2, 0xdc, 0x0);
writeRegister(0x2, 0xe2, 0x8);
writeRegister(0x2, 0xe3, 0x32);
writeRegister(0x2, 0xe5, 0x2);
writeRegister(0x2, 0xe6, 0x08);
writeRegister(0x4, 0x0, 0x0);
writeRegister(0x4, 0x1c, 0x80);
writeRegister(0x4, 0x1e, 0xe0);
writeRegister(0x4, 0x1f, 0x00);
writeRegister(0x4, 0x20, 0x80);
writeRegister(0x4, 0x22, 0xe0);
writeRegister(0x4, 0x23, 0x00);
writeRegister(0x4, 0x26, 0x20);
writeRegister(0x4, 0x27, 0xc0);
writeRegister(0x4, 0x33, 0x00);
writeRegister(0x4, 0x3c, 0x19); // 0x0c in ds
writeRegister(0x4, 0x3e, 0x33); // 0x0c in ds
writeRegister(0x4, 0x40, 0x70);
writeRegister(0x4, 0x41, 0x00);
writeRegister(0x4, 0x42, 0x00);
writeRegister(0x7, 0x2, 0x08);
writeRegister(0x7, 0x14, 0xf0);
writeRegister(0x7, 0x15, 0x00);
writeRegister(0x7, 0x21, 0x3d);
writeRegister(0x7, 0x35, 0x00);
writeRegister(0x7, 0x42, 0x27);
writeRegister(0x7, 0x46, 0xf);
writeRegister(0x7, 0x4a, 0x20);
writeRegister(0x7, 0x9c, 0xf);
writeRegister(0x7, 0x9d, 0x2);
writeRegister(0x7, 0xa4, 0x1c);
writeRegister(0x7, 0xa5, 0x12);
writeRegister(0x7, 0xa8, 0x1c);
writeRegister(0x7, 0xa9, 0x62);
writeRegister(0x7, 0xce, 0xe);
writeRegister(0x7, 0xec, 0x1c);
writeRegister(0x7, 0xed, 0x12);
writeRegister(0x7, 0xf0, 0xfb);
writeRegister(0x7, 0xf1, 0xb);
writeRegister(0x7, 0xf2, 0xfb);
writeRegister(0x7, 0xf3, 0xb);
writeRegister(0x8, 0x2c, 0x1C);
writeRegister(0x8, 0x2e, 0xEB);
writeRegister(0x8, 0x2f, 0x1F);
writeRegister(0x8, 0x42, 0x10);
writeRegister(0x8, 0x54, 0x1C);
writeRegister(0x8, 0x56, 0xEB);
writeRegister(0x8, 0x57, 0x1F);
writeRegister(0x8, 0x6a, 0x2e);
writeRegister(0x8, 0x70, 0xc3);
writeRegister(0x8, 0x72, 0x5);
writeRegister(0x8, 0x74, 0xb6);
writeRegister(0x8, 0x75, 0x1);
writeRegister(0x8, 0x76, 0x5);
writeRegister(0x8, 0x7c, 0x1C);
writeRegister(0x8, 0x7e, 0xEB);
writeRegister(0x8, 0x7f, 0x1F);
writeRegister(0x8, 0x92, 0x10);
writeRegister(0x8, 0xe8, 0x11);
writeRegister(0x8, 0xf5, 0xf);
writeRegister(0x9, 0x97, 0x20);
writeRegister(0xb, 0x4, 0x3);
writeRegister(0xc, 0xc, 0x18);
writeRegister(0x10, 0x2e, 0x7);
writeRegister(0x10, 0x30, 0x4e);
writeRegister(0x10, 0x6e, 0x7);
writeRegister(0x10, 0x70, 0x35);
writeRegister(0x10, 0x96, 0x1);
writeRegister(0x10, 0x9e, 0x38);
writeRegister(0x10, 0xa0, 0x4c);
writeRegister(0x11, 0x3a, 0x4);
writeRegister(0x12, 0x56, 0xf);
writeRegister(0x12, 0x96, 0xf);
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


    
}

int main () {

    int status = XGpio_Initialize(&gpio, XPAR_XGPIO_0_BASEADDR);
    if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    status = XSpi_Initialize(&spi, XPAR_XSPI_0_BASEADDR);
    if (status != XST_SUCCESS) {
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

    XGpio_DiscreteClear(&gpio, 1, 0xffff);
    usleep(200*1000);
    XGpio_DiscreteSet(&gpio, 1, CLKGEN_RST_PIN | CLKGEN_OE_PIN);
    

    XSpi_SetOptions(&spi,XSP_MASTER_OPTION | XSP_CLK_ACTIVE_LOW_OPTION  | XSP_MANUAL_SSELECT_OPTION);
    u32 ctr = 0;
    u8 prevHs = 0;
    while(1) {
        u32 val = XGpio_DiscreteRead(&gpio, 1);
        if (state == SEQ_STATE_READY && !(val & CAMERA_2_XHS_PIN) && prevHs) {
            ctr++;
            prevHs = 0;
        } else if (state == SEQ_STATE_READY && (val & CAMERA_2_XHS_PIN)) {
            prevHs = 1;
        }
        if (!prevHs && ctr > 0 && (ctr % 100) == 0) {
            print("ctr");
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
                camera1PowerGood = (val & CAMERA_1_PGOOD_PIN) && 1;
                camera2PowerGood = (val & CAMERA_2_PGOOD_PIN) && 1;
                if ((!CAMERA_1_PRESENT || camera1PowerGood) && (!CAMERA_2_PRESENT || camera2PowerGood)) {
                    state = SEQ_STATE_EN_WAIT_XCLR;
                    cameraTimeStart = rdtime();
                    cameraTimeEnd = cameraTimeStart + ITERS_PER_USEC * XCLR_WAIT;

                    XSpi_Start(&spi);
                    XSpi_IntrGlobalDisable(&spi);
                    XSpi_SetOptions(&spi,XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
                    u32 ControlReg = XSpi_GetControlReg(&spi);
                    ControlReg &= ~ XSP_CR_LSB_MSB_FIRST_MASK;
                    XSpi_SetControlReg(&spi, ControlReg);
                    XSpi_SetSlaveSelect(&spi, 1);
                    writeRegister(0x01, 0xff, 0x03);
                    writeRegister(0x01, 0x2d, 0x0);
                    writeRegister(0x01, 0x2f, 0x0);
                    XSpi_SetSlaveSelect(&spi,0); 
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
                    usleep(20000);
                    state = SEQ_STATE_EN_WAIT_STANDBY;
                }
                break;
            case SEQ_STATE_EN_WAIT_STANDBY:
                XSpi_SetSlaveSelect(&spi, 0b100);
                programInitialEC10bit();
                writeRegister(2,0,0);
                usleep(1138*1000);
                writeRegister(2, 0x10, 0);
                XSpi_SetSlaveSelect(&spi, 0);
                state = SEQ_STATE_READY;
                break;
            case SEQ_STATE_READY:
                camera1PowerGood = (val & CAMERA_1_PGOOD_PIN) && 1;
                camera2PowerGood = (val & CAMERA_2_PGOOD_PIN) && 1;
                if (!cameraEnable || (CAMERA_1_PRESENT && !camera1PowerGood) 
                    || (CAMERA_2_PRESENT && !camera2PowerGood)) {
                    state = SEQ_STATE_DISABLE;
                }
                break;
            case SEQ_STATE_DISABLE:
                XGpio_DiscreteClear(&gpio, 1, CAMERA_1_EN_PIN|CAMERA_1_XCLR_PIN
                    |CAMERA_2_EN_PIN|CAMERA_2_XCLR_PIN);
                XGpio_DiscreteSet(&gpio, 1, CLKGEN_OE_PIN);
                XSpi_Stop(&spi);
                cameraEnable  = 0;
                state = SEQ_STATE_INIT;
                break;
            case SEQ_STATE_FAIL:
                XGpio_DiscreteClear(&gpio, 1, CAMERA_1_EN_PIN|CAMERA_1_XCLR_PIN
                    |CAMERA_2_EN_PIN|CAMERA_2_XCLR_PIN);
                XGpio_DiscreteSet(&gpio, 1, CLKGEN_OE_PIN);
                XSpi_Stop(&spi);
                cameraEnable = 0;
                break;
        }


    }
}