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
#include "main.h"

#define BUFFER_SIZE	3


static XSpi spi;
static XGpio gpio;




u8 ReadBuffer[BUFFER_SIZE];
u8 WriteBuffer[BUFFER_SIZE];

const u32 DIRECTION_MASK = 0b0000110010110010100;

const u32 CLKGEN_RST_PIN = 1 << 0;
const u32 CLKGEN_OE_PIN = 1 << 1;
const u32 CLKGEN_LOL_PIN = 1 << 2;
const u32 CAMERA_EN_PIN[2] = {1 << 3, 1 << 9};
const u32 CAMERA_PGOOD_PIN[2] = {1 << 4, 1 << 10};
const u32 CAMERA_XCLR_PIN[2] = {1 << 5, 1 << 11};
const u32 CAMERA_XMASTER_PIN[2] = {1 << 6, 1 << 12};
const u32 CAMERA_XHS_PIN[2] = {1 << 7, 1 << 13};
const u32 CAMERA_XVS_PIN[2] = {1 << 8, 1 << 14};
const u32 CAMERA_XTRIG1_PIN[2] = {1 << 15, 1 << 17};
const u32 CAMERA_XTRIG2_PIN[2] = {1 << 16, 1 << 18};

const u32 PGOOD_TIMEOUT = 150; // ms
const u32 XCLR_WAIT = 1; // us
const u32 INCK_WAIT = 2; // us

#define ITERS_PER_SEC Xil_GetRISCVFrequency()
#define ITERS_PER_MSEC ((ITERS_PER_SEC + 500U)/1000U)
#define ITERS_PER_USEC ((ITERS_PER_MSEC + 500U)/1000U)


const u8 CAMERA_PRESENT[2] = {0, 1};

void* const SLVS_EC_BASE[2] = {(void*)XPAR_M_AVALON_0_BASEADDR,(void*)XPAR_M_AVALON_1_BASEADDR};

volatile u32* const CONFIG_REG[2] = {(u32*)(SLVS_EC_BASE[0] + 0x20), (u32*)(SLVS_EC_BASE[1] + 0x20)};
volatile u32* const CTRL_REG[2] = {(u32*)(SLVS_EC_BASE[0] + 0x34), (u32*)(SLVS_EC_BASE[1] + 0x34)};

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
        // we can't check 0 writes, so send them twice
        if ((new == data && data != 0) || (data == 0 && i > 0)) {
            return XST_SUCCESS;
        }
    }
    return XST_FAILURE;
}

const init_reg_t VARIABLE_REGS[] = {
    // VMAX
    {0x2, 0xd4, 0xa8},
    {0x2, 0xd5, 0x08},
    {0x2, 0xd6, 0x00},
    // HMAX
    {0x2, 0xd8, 0x12},
    {0x2, 0xd9, 0x01},
    // GMRWT
    {0x2, 0xe2, 0x08},
    // GMTWT
    {0x2, 0xe3, 0x32},
    // GSDLY
    {0x2, 0xe6, 0x08},
    // SHS. Range [GMTWT+4] to [VMAX-1]
    {0x4, 0x40, 0x40},
    {0x4, 0x41, 0},
    {0x4, 0x42, 0},
    // GAIN_RTS
    {0x7, 0x2, 0x0},
    // GAIN (db*10)
    {0x7, 0x14, 240},
    {0x7, 0x15, 0}
};

// commented out registers are kept at default
const init_reg_t COMMON_SLVS_EC_INIT[] = {
    {0x2, 0x4, 0xa8},
    {0x2, 0x5, 0x2},
    // {0x2, 0x14, /* FILLME */},
    // {0x2, 0x15, /* FILLME */},
    // {0x2, 0x16, /* FILLME */},
    // {0x2, 0x18, /* FILLME */},
    // {0x2, 0x19, /* FILLME */},
    // {0x2, 0x1b, /* FILLME */},
    {0x2, 0x3c, 0x3},
    {0x2, 0xd0, 0xa8},
    {0x2, 0xd1, 0x9},
    {0x2, 0xd2, 0xa8},
    {0x2, 0xd3, 0x9},
    {0x2, 0xdc, 0x0},
    {0x2, 0xe5, 0x2},
    // {0x4, 0x1c, /* FILLME */},
    // {0x4, 0x1e, /* FILLME */},
    // {0x4, 0x1f, /* FILLME */},
    // {0x4, 0x20, /* FILLME */},
    // {0x4, 0x22, /* FILLME */},
    // {0x4, 0x23, /* FILLME */},
    // {0x4, 0x26, /* FILLME */},
    {0x4, 0x27, 0xc0},
    {0x4, 0x33, 0x00},
    {0x4, 0x3c, 0x19},
    {0x4, 0x3e, 0x33},
    {0x7, 0x21, 0x3d},
    {0x7, 0x35, 0x0},
    {0x7, 0x42, 0x27},
    {0x7, 0x46, 0xf},
    {0x7, 0x4a, 0x20},
    {0x7, 0x9c, 0xf},
    {0x7, 0x9d, 0x2},
    {0x7, 0xa5, 0x12},
    {0x7, 0xa9, 0x62},
    {0x7, 0xce, 0xe},
    {0x7, 0xed, 0x12},
    {0x7, 0xf0, 0xfb},
    {0x7, 0xf1, 0xb},
    {0x7, 0xf2, 0xfb},
    {0x7, 0xf3, 0xb},
    {0x8, 0x42, 0x10},
    {0x8, 0x6a, 0x2e},
    {0x8, 0x70, 0xc3},
    {0x8, 0x72, 0x5},
    {0x8, 0x74, 0xb6},
    {0x8, 0x75, 0x1},
    {0x8, 0x76, 0x5},
    {0x8, 0x92, 0x10},
    {0x8, 0xf5, 0xf},
    {0x9, 0x97, 0x20},
    {0xb, 0x4, 0x3},
    {0xc, 0xc, 0x18},
    {0x10, 0x2e, 0x7},
    {0x10, 0x30, 0x4e},
    {0x10, 0x6e, 0x7},
    {0x10, 0x70, 0x35},
    {0x10, 0x96, 0x1},
    {0x10, 0x9e, 0x38},
    {0x10, 0xa0, 0x4c},
    {0x11, 0x3a, 0x4},
    {0x13, 0x82, 0x0},
    {0x13, 0xa2, 0x3},
    {0x14, 0x32, 0x3c},
    {0x14, 0x35, 0x22},
    {0x15, 0x6, 0x0},
    {0x15, 0x7, 0x0},
    {0x15, 0x8, 0x0},
    {0x15, 0x9, 0x0},
    {0x15, 0x10, 0x4},
    {0x15, 0x11, 0x4},
    {0x15, 0x12, 0x4},
    {0x15, 0x13, 0x4},
    {0x15, 0x1e, 0x16},
    {0x15, 0x1f, 0x16},
    {0x15, 0x3c, 0x8a},
    {0x15, 0x3d, 0x2},
    {0x15, 0x3e, 0xe8},
    {0x15, 0x3f, 0x5},
    {0x15, 0x40, 0x9e},
    {0x15, 0x41, 0xc},
    {0x16, 0x6a, 0x4c},
    {0x16, 0x6e, 0x51},
    {0x16, 0x72, 0x57},
    {0x16, 0x76, 0x79},
    {0x16, 0x8a, 0x4c},
    {0x16, 0x8e, 0x51},
    {0x16, 0x92, 0x57},
    {0x16, 0x96, 0x79},
    {0x16, 0xec, 0x3f},
    {0x16, 0xf0, 0x44},
    {0x16, 0xf4, 0x4a},
    {0x17, 0x10, 0x3f},
    {0x17, 0x14, 0x44},
    {0x17, 0x18, 0x4a},
    {0x17, 0x76, 0xbe},
    {0x17, 0x7a, 0xb1},
    {0x17, 0x80, 0xbc},
    {0x17, 0x84, 0xaf},
    {0x19, 0x3c, 0x6},
    {0x19, 0x3d, 0x6},
    {0x19, 0x3e, 0x6},
    {0x19, 0x3f, 0x6},
    {0x19, 0x49, 0x9f},
    {0x19, 0x4a, 0x99},
    {0x19, 0x4b, 0x9},
    {0x19, 0x53, 0x90},
    {0x19, 0x54, 0x99},
    {0x19, 0x55, 0x9},
    {0x19, 0x88, 0x4},
    {0x1a, 0x64, 0xdc},
    {0x1a, 0x68, 0xdc},
    {0x1a, 0x6c, 0xdc},
    {0x1a, 0x74, 0xdc},
    {0x1a, 0x78, 0xdc},
    {0x1a, 0x7c, 0xdc},
    {0x1a, 0xa4, 0xf4},
    {0x1a, 0xa8, 0xf4},
    {0x1a, 0xac, 0xf4},
    {0x1a, 0xb4, 0xf4},
    {0x1a, 0xb8, 0xf4},
    {0x1a, 0xbc, 0xf4},
    {0x1b, 0x1, 0xa},
    {0x1b, 0x2, 0x1},
    {0x1b, 0x16, 0x0},
    {0x1b, 0x17, 0x0},
    {0x1b, 0x18, 0xff},
    {0x1b, 0x19, 0xf},
    {0x1b, 0x1e, 0xff},
    {0x1b, 0x1f, 0xf},
    {0x1b, 0x20, 0x0},
    {0x1b, 0x21, 0x0},
    {0x1b, 0x26, 0xff},
    {0x1b, 0x27, 0xf},
    {0x1b, 0x28, 0x0},
    {0x1b, 0x29, 0x0},
    {0x1c, 0x34, 0xa}
};

const init_reg_t SLVS_EC_8BIT_INIT[] =  {
{0x4, 0x0, 0x25},
{0x6, 0x30, 0x2},
{0x7, 0xa4, 0x8},
{0x7, 0xa8, 0x8},
{0x7, 0xb4, 0xf},
{0x7, 0xec, 0x8},
{0x8, 0x2e, 0x24},
{0x8, 0x56, 0x44},
{0x8, 0x7e, 0x24},
{0x8, 0xe8, 0x11},
{0x12, 0x56, 0x23},
{0x12, 0x96, 0x23},
{0x16, 0x60, 0x6c},
{0x19, 0x2e, 0x6},
{0x19, 0x2f, 0x6},
{0x19, 0x30, 0x6},
{0x19, 0x31, 0x6},
{0x1b, 0x0, 0x44},
{0x1b, 0x8, 0x6e}
};  

// we only support 4752 2lane
const init_reg_t SLVS_EC_10BIT_INIT[] = {
    {0x4, 0x0, 0x5},
    {0x7, 0xa4, 0x1c},
    {0x7, 0xa8, 0x1c},
    {0x7, 0xec, 0x1c},

    {0x8, 0x2c, 0x1c},
    {0x8, 0x2e, 0xeb},
    {0x8, 0x2f, 0x1f},

    {0x8, 0x54, 0x1c},
    {0x8, 0x56, 0xeb},
    {0x8, 0x57, 0x1f},

    {0x8, 0x7c, 0x1c},
    {0x8, 0x7e, 0xeb},
    {0x8, 0x7f, 0x1f},

    {0x8, 0xe8, 0x11},
    // default
    // {0x12, 0x56, /* FILLME */},
    // {0x12, 0x96, /* FILLME */},
    {0x16, 0x60, 0x6c},

    {0x17, 0xe6, 0x53},
    {0x17, 0xf0, 0x90},
    {0x17, 0xf2, 0x8A},
    {0x17, 0xf8, 0x8E},
    {0x17, 0xfa, 0x90},



    {0x18, 0x4, 0x8e},
    {0x18, 0x6, 0x90},
    {0x18, 0xc, 0x8a},
    {0x18, 0xe, 0xbb},
    {0x18, 0x14, 0x90},
    {0x18, 0x16, 0x8a},
    {0x18, 0x34, 0x4a},
    {0x18, 0x36, 0x90},
    {0x18, 0x3c, 0x4c},
    {0x18, 0x3e, 0x92},
    {0x18, 0x44, 0x4e},
    {0x18, 0x46, 0x94},
    {0x18, 0x4c, 0x47},
    {0x18, 0x4e, 0x4d},
    {0x18, 0x54, 0x49},
    {0x18, 0x56, 0x50},
    {0x18, 0x5c, 0x4b},
    {0x18, 0x5e, 0x52},
    {0x18, 0x6a, 0x9e},
    {0x18, 0x70, 0x98},
    {0x18, 0x76, 0x96},
    {0x18, 0x78, 0xba},
    {0x18, 0x98, 0x93},
    {0x18, 0x9a, 0xb9},

    {0x19, 0x28, 0xd4},
    {0x19, 0x29, 0xe},
    {0x19, 0x2f, 0x4},
    {0x19, 0x30, 0x4},
    {0x19, 0x31, 0x4},
    {0x1b, 0x0, 0x64},
    {0x1b, 0x8, 0x6e}
};

const init_reg_t SLVS_EC_12BIT_INIT[] = {
    {0x4, 0x0, 0x15},
    {0x6, 0x30, 0x1},
    {0x7, 0xa4, 0x8},
    {0x7, 0xa8, 0x8},
    {0x7, 0xb4, 0xf0},
    {0x7, 0xec, 0x8},
    {0x8, 0x2e, 0x24},
    {0x8, 0x56, 0x44},
    {0x8, 0x7e, 0x24},
    {0x8, 0xe8, 0x13},
    {0x12, 0x56, 0x23},
    {0x12, 0x96, 0x23},
    {0x19, 0x2e, 0x6},
    {0x19, 0x2f, 0x6},
    {0x19, 0x30, 0x6},
    {0x19, 0x31, 0x6},
    {0x1b, 0x0, 0x6c},
    {0x1b, 0x8, 0x68}
};

#define ARRAY_LEN(x) (sizeof(x)/sizeof(*x))


u32 transmitRegister(const init_reg_t* regs, int len) {
    for (int i = 0; i < len; i++) {
        init_reg_t reg = regs[i];
        if (writeRegister(reg.chipId, reg.address, reg.value) != XST_SUCCESS) {
            return XST_FAILURE;
        }
    }

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
    *CTRL_REG[0] = 0x0;
    *CTRL_REG[1] = 0x0;

    *CTRL_REG[0] = 0x1;
    *CTRL_REG[0] = 0x0;

    *CTRL_REG[1] = 0x1;
    *CTRL_REG[1] = 0x0;
    usleep(1000*1000);
    XGpio_DiscreteSet(&gpio, 1, CLKGEN_RST_PIN | CLKGEN_OE_PIN);
    
    *CONFIG_REG[0] = 0x09a80212;
    *CONFIG_REG[1] = 0x09a80212;
    

    XSpi_SetOptions(&spi,XSP_MASTER_OPTION | XSP_CLK_ACTIVE_LOW_OPTION | XSP_CLK_PHASE_1_OPTION | XSP_MANUAL_SSELECT_OPTION);
    



    u32 writeAllowedTimeout[2] = {0, 0};
    u32 temperatureTimer[2] = {0,0};
    u8 prevVs[2] = {0, 0};
    u8 temperature[2] = {0,0};
    while(1) {
        u32 val = XGpio_DiscreteRead(&gpio, 1);

        if (state == SEQ_STATE_READY) {
            for (u8 i = 0; i < 2; i++) {
                if (!(val & CAMERA_XVS_PIN[i]) && prevVs[i]) {
                    prevVs[i] = 0;
                    writeAllowedTimeout[i] = rdtime() + ITERS_PER_USEC * 14;
                } else if (val & CAMERA_XVS_PIN[i]) {
                    prevVs[i] = 1;
                }
            

                // TODO: write is also allowed after comm prohibited period (4 to GMRWT+GMTWT+4 [H] after VS)
                if (rdtime() < writeAllowedTimeout[i]) {
                    if (temperatureTimer[i] == 0) {
                        XSpi_SetSlaveSelect(&spi, 0b10 << i);
                        writeRegisterOneShot(0x07, 0x96, 1);
                        temperatureTimer[i] = rdtime() + ITERS_PER_USEC * 250;
                    }

                    if (rdtime() > temperatureTimer[i]) {
                        temperature[i] = readRegister(0x07, 0x94); 
                        temperatureTimer[i] = 0;
                    }
                }
            }
        }

        switch(state) {
            case SEQ_STATE_INIT:
                if (cameraEnable) {
                    state = SEQ_STATE_EN_INIT;
                }
                break;
            case SEQ_STATE_EN_INIT:
                XGpio_DiscreteSet(&gpio, 1, CAMERA_EN_PIN[0] | CAMERA_EN_PIN[1]);
                state = SEQ_STATE_EN_WAIT_POWERON;
                cameraTimeStart = rdtime();
                cameraTimeEnd = cameraTimeStart + ITERS_PER_MSEC * PGOOD_TIMEOUT;
                break;
            case SEQ_STATE_EN_WAIT_POWERON:
                cameraPowerGood[0] = (val & CAMERA_PGOOD_PIN[0]) && 1;
                cameraPowerGood[1] = (val & CAMERA_PGOOD_PIN[1]) && 1;

                if ((!CAMERA_PRESENT[0] || cameraPowerGood[0]) && (!CAMERA_PRESENT[1] || cameraPowerGood[1])) {
                    state = SEQ_STATE_EN_WAIT_XCLR;
                    cameraTimeStart = rdtime();
                    cameraTimeEnd = cameraTimeStart + ITERS_PER_USEC * XCLR_WAIT;

                    XGpio_DiscreteSet(&gpio, 1, CAMERA_XTRIG1_PIN[0] | CAMERA_XTRIG2_PIN[0]
                        |  CAMERA_XTRIG1_PIN[1] | CAMERA_XTRIG2_PIN[1]);

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
                    XGpio_DiscreteSet(&gpio, 1, CAMERA_XCLR_PIN[0]|CAMERA_XCLR_PIN[1]);
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
                    if (transmitRegister(COMMON_SLVS_EC_INIT, ARRAY_LEN(COMMON_SLVS_EC_INIT)) != XST_SUCCESS) {
                        state = SEQ_STATE_FAIL;
                        break;
                    };
                    if (transmitRegister(SLVS_EC_10BIT_INIT, ARRAY_LEN(SLVS_EC_10BIT_INIT)) != XST_SUCCESS) {
                        state = SEQ_STATE_FAIL;
                        break;
                    };
                    if (transmitRegister(VARIABLE_REGS, ARRAY_LEN(VARIABLE_REGS)) != XST_SUCCESS) {
                        state = SEQ_STATE_FAIL;
                        break;
                    };
                    usleep(10*1000);
                    writeRegister(2,0,0);
                    usleep(1138*1000);
                    *CTRL_REG[0] = 0x0101;
                    *CTRL_REG[1] = 0x0101;
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
                cameraPowerGood[0] = (val & CAMERA_PGOOD_PIN[0]) && 1;
                cameraPowerGood[1] = (val & CAMERA_PGOOD_PIN[1]) && 1;
                if (!cameraEnable || (CAMERA_PRESENT[0] && !cameraPowerGood[0]) 
                    || (CAMERA_PRESENT[1] && !cameraPowerGood[1])) {
                    state = SEQ_STATE_DISABLE;
                }

                // if (doRead) {
                //     XAxiVdma_StartReadFrame(&vdma, &vdma_context[0].ReadCfg);
                //     doRead = 0;
                // }
                break;

            case SEQ_STATE_DISABLE:
                XGpio_DiscreteClear(&gpio, 1, CAMERA_XCLR_PIN[0]|CAMERA_XCLR_PIN[1]
                    |CAMERA_XTRIG1_PIN[0]|CAMERA_XTRIG2_PIN[0]|CAMERA_XTRIG1_PIN[1]|CAMERA_XTRIG2_PIN[1]);
                XGpio_DiscreteSet(&gpio, 1, CLKGEN_OE_PIN);
                XSpi_Stop(&spi);
                XGpio_DiscreteClear(&gpio, 1, CAMERA_EN_PIN[0]|CAMERA_EN_PIN[1]);
                cameraEnable  = 0;
                state = SEQ_STATE_INIT;
                break;
            case SEQ_STATE_FAIL:
                XGpio_DiscreteClear(&gpio, 1, CAMERA_XCLR_PIN[0]|CAMERA_XCLR_PIN[1]
                    |CAMERA_XTRIG1_PIN[0]|CAMERA_XTRIG2_PIN[0]|CAMERA_XTRIG1_PIN[1]|CAMERA_XTRIG2_PIN[1]);
                XGpio_DiscreteSet(&gpio, 1, CLKGEN_OE_PIN);
                XSpi_Stop(&spi);
                XGpio_DiscreteClear(&gpio, 1, CAMERA_EN_PIN[0]|CAMERA_EN_PIN[1]);
                cameraEnable = 0;
                break;
        }


    }
}