#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h> 	/* platform operations */
#include <linux/ioport.h> 		/* ioresource type */
#include <linux/irq.h> 			/* irq flags type */
#include <linux/interrupt.h>		/* for interrupt request */
#include <asm/io.h>			/* io operations */
#include <linux/i2c.h> 
#include <linux/of.h>			/* device tree operations */
#include <linux/of_platform.h> 		/* get pdev from node */
#include <linux/clk.h>			/* clock frame-work */
#include <linux/slab.h>			/* dynamic memory alloc */
#include <linux/device.h>		/* device operations */
#include <linux/delay.h>


#define SUN7I_I2C_CTLR_NAME 	"sun7i-i2c"
#define NUM_I2C_CHIPS		2	

/* I2C CONTROLLERS PHYSICAL ADDRESSES */
#define I2C0_ADAP_PA 0x01C2AC00
#define I2C1_ADAP_PA 0x01C2B000
#define I2C2_ADAP_PA 0x01C2B400
#define I2C3_ADAP_PA 0x01C2B800
#define I2C4_ADAP_PA 0x01C2C000

#define I2C_MEM_SIZE 0x3FF

#define I2C0_IRQ_NUM 41
#define I2C1_IRQ_NUM 42
#define I2C2_IRQ_NUM 43
#define I2C3_IRQ_NUM 122
#define I2C4_IRQ_NUM 123

/***********************************************************************/
/* platform data */


/* I2C CONTROLLER REGISTER OFFSETS */
#define I2C_ADDR_REG	0x00
#define I2C_XADDR_REG	0x04
#define I2C_DATA_REG	0x08
#define I2C_CTL_REG	0x0C
#define I2C_STAT_REG	0x10
#define I2C_CLK_REG	0x14
#define I2C_SRST_REG	0x18
#define I2C_EFR_REG 	0x1C
#define I2C_LCR_REG	0x20
#define I2C_DVFS_REG	0x24

/* I2C address register */
#define I2C_GCE_EN		(0x1 <<0) /* general call address enable for slave mode */
#define I2C_ADDR_MASK		(0x7f<<1) /* 7:1bits */
/* 31:8bits reserved */ 


/* I2C extend address register */
#define I2C_XADDR_MASK		(0xff) /* 7:0bits for extend slave address */
/* 31:8bits reserved */


/* I2C Data register default is 0x0000_0000 */
#define I2C_DATA_MASK		(0xff) /* 7:0bits for send or received */

/* I2C Control Register Bit Fields & Masks, default value: 0x0000_0000*/
/* 1:0 bits reserved */
#define I2C_CTL_ACK		(0x1<<2) /* set 1 to send A_ACK,then low level on SDA */
#define I2C_CTL_INTFLG		(0x1<<3) /* INT_FLAG,interrupt status flag: set '1' when interrupt coming */
#define I2C_CTL_STP		(0x1<<4) /* M_STP,Automatic clear 0 */
#define I2C_CTL_STA		(0x1<<5) /* M_STA,atutomatic clear 0 */
#define I2C_CTL_BUSEN		(0x1<<6) /* BUS_EN, master mode should be set 1.*/
#define I2C_CTL_INTEN		(0x1<<7) /* INT_EN */
/* 31:8 bit reserved */

/* I2C Status Register Bit Fields & Masks  */
#define I2C_STAT_MASK                   (0xff)
/* 7:0 bits use only,default is 0xF8 */
#define I2C_STAT_BUS_ERR                (0x00)  /* BUS ERROR */
/* Master mode use only */
#define I2C_STAT_TX_STA                 (0x08)  /* START condition transmitted */
#define I2C_STAT_TX_RESTA               (0x10)  /* Repeated START condition transmitted */
#define I2C_STAT_TX_AW_ACK              (0x18)  /* Address+Write bit transmitted, ACK received */
#define I2C_STAT_TX_AW_NAK              (0x20)  /* Address+Write bit transmitted, ACK not received */
#define I2C_STAT_TXD_ACK                (0x28)  /* data byte transmitted in master mode,ack received */
#define I2C_STAT_TXD_NAK                (0x30)  /* data byte transmitted in master mode ,ack not received */
#define I2C_STAT_ARBLOST                (0x38)  /* arbitration lost in address or data byte */
#define I2C_STAT_TX_AR_ACK              (0x40)  /* Address+Read bit transmitted, ACK received */
#define I2C_STAT_TX_AR_NAK              (0x48)  /* Address+Read bit transmitted, ACK not received */
#define I2C_STAT_RXD_ACK                (0x50)  /* data byte received in master mode ,ack transmitted */
#define I2C_STAT_RXD_NAK                (0x58)  /* date byte received in master mode,not ack transmitted */
/* Slave mode use only */
#define I2C_STAT_RXWS_ACK               (0x60)  /* Slave address+Write bit received, ACK transmitted */
#define I2C_STAT_ARBLOST_RXWS_ACK       (0x68)
#define I2C_STAT_RXGCAS_ACK             (0x70)  /* General Call address received, ACK transmitted */
#define I2C_STAT_ARBLOST_RXGCAS_ACK     (0x78)
#define I2C_STAT_RXDS_ACK               (0x80)
#define I2C_STAT_RXDS_NAK               (0x88)
#define I2C_STAT_RXDGCAS_ACK            (0x90)
#define I2C_STAT_RXDGCAS_NAK            (0x98)
#define I2C_STAT_RXSTPS_RXRESTAS        (0xA0)
#define I2C_STAT_RXRS_ACK               (0xA8)

#define I2C_STAT_ARBLOST_SLAR_ACK       (0xB0)
/* 10bit Address, second part of address */
#define I2C_STAT_TX_SAW_ACK             (0xD0)  /* Second Address byte+Write bit transmitted,ACK received */
#define I2C_STAT_TX_SAW_NAK             (0xD8)  /* Second Address byte+Write bit transmitted,ACK not received */

#define I2C_STAT_IDLE                   (0xF8)  /* No relevant status infomation,INT_FLAG = 0 */


/* status or interrupt source */
/*------------------------------------------------------------------------------
* Code   Status
* 00h    Bus error
* 08h    START condition transmitted
* 10h    Repeated START condition transmitted
* 18h    Address + Write bit transmitted, ACK received
* 20h    Address + Write bit transmitted, ACK not received
* 28h    Data byte transmitted in master mode, ACK received
* 30h    Data byte transmitted in master mode, ACK not received
* 38h    Arbitration lost in address or data byte
* 40h    Address + Read bit transmitted, ACK received
* 48h    Address + Read bit transmitted, ACK not received
* 50h    Data byte received in master mode, ACK transmitted
* 58h    Data byte received in master mode, not ACK transmitted
* 60h    Slave address + Write bit received, ACK transmitted
* 68h    Arbitration lost in address as master, slave address + Write bit received, ACK transmitted
* 70h    General Call address received, ACK transmitted
* 78h    Arbitration lost in address as master, General Call address received, ACK transmitted
* 80h    Data byte received after slave address received, ACK transmitted
* 88h    Data byte received after slave address received, not ACK transmitted
* 90h    Data byte received after General Call received, ACK transmitted
* 98h    Data byte received after General Call received, not ACK transmitted
* A0h    STOP or repeated START condition received in slave mode
* A8h    Slave address + Read bit received, ACK transmitted
* B0h    Arbitration lost in address as master, slave address + Read bit received, ACK transmitted
* B8h    Data byte transmitted in slave mode, ACK received
* C0h    Data byte transmitted in slave mode, ACK not received
* C8h    Last byte transmitted in slave mode, ACK received
* D0h    Second Address byte + Write bit transmitted, ACK received
* D8h    Second Address byte + Write bit transmitted, ACK not received
* F8h    No relevant status information or no interrupt
*-----------------------------------------------------------------------------*/

/* I2C mode select */
#define I2C_MASTER_MODE                 (1)
#define I2C_SLAVE_MODE                  (0)     /* seldom use */

#define SUN7I_I2C_ADDR_SIZE             0x3FF

#define I2C0_BASE_ADDR_START  (SW_PA_I2C0_IO_BASE )
#define I2C0_BASE_ADDR_END    (I2C0_BASE_ADDR_START + SUN7I_I2C_ADDR_SIZE)

#define I2C1_BASE_ADDR_START  (SW_PA_I2C1_IO_BASE )
#define I2C1_BASE_ADDR_END    (I2C1_BASE_ADDR_START + SUN7I_I2C_ADDR_SIZE)

#define I2C2_BASE_ADDR_START  (SW_PA_I2C2_IO_BASE )
#define I2C2_BASE_ADDR_END    (I2C2_BASE_ADDR_START + SUN7I_I2C_ADDR_SIZE)

#define I2C3_BASE_ADDR_START  (SW_PA_I2C3_IO_BASE )
#define I2C3_BASE_ADDR_END    (I2C3_BASE_ADDR_START + SUN7I_I2C_ADDR_SIZE)

#define I2C4_BASE_ADDR_START  (SW_PA_I2C4_IO_BASE )
#define I2C4_BASE_ADDR_END    (I2C4_BASE_ADDR_START + SUN7I_I2C_ADDR_SIZE)


/* I2C Clock Register Bit Fields & Masks,default value:0x0000_0000 */
/*
Fin is APB CLOCK INPUT;
Fsample = F0 = Fin/2^CLK_N; 
          F1 = F0/(CLK_M+1);
          
Foscl = F1/10 = Fin/(2^CLK_N * (CLK_M+1)*10); 
Foscl is clock SCL;standard mode:100KHz or fast mode:400KHz        
*/
#define I2C_CLK_DIV_M		(0xF<<3) /* 6:3bit  */
#define I2C_CLK_DIV_N		(0x7<<0) /* 2:0bit */


/* I2C Soft Reset Register Bit Fields & Masks  */
#define I2C_SRST_SRST		(0x1<<0) /* write 1 to clear 0, when complete soft reset clear 0 */

/* I2C Enhance Feature Register Bit Fields & Masks  */
/* default -- 0x0 */
#define I2C_EFR_MASK		(0x3<<0)/* 00:no,01: 1byte, 10:2 bytes, 11: 3bytes */
#define I2C_EFR_WARC_0		(0x0<<0)
#define I2C_EFR_WARC_1		(0x1<<0)
#define I2C_EFR_WARC_2		(0x2<<0)
#define I2C_EFR_WARC_3		(0x3<<0)

/* I2C line control register -default value: 0x0000_003a */
#define I2C_LCR_SDA_EN                  (0x01<<0)       /* SDA line state control enable ,1:enable;0:disable */
#define I2C_LCR_SDA_CTL                 (0x01<<1)       /* SDA line state control bit, 1:high level;0:low level */
#define I2C_LCR_SCL_EN                  (0x01<<2)       /* SCL line state control enable ,1:enable;0:disable */
#define I2C_LCR_SCL_CTL                 (0x01<<3)       /* SCL line state control bit, 1:high level;0:low level */
#define I2C_LCR_SDA_STATE_MASK          (0x01<<4)   /* current state of SDA,readonly bit */
#define I2C_LCR_SCL_STATE_MASK          (0x01<<5)   /* current state of SCL,readonly bit */
/* 31:6bits reserved */
#define I2C_LCR_IDLE_STATUS             (0x3a)



struct sunxi_i2c_platform_data {
	int bus_num;
	unsigned int frequency;
};

#define I2C_TRANSFER_SPEED_400K (400000)
#define I2C_TRANSFER_SPEED_200K (200000)
#define I2C_TRANSFER_SPEED_100K (100000)
/****************************************************************/

#ifndef CONFIG_I2C_SUN7I
#define DRIVER_OUTOF_TREE
#endif

//#define SUNXI_I2C_DEBUG

#ifdef SUNXI_I2C_DEBUG
#define i2c_dbg(x...)   printk(x)
#else
#define i2c_dbg(x...)
#endif

#define CONFIG_SUNXI_IIC_PRINT_TRANSFER_INFO

#define AWXX_I2C_OK      0
#define AWXX_I2C_FAIL   -1
#define AWXX_I2C_RETRY  -2
#define AWXX_I2C_SFAIL  -3  /* start fail */
#define AWXX_I2C_TFAIL  -4  /* stop  fail */

/* i2c_adapter: transfer status */
enum
{
	I2C_XFER_IDLE    = 0x1,
	I2C_XFER_START   = 0x2,
	I2C_XFER_RUNNING = 0x4,
};


/* private structure of I2C, to store important data */
struct sunxi_i2c {

	int 			bus_num;
	unsigned int		status; /* start, running, idle */
	unsigned int		suspend_flag;

	spinlock_t		lock; /* syn */
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	struct i2c_adapter	adap;

	struct clk		*clk;
	unsigned int		bus_freq;

	void __iomem		*base_addr;

	unsigned long		iobase; // for remove
	unsigned long		iosize; // for remove

	int			irq;

	unsigned int 		debug_state; /* log the i2c machine state */

};
/***************************{ IO Operations }**************************/

/*******************{ STEP-7 }*****************************/

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Enabling the I2C BUS */
static void i2c_bus_enable(void *base_addr)
{
	unsigned int data = 0;
	data = readl(base_addr + I2C_CTL_REG);
	data |= I2C_CTL_BUSEN;
	writel(data, base_addr + I2C_CTL_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Disabling the I2C BUS */
static void i2c_bus_disable(void *base_addr)
{
	unsigned int data = 0;
	data = readl(base_addr + I2C_CTL_REG);
	data &= ~(I2C_CTL_BUSEN);
	writel(data, base_addr + I2C_CTL_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* SET the CLOCK REG for set the I2C Bus Spped */
static void i2c_clock_set(void *base_addr,int clk_m, int clk_n)
{
	unsigned int data = 0;

	data = readl(base_addr + I2C_CLK_REG);
	data &= ~(I2C_CLK_DIV_M | I2C_CLK_DIV_N); // masking clk_m, clk_n
	data |= (clk_m << 0x03) | clk_n;

	writel(data, base_addr + I2C_CLK_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* RESET the I2C contoller */
static inline void i2c_soft_reset(void *base_addr)
{
        unsigned int data = readl(base_addr + I2C_SRST_REG);
        data |= I2C_SRST_SRST; /* set soft reset bit,0x0000 0001 */
        writel(data, base_addr + I2C_SRST_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Get interrupt status */
static inline unsigned int i2c_get_irq_status(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_STAT_REG);
	return (data & I2C_STAT_MASK);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Get interrupt flag status */
static inline unsigned int i2c_get_irq_flag(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	return (data & I2C_CTL_INTFLG);//0x00001000
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Get the LCR reg value */
static inline unsigned int i2c_get_lcr_val(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_LCR_REG);
	return data;
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Enable SDA or SCL cntrol in LCR reg */
static void i2c_enable_lcr(void *base_addr, unsigned int sda_scl)
{
	unsigned int data = readl(base_addr + I2C_LCR_REG);

	if(sda_scl & 0x01) {
		data |= I2C_LCR_SCL_EN;/* enable scl line control */
	} else {
		data |= I2C_LCR_SDA_EN;/* enable sda line control */
	}
	writel(data, base_addr + I2C_LCR_REG);
}
	
/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Disable SDA or SCL control in LCR reg */
static void i2c_disable_lcr(void *base_addr, unsigned int sda_scl)
{
	unsigned int data = readl(base_addr + I2C_LCR_REG);

	if(sda_scl & 0x01) {
		data &= ~I2C_LCR_SCL_EN;/* disable scl line control */
	} else {
		data &= ~I2C_LCR_SDA_EN;/* disable sda line control */
	}
	writel(data, base_addr + I2C_LCR_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Get SDA state from LCR reg */
static unsigned int i2c_get_sda(void *base_addr)
{
	unsigned int status = 0;
	status = I2C_LCR_SDA_STATE_MASK & readl(base_addr + I2C_LCR_REG);
	status >>= 4;
	return (status & 0x1);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Set SCL level(high/low) manually, in LCR reg */
static void i2c_set_scl(void *base_addr, unsigned int hi_lo)
{
	unsigned int data = readl(base_addr + I2C_LCR_REG);
	data &= ~I2C_LCR_SCL_CTL;
	hi_lo   &= 0x01;
	data |= (hi_lo << 3);
	writel(data, base_addr + I2C_LCR_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Enable the IRQ for I2C comm. */
static inline void i2c_enable_irq(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);

	/*
	 * 1. when enable irq for next operation, set intflag to 1 to prevent to clear it by a mistake
	 *    (intflag bit is write-0-to-clear bit)
	 * 2. Similarly, mask startbit and stopbit to prevent to set it twice by a mistake
	 *    (start bit and stop bit are self-clear-to-0 bits)
	 */
	data |= (I2C_CTL_INTEN | I2C_CTL_INTFLG);
	data &= ~(I2C_CTL_STA | I2C_CTL_STP);
	writel(data, base_addr + I2C_CTL_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Disable the IRQ for I2C comm */
static inline void i2c_disable_irq(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data &= ~I2C_CTL_INTEN;
	writel(data, base_addr + I2C_CTL_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* it starts i2c comm., the start bit will be cleared automatically */
static inline void i2c_set_start(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data |= I2C_CTL_STA;
	writel(data, base_addr + I2C_CTL_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* get start bit status, poll if start signal is sent */
static inline unsigned int i2c_get_start(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data >>= 5;
	return data & 1;
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* trigger stop signal, the stop bit will be cleared automatically */
static inline void i2c_set_stop(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data |= I2C_CTL_STP;
	writel(data, base_addr + I2C_CTL_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* get stop bit status, poll if stop signal is sent */
static inline unsigned int i2c_get_stop(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data >>= 4;
	return data & 1;
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* Disabling Acknowledgement, used for reading last byte of I2C comm. */
static inline void i2c_disable_ack(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data &= ~I2C_CTL_ACK;
	writel(data, base_addr + I2C_CTL_REG);
	return;
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* when sending ack or nack, it will send ack automatically */
static inline void i2c_enable_ack(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data |= I2C_CTL_ACK;
	writel(data, base_addr + I2C_CTL_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* clear the interrupt flag */
static inline void i2c_clear_irq_flag(void *base_addr)
{
	unsigned int data = readl(base_addr + I2C_CTL_REG);
	data &= ~I2C_CTL_INTFLG;//0x 1111_0111
	writel(data ,base_addr + I2C_CTL_REG);

	/* read two more times to make sure that interrupt flag does really be cleared */
	{
		unsigned int temp;
		temp = readl(base_addr + I2C_CTL_REG);
		temp |= readl(base_addr + I2C_CTL_REG);
	}
}

/* Set the val in Enhanced Feature Register */
static inline void i2c_set_EFR(void *base_addr, unsigned int efr)
{
	/* read the data from EFR reg*/
	unsigned int data = readl(base_addr + I2C_EFR_REG);
	
	/* clear the DBN bit*/
	data &= ~I2C_EFR_MASK;
	/* Except the bit, clear all data */
	efr     &= I2C_EFR_MASK;
	/* update the value of bit to the data */
	data |= efr;
	/* write to the EFR reg */
	writel(data, base_addr + I2C_EFR_REG);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* get data first, then clear flag */
static inline void i2c_get_byte(void *base_addr, unsigned char  *buffer)
{
	/* get the LSByte of reg */
	*buffer = (unsigned char)( I2C_DATA_MASK & readl(base_addr + I2C_DATA_REG) );
	/* clear interrupt flag for next byte interrupt */
	i2c_clear_irq_flag(base_addr);
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* only get data, we will clear the flag when stop */
static inline void i2c_get_last_byte(void *base_addr, unsigned char  *buffer)
{
	*buffer = (unsigned char)( I2C_DATA_MASK & readl(base_addr + I2C_DATA_REG) );
}

/* TODO: Update the comments for below code, by ref'ng data sheet */
/* write data and clear irq flag to trigger send flow */
static inline void i2c_put_byte(void *base_addr, const unsigned char *buffer)
{
	writel((unsigned int)*buffer, base_addr + I2C_DATA_REG);
	i2c_clear_irq_flag(base_addr);
}
/***************************{ IO Operations }**************************/

/*******************{ STEP-7 }*****************************/

/*******************{ I2C Functionality }********************************/

/* Function start i2c first time */
static int i2c_start(void *base_addr)
{
	/* timeout count */
	unsigned int timeout = 0xff;

	/* with in timeout count, it should start */
	i2c_set_start(base_addr);
	while((1 == i2c_get_start(base_addr))&&(--timeout));
	if(timeout == 0) {
		i2c_dbg("[I2C]: START can't sendout!\n");
		return AWXX_I2C_FAIL;
	}

	return AWXX_I2C_OK;
}

/* Function to restart the I2C (basically used for read operation) */
static int i2c_restart(void  *base_addr)
{
	/* Timeout count */
	unsigned int timeout = 0xff;
	/* start the i2c */
	i2c_set_start(base_addr);
	/* clear the flags irq previous flag for next i2c comm */
	i2c_clear_irq_flag(base_addr);
	/* Checking the i2c has start with in timeout */
	while((1 == i2c_get_start(base_addr))&&(--timeout));
	if(timeout == 0) {
		/* if not start, return the error */
		i2c_dbg("[I2C]: Restart can't sendout!\n");
		return AWXX_I2C_FAIL;
	}
	/* if start, return I2C_OK */
	return AWXX_I2C_OK;
}

/* Function to stop I2C communication of controller */
static int i2c_stop(void *base_addr)
{
	/* count for generate some dalay */
	unsigned int timeout = 0xff;
	/* function disables stops the i2c */
	i2c_set_stop(base_addr);
	/* Clear the irq flag, to start next i2c comm. */
	i2c_clear_irq_flag(base_addr);

	/* it must delay 1 nop to check stop bit */
	i2c_get_stop(base_addr);
	/* delay, for above operation and check stop status */
	while(( 1 == i2c_get_stop(base_addr))&& (--timeout));
	if(timeout == 0) {
		i2c_dbg("[I2C]: BUS: STOP can't sendout!\n",);
		return AWXX_I2C_FAIL;
	}

	/* Checkthe status reg, fot bus interrupt IDLE state */
	timeout = 0xffff;

	while((I2C_STAT_IDLE != i2c_get_irq_status(base_addr)) && (--timeout));
	if(timeout == 0)
	{
		i2c_dbg("[I2C]: INT: State isn't idle(0xf8)\n");
		return AWXX_I2C_FAIL;
	}
	/* Get LCR reg status fot validation for bus IDLE state */
	timeout = 0xff;
	while((I2C_LCR_IDLE_STATUS != i2c_get_lcr_val(base_addr))&&(--timeout));
	if(timeout == 0) {
		i2c_dbg("[I2C]: LCR: STOP can't sendout!\n");
		return AWXX_I2C_FAIL;
	}
	/* If all states are well, return I2C_OK */
	return AWXX_I2C_OK;
}

/* Contoroling clock of I2C manually to set the bus into IDLE state */
static int i2c_send_clk_9pulse(void *base_addr, int bus_num)
{
	int i2c_scl = 1;
	int low = 0;
	int high = 1;
	int cycle = 0;

	/* enable scl control */
	i2c_enable_lcr(base_addr, i2c_scl);

	while(cycle < 10) {
		i2c_dbg("[I2C-%d]: cycle=%d\n",bus_num, cycle);
		if( i2c_get_sda(base_addr)
		&& i2c_get_sda(base_addr)
		&& i2c_get_sda(base_addr) ) {
			break;
		}
		/* i2c_scl -> low */
		i2c_set_scl(base_addr, low);
		udelay(1000);

		/* i2c_scl -> high */
		i2c_set_scl(base_addr, high);
		udelay(1000);
		cycle++;
	}

	if(i2c_get_sda(base_addr)){
		i2c_disable_lcr(base_addr, i2c_scl);
		return AWXX_I2C_OK;
	} else {
		i2c_dbg("[i2c%d] SDA is still Stuck Low, failed. \n", bus_num);
		i2c_disable_lcr(base_addr, i2c_scl);
		return AWXX_I2C_FAIL;
	}
}
/*******************{ I2C Functionality }********************************/


/**********************{ Algorithm }******************************/

/* It is Enabled by interrupt when the operation success */
static int i2c_sunxi_xfer_complete(struct sunxi_i2c *i2c_data, int code)
{
	int ret = AWXX_I2C_OK;
	/* Releasing all msg related to stuff */
	i2c_data->msg     = NULL;
	i2c_data->msg_num = 0;/* Important to satisfy conditio in sleeping work function */
	i2c_data->msg_ptr = 0;
	i2c_data->status  = I2C_XFER_IDLE;

	/* i2c_data->msg_idx  store the information */
	if(code == AWXX_I2C_FAIL) {
		i2c_dbg("[I2C-%d]: Maybe Logic Error\n",i2c_data->bus_num);
		i2c_data->msg_idx = code;
		ret = AWXX_I2C_FAIL;
	}
	else if(code != AWXX_I2C_OK) {
		/* return the ERROR code, for debug or detect error type */
		i2c_data->msg_idx = code;
		ret = AWXX_I2C_FAIL;
	}

	/* Wake-Up the slept function */
	wake_up(&i2c_data->wait);

	return ret;
}

/*
 * This is the core function of I2C transfer/receive.
 * And it is called by 'i2c_sunxi_xfer' function.
 */
static int i2c_sunxi_do_xfer(struct sunxi_i2c *i2c_data, struct i2c_msg *msgs, int num)
{
	unsigned long timeout = 0;
	int ret = AWXX_I2C_FAIL;

	/* Reset the i2c controller */
	i2c_soft_reset(i2c_data->base_addr);
	udelay(100);

/* FIXME: while condition logic */
	/* test the bus is free,already protect by the semaphore at DEV layer */
	/* The bus should not be any of the three below statuses */
	while( I2C_STAT_IDLE != i2c_get_irq_status(i2c_data->base_addr) &&
	       I2C_STAT_BUS_ERR != i2c_get_irq_status(i2c_data->base_addr) &&
	       I2C_STAT_ARBLOST_SLAR_ACK != i2c_get_irq_status(i2c_data->base_addr)) {
		i2c_dbg("[I2C-%d]: bus is busy, status = %x\n", i2c_data->bus_num, i2c_get_irq_status(i2c_data->base_addr));
		/* 'i2c_send_clk_9pulse' is used to set i2c bus into STATE_IDLE manually */
		if (AWXX_I2C_OK ==
			  i2c_send_clk_9pulse(i2c_data->base_addr, i2c_data->bus_num)) {
			break;
		}
		/* if it is not proper, goto i2c_sunxi_xfer gor retry */
		ret = AWXX_I2C_RETRY;
		goto exit;
	}

	/* lock the critical code */
	spin_lock_irq(&i2c_data->lock);
	/* update the i2c_data for slave device msgs transfer/receive */
	i2c_data->msg     = msgs; /* msg reference */
	i2c_data->msg_num = num;  /* total number of msgs */
	i2c_data->msg_ptr = 0;    /* initialize the pointer of specific msg to write/read byte */
	i2c_data->msg_idx = 0;	  /* initialize the number of msg to start write/read */
	i2c_data->status  = I2C_XFER_START; /* Update the status */
	i2c_enable_irq(i2c_data->base_addr);  /* enable irq */
	i2c_disable_ack(i2c_data->base_addr); /* disabe ACK */
	i2c_set_EFR(i2c_data->base_addr, 0);  /* set the special function register,default:0. */
	/* unlock the critical code */
	spin_unlock_irq(&i2c_data->lock);

	/* START signal,needn't clear int flag  */
	ret = i2c_start(i2c_data->base_addr);
	/* if it is fail, then goto i2c_sunxi_xfer to retry */
	if(ret == AWXX_I2C_FAIL) {
		/* reset the i2c controller */
		i2c_soft_reset(i2c_data->base_addr);
		/* disable irq */
		i2c_disable_irq(i2c_data->base_addr);
		/* set the status to IDLE */
		i2c_data->status  = I2C_XFER_IDLE;
		ret = AWXX_I2C_RETRY;
		goto exit;
	}
	/* if it starts succesfullu, then update status with RUNNING */
	i2c_data->status  = I2C_XFER_RUNNING;

	/* sleep and wait, do the transfer at interrupt handler,
	 * timeout = 5*HZ (HZ=100 ticks/sec).
	 * The 'wake_up' call in 'i2c_sunxi_xfer_complete' wakeup the work event,
	 *  when second arg of below function (condition) satisfies.
	 */
	timeout = wait_event_timeout(i2c_data->wait, i2c_data->msg_num == 0, i2c_data->adap.timeout);
	/* return code,if(msg_idx == num) succeed,
	 * i.e number of messages tx/rx success */
	ret = i2c_data->msg_idx;

	/* Checkthe timeout error */
	if (timeout == 0){
		i2c_dbg("[I2C-%d]: xfer timeout\n", i2c_data->bus_num);
		ret = -ETIME;
	}

exit:
	return ret;
}

/*******************{ STEP-6 }*****************************/

/*
 * This function is responsible to start the i2c communication,
 * and for data transfer and receive.
 *
 * i2c_sunxi_xfer function which is called by other i2c slave device 
 * drivers. It is possible by registering the i2cslave device with 
 * specific i2c controller bus. Then the bus adapter(i2c controller)
 * information is shared to client(slave device). 
 *
 * From the adapter information, client get the algoritm structure.
 * From algorithm structure it will get all properties of i2c and
 * its i2c xfer(i2c trans/recieve) function.
 */
static int i2c_sunxi_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	/* Get the i2c_data from algo_data */
	struct sunxi_i2c *i2c_data = (struct sunxi_i2c *)adap->algo_data;
	int ret = AWXX_I2C_FAIL;
	int i   = 0;

	/* i2c_sunxi_do_xfer is the core function for start i2c comm.
	 * Here it tries only retries times(2 times) to call the core
	 * function.
	 */
	for(i = adap->retries; i >= 0; i--) {
		/* calling core function */
		ret = i2c_sunxi_do_xfer(i2c_data, msgs, num);
		/* if core function works properly, then it will 
		 * return to called func (slave driver) */
		if(ret != AWXX_I2C_RETRY) {
			return ret;
		}

		i2c_dbg("[I2C-%d]: Retrying transmission %d\n",i2c_data->bus_num, i);
		/* For every retry, i2c needs 100us */
		udelay(100);
	}

	ret = -EREMOTEIO;

	return ret;
}

/* Enables/Give support for I2C functionalities */
static unsigned int i2c_sunxi_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C|I2C_FUNC_10BIT_ADDR|I2C_FUNC_SMBUS_EMUL;
}

/* I2C algorithm structure which have i2c transfer function,
 * and different i2c functionalities, like 10-bit addr, SMBus */
static const struct i2c_algorithm i2c_sunxi_algorithm = {
        .master_xfer      = i2c_sunxi_xfer,
        .functionality    = i2c_sunxi_functionality,
};
/*******************{ STEP-6 }*****************************/

/**********************{ Algorithm }******************************/


/***************{ IRQ Handler }*********************/

/*
*         7 bits addr: 7-bits addr+0 bit r/w.
*         10bits addr: 1111_11xx_xxxx_xxxx --> 1111_0xx0 rw,xxxx_xxxx
*         send the 7 bits addr,or the first part of 10 bits addr
*/
static void i2c_sunxi_addr_byte(struct sunxi_i2c *i2c_data)
{
	unsigned char addr = 0;
	unsigned char tmp  = 0;

	if(i2c_data->msg[i2c_data->msg_idx].flags & I2C_M_TEN) {
		/* 0111_10xx,ten bits address--9:8bits */
		tmp = 0x78 | (((i2c_data->msg[i2c_data->msg_idx].addr) >> 8 ) & 0x03);
		addr = tmp << 1; /* 1111_0xx0 */
		//how about the second part of ten bits addr???
		//Answer: deal at i2c_sunxi_core_process()
	}
	else {
		/* 7-1bits addr,xxxx_xxx0 */
		addr = (i2c_data->msg[i2c_data->msg_idx].addr & 0x7f) << 1;
			i2c_dbg("[I2C-%d]: i2c slave addr: [0x%x]\n", i2c_data->bus_num, addr);
	}
	/* for read bit, default value is write */
	if (i2c_data->msg[i2c_data->msg_idx].flags & I2C_M_RD) {
		addr |= 1;
	}

	/* send 7bits+r/w or the first part of 10bits */
	i2c_put_byte(i2c_data->base_addr, &addr);

	return;
}

/* Function responsponsible for core operation of interrupt for each state.
 * Depending up on state it will send data, or receive data, write addr
 * into addr register, or send ACK and NACK(non ack).
 */
static int i2c_sunxi_core_process(struct sunxi_i2c *i2c_data)
{
	void *base_addr = i2c_data->base_addr;
	int  ret        = AWXX_I2C_OK;
	int  err_code   = 0;
	unsigned char  state = 0;
	unsigned char  tmp   = 0;

	/* It reads the status from i2c status reg */
	state = i2c_get_irq_status(base_addr);

	/* Debug msg to debug the device addr and current status */
	i2c_dbg("[I2C-%d]: I2C SLAVE ADDR:[0x%x], I2C STATE : [0x%x]\n", \
			i2c_data->bus_num, i2c_data->msg->addr, state);

	/* verifying the msg buffer really has the valid information or not,
	 * which is provided by programmer.
	 * if not exit from core operation 
	 */
	if (i2c_data->msg == NULL) {
		err_code = 0xfe;
		printk("[I2C-%d]: Invalid msg buffer: [0x%x]\n",
					 i2c_data->bus_num, err_code);
		goto exit_msg;
	}

	/* Code for every valid i2c state */
	switch (state) {

	/* On reset or stop, the bus is idle, use only at poll method */
	case 0xf8:
		err_code = 0xf8;
		goto exit_err;

	/* A START condition has been transmitted */
	case 0x08:
	/* A repeated start condition has been transmitted */
	case 0x10:
		/* send slave address */
		i2c_sunxi_addr_byte(i2c_data);
		break;

	/* SLA+W has been transmitted; NACK has been received */
	case 0x20:
	/* Second addr has been transmitted, ACK has not been received! */
	case 0xd8:
		err_code = 0x20;
		goto exit_err;

	/* SLA+W has been transmitted; ACK has been received */
	case 0x18:
		/* Check for i2c slave device having 10-bit support, 
		 * and send second part of 10-bit addr */
		if (i2c_data->msg[i2c_data->msg_idx].flags & I2C_M_TEN) {
			/* The remaining 8 bits of address */
			tmp = i2c_data->msg[i2c_data->msg_idx].addr & 0xff;
			/* After put the remain 8-bit addr into data reg of I2C
			 * 0xd0 status will be generated by I2C controller
			 */
			i2c_put_byte(base_addr, &tmp);
			break;	
			}

		/* For 7 bit addr, i2c directly send data byte i.e execute case 0x28  */


	/* Second part addr has been transmitted, ACK was received!     */
	case 0xd0:
	/* Data byte in DATA REG has been transmitted; ACK has been received */
	case 0x28:
		i2c_dbg("[I2C-%d]: tot-msgs:%d, msg-%d_len:%d, msg-%d[%d]:0x%x\n",
			 i2c_data->bus_num,  i2c_data->msg_num, i2c_data->msg_idx, 
			 i2c_data->msg[i2c_data->msg_idx].len, i2c_data->msg_idx,
			 i2c_data->msg_ptr, i2c_data->msg[i2c_data->msg_idx].buf[i2c_data->msg_ptr]);
		
		/* after send register address then START send write data  */
		if (i2c_data->msg_ptr < i2c_data->msg[i2c_data->msg_idx].len) {
			i2c_put_byte(base_addr, &(i2c_data->msg[i2c_data->msg_idx].buf[i2c_data->msg_ptr]));
			i2c_data->msg_ptr++;
			break;
		}

		/* move to next msg */
		i2c_data->msg_idx++; 
		/* Set the start byte index of i2c msg */
		i2c_data->msg_ptr = 0;
		/* validation check for num of msgs */
		if (i2c_data->msg_idx == i2c_data->msg_num) {
			/* Successfully sent the msg */
			err_code = AWXX_I2C_OK;
			goto exit_ok;
		}

		/* FIXME: Checking is needed */
		/* Case for restart to send next msg */
		else if (i2c_data->msg_idx < i2c_data->msg_num) {
			/* Restating the I2C for next msg */
			ret = i2c_restart(base_addr);
			if (ret == AWXX_I2C_FAIL) {
				err_code = AWXX_I2C_SFAIL;
				goto exit_err;
			}
		}
		/* Case for msg_idx > msg_num */
		else {
			err_code = AWXX_I2C_FAIL;
			goto exit_err;
		}
		break;

	/* Data byte in I2CDAT has been transmitted, NACK has been received */
	case 0x30:
		/* err, wakeup the thread */
		err_code = 0x30;
		goto exit_err;

	/* Arbitration(bus holding) lost during SLA+W, SLA+R or data bytes */
	case 0x38:
		/* err,wakeup the thread */
		err_code = 0x38;
		goto exit_err;

	/* SLA+R has been transmitted; ACK has been received */
	case 0x40:
		/* with Restart,needn't to send second part of 10 bits addr, refer-"I2C-SPEC v2.1" */
		/* enable A_ACK need it(receive data len) more than 1. */
		if(i2c_data->msg[i2c_data->msg_idx].len > 1) {
			/* send register addr complete,then enable the A_ACK and get ready for receiving data */
			/* ACK is tx'd when next byte is required */
			i2c_enable_ack(base_addr);
			i2c_clear_irq_flag(base_addr);/* jump to case 0x50 */
		}
		else if(i2c_data->msg[i2c_data->msg_idx].len == 1) {
			i2c_clear_irq_flag(base_addr);/* jump to case 0x58 */
		}
		break;

	/* SLA+R has been transmitted; NACK has been received */
	case 0x48:
		err_code = 0x48;//err,wakeup the thread
		goto exit_err;

	/* Data bytes has been received; ACK has been transmitted */
	case 0x50:
		/* Debug print */
		i2c_dbg("[I2C-%d]: tot-msgs:%d, msg-%d_len:%d, msg-%d[%d]:0x%x\n",
			 i2c_data->bus_num,  i2c_data->msg_num, i2c_data->msg_idx, 
			 i2c_data->msg[i2c_data->msg_idx].len, i2c_data->msg_idx,
			 i2c_data->msg_ptr, i2c_data->msg[i2c_data->msg_idx].buf[i2c_data->msg_ptr]);

		/* receive first data byte */
		if (i2c_data->msg_ptr < i2c_data->msg[i2c_data->msg_idx].len) {
			/* more than 2 bytes, the last byte need not to send ACK */
			if( (i2c_data->msg_ptr + 2) == i2c_data->msg[i2c_data->msg_idx].len ) {
				i2c_disable_ack(base_addr);/* last byte no ACK */
			}
			/* get data then clear flag, for next comming data */
			i2c_get_byte(base_addr, &i2c_data->msg[i2c_data->msg_idx].buf[i2c_data->msg_ptr]);
			i2c_data->msg_ptr++;

			break;
		}

		/* err process, the last byte should be @case 0x58 */
		err_code = AWXX_I2C_FAIL;/* err, wakeup */
		goto exit_err;

	/* Data byte has been received; NACK has been transmitted for last byte */
	case 0x58: 
		/* Debug print */
		i2c_dbg("[I2C-%d]: tot-msgs:%d, msg-%d_len:%d, msg-%d[%d]:0x%x\n",
			 i2c_data->bus_num,  i2c_data->msg_num, i2c_data->msg_idx, 
			 i2c_data->msg[i2c_data->msg_idx].len, i2c_data->msg_idx,
			 i2c_data->msg_ptr, i2c_data->msg[i2c_data->msg_idx].buf[i2c_data->msg_ptr]);

		/* received the last byte  */
		if ( i2c_data->msg_ptr == i2c_data->msg[i2c_data->msg_idx].len - 1 ) {
			/* get the last byte */
			i2c_get_last_byte(base_addr, &i2c_data->msg[i2c_data->msg_idx].buf[i2c_data->msg_ptr]);
			/* move tonext msg */
			i2c_data->msg_idx++;
			/* initialize msg_ptr to first pos */
			i2c_data->msg_ptr = 0;
			if (i2c_data->msg_idx == i2c_data->msg_num) {
				err_code = AWXX_I2C_OK; // succeed,wakeup the thread
				goto exit_ok;
			}
			/* Restart the i2c for next msg */
			else if(i2c_data->msg_idx < i2c_data->msg_num) {
				/* repeat start */
				ret = i2c_restart(base_addr);
				if(ret == AWXX_I2C_FAIL) {/* START fail */
					err_code = AWXX_I2C_SFAIL;
					goto exit_err;
				}
				break;
			}
		}
		else {
			/* if it's not last byte display error */
			err_code = 0x58;
			goto exit_err;
		}

	/* Bus error during master or slave mode, due to illegal level condition */
	case 0x00:
		err_code = 0xff;
		goto exit_err;

	default:
		err_code = state;
		goto exit_err;
	}

	/* Debug state is for just debugging */
	i2c_data->debug_state = state;

	return ret;

exit_ok:
exit_err:
	if(AWXX_I2C_FAIL == i2c_stop(base_addr))
		i2c_dbg("[I2C-%d]: STOP failed!\n", i2c_data->bus_num);

exit_msg:
	/* Intemate the work_queue, that operation has completed.
	 * That work event will WAKE_UP after processor releases the IRQ.
	 */
	ret = i2c_sunxi_xfer_complete(i2c_data, err_code);
	/* Debug state is for just debugging */
	i2c_data->debug_state = state;

	return ret;
}


/*******************{ STEP-5 }*****************************/

/* Irq handler */
static irqreturn_t i2c_sun7i_handler(int irq, void *data)
{
	/* Getting i2c_data */
	struct sunxi_i2c *i2c_data = (struct sunxi_i2c *)data;
	int ret = AWXX_I2C_FAIL;

	/* Get the CTL reg int_flag bit, 
	 * and should be one for all states except 0xf8.
	 */
	if(!i2c_get_irq_flag(i2c_data->base_addr)) {
		pr_warning("[I2C-%d]: unknown interrupt!", i2c_data->bus_num);
		return ret;
	}

	/* Disable the irq to prevent another interrupt on same line */
	i2c_disable_irq(i2c_data->base_addr);

	/* Start i2c core operation(addr, r/w) */
	ret = i2c_sunxi_core_process(i2c_data);

	/* Enable irq only when i2c is transfering, otherwise,disable irq */
	if(i2c_data->status != I2C_XFER_IDLE) {
		i2c_enable_irq(i2c_data->base_addr);
	}

	return IRQ_HANDLED;
}

/*******************{ STEP-5 }*****************************/

/****************************{ IRQ Handler }*****************************/


/*
* Fin is APB CLOCK INPUT;
* Fsample = F0 = Fin/2^CLK_N;
* F1 = F0/(CLK_M+1);
* Foscl = F1/10 = Fin/(2^CLK_N * (CLK_M+1)*10);
* Foscl is clock SCL;100KHz or 400KHz
*
* clk_in: apb clk clock
* sclk_req: freqence to set in HZ
*/

static void i2c_set_speed(struct sunxi_i2c *i2c_data)
{
	
/* 
 * FIXME:	Logic Only for 400K, 200K, and 100K speeds,
 *		But not for all speeds,
 * TODO:	Generalize the logic for all speeds.
 */
	unsigned int i2c_bus_speed = i2c_data->bus_freq;
	/* Get the input clock rate fot i2c contoller */	
	unsigned int f_in = clk_get_rate(i2c_data->clk); /* 24M Hz */
	int clk_m = 0, clk_n = 0;

/*
TODO: Generalized code, But not tested
for(clk_n=0;clk_n<8;clk_n++)
{
	clk_m = f_in/(10*i2c_bus_speed*(0x01<<clk_n))-1;
	for(;clk_m<16;clk_m++)// 4 bits for clk_m 
	{
		unsigned int f_req = 0;
		f_req=f_in/(10*(1<<clk_n)*(clk_m));//recalculating required freq.
		if(f_req <= i2c_bus_speed)
		goto wr_speed;
	}
}
wr_speed:
*/

	/* Logic for 400K, 200K, and 100K speeds */
	switch(i2c_bus_speed) {
	case I2C_TRANSFER_SPEED_400K:
		clk_m = 2;
		clk_n = 1;
		break;
	case I2C_TRANSFER_SPEED_200K:
		clk_m = 2;
		clk_n = 2;
		break;
	case I2C_TRANSFER_SPEED_100K:
		clk_m = 3;
		clk_n = 2;
		break;
	default:
		pr_info("[I2C-%d]: Error: I2C bus speed shouldn't be other than"
			" 400K or 200K or 100K\n", i2c_data->bus_num);
		break;	
	}
	
	i2c_clock_set(i2c_data->base_addr, clk_m, clk_n);
	
	pr_info("[I2C-%d]: Set the speed to: %u\n",i2c_data->bus_num, f_in/(10*(1<<clk_n)*(clk_m+1))); 
}

/***********{ H/W INIT ROUTINES }***********************/

static void i2c_sunxi_hw_init(struct sunxi_i2c *i2c_data)
{

/* FIXME: */
/***** GPIO PINMUX for I2C *****/
/* This is the place where pinmux can be done manually */	
#if 0

switch(i2c_data->bus_num)
{
case 0:
	/* pb0-pb1 I2C0 SDA,SCK */
	i2c_dbg("config i2c gpio with gpio_config api \n");
	
case 1:	

case 2:

}
#endif	

	/* Enable the i2c bus */
	i2c_bus_enable(i2c_data->base_addr);
	/* Set speed of I2C */
	i2c_set_speed(i2c_data);
	/* Resetting the I2C contoller */
	i2c_soft_reset(i2c_data->base_addr);

}

static void i2c_sunxi_hw_exit(struct sunxi_i2c *i2c_data)
{
	/* disable the i2c bus */
	i2c_bus_disable(i2c_data->base_addr);
}
/***********{ H/W INIT ROUTINES }***********************/

/*****************{ OPTIONAL }**************************/

/* As mentioned in probe function it is used only for executing this
 * driver as a kernel module. For In-tree driver it's not required.
 */
#ifdef DRIVER_OUTOF_TREE
int pin_ctrl_bind_pins(struct device *dev)
{
        int ret;

        dev->pins = devm_kzalloc(dev, sizeof(*(dev->pins)), GFP_KERNEL);
        if (!dev->pins)
                return -ENOMEM;

        dev->pins->p = devm_pinctrl_get(dev);
        if (IS_ERR(dev->pins->p)) {
                dev_dbg(dev, "no pinctrl handle\n");
                ret = PTR_ERR(dev->pins->p);
                goto cleanup_alloc;
        }
        
        dev->pins->default_state = pinctrl_lookup_state(dev->pins->p,
                                        PINCTRL_STATE_DEFAULT);
        if (IS_ERR(dev->pins->default_state)) {
                dev_dbg(dev, "no default pinctrl state\n");
                ret = 0;
                goto cleanup_get;
        }

        ret = pinctrl_select_state(dev->pins->p, dev->pins->default_state);
        if (ret) {
                dev_dbg(dev, "failed to activate default pinctrl state\n");
                goto cleanup_get;
        }

#ifdef CONFIG_PM
        /*
         * If power management is enabled, we also look for the optional
         * sleep and idle pin states, with semantics as defined in
         * <linux/pinctrl/pinctrl-state.h>
         */
        dev->pins->sleep_state = pinctrl_lookup_state(dev->pins->p,
                                        PINCTRL_STATE_SLEEP);
        if (IS_ERR(dev->pins->sleep_state))
                /* Not supplying this state is perfectly legal */
                dev_dbg(dev, "no sleep pinctrl state\n");

        dev->pins->idle_state = pinctrl_lookup_state(dev->pins->p,
                                        PINCTRL_STATE_IDLE);
        if (IS_ERR(dev->pins->idle_state))
                /* Not supplying this state is perfectly legal */
                dev_dbg(dev, "no idle pinctrl state\n");
#endif

        return 0;

        /*
         * If no pinctrl handle or default state was found for this device,
         * let's explicitly free the pin container in the device, there is
         * no point in keeping it around.
         */
cleanup_get:
        devm_pinctrl_put(dev->pins->p);
cleanup_alloc:
        devm_kfree(dev, dev->pins);
        dev->pins = NULL;

        /* Only return deferrals */
        if (ret != -EPROBE_DEFER)
                ret = 0;

        return ret;
}
#endif
/*****************{ OPTIONAL }**************************/

/*******************{ STEP-4 }*****************************/

/* Probe routine *******************************************/
static int sun7i_i2c_probe(struct platform_device *pd)
{

	struct sunxi_i2c *i2c_data = NULL;
	struct resource *r = NULL;
	struct sunxi_i2c_platform_data *pdata = dev_get_platdata(&pd->dev);
	int ret;
	int irq;

	pr_info("[I2C-%d]: Probe started...", pd->id);
	/* Platform data validation */
	if(pdata == NULL) {
		pr_info("[I2C-%d]: %s: Platform data Error\n", pd->id, __func__);
		return -ENODEV;
	}
	/* creat dynamic memory for i2c_data(private data) */
	i2c_data = devm_kzalloc(&pd->dev, sizeof(struct sunxi_i2c), GFP_KERNEL);
	if (!i2c_data) {
		pr_info("[I2C-%d] %s: memory not allocated for private structure\n",
				 pd->id, __func__);
		return -ENOMEM;
	}

	/* Get the PA resource from platform device */
	r = platform_get_resource(pd, IORESOURCE_MEM, 0);
	/* Get the IRQ resource from platform device */ 
	irq = platform_get_irq(pd, 0);
	/* Validaion for both IRQ and PA resources */
	if (r == NULL || irq < 0) {
		pr_info("[I2C-%d]: Error in getting resource\n", pd->id);
		return -ENODEV;
	}

	i2c_data->iobase = r->start;
	i2c_data->iosize = resource_size(r);

	/* Check the memory region and remap the memory of size mentioned
	 *	in IORESOURCE_MEM resource.
	 */
	i2c_data->base_addr = devm_ioremap_resource(&pd->dev, r);
	/* Validation for remapped base addr of io resource */
        if (IS_ERR(i2c_data->base_addr)) {
		pr_info("[I2C-%d]: Error in ioremap\n", pd->id);
                return PTR_ERR(i2c_data->base_addr);
	}

/************{ clk and pinmux hacking }****************/
/* Getting the clock from device tree */
{
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	char *node_path[] = {
		"/soc@01c00000/i2c@01c2ac00",
		"/soc@01c00000/i2c@01c2b000",
		"/soc@01c00000/i2c@01c2b400",
		"/soc@01c00000/i2c@01c2b800",
		"/soc@01c00000/i2c@01c2c000",
	};

	/* FInd the DT node of respctive i2c controller using DT PATH */
	np = of_find_node_opts_by_path(node_path[pd->id], NULL);
	/* After getting the np release the list */
	if(np)
		of_node_put(np);

	/* Find the platform device instance reference from np */
	pdev = of_find_device_by_node(np);
	/* After getting the pdev release the list */
	if(pdev)
		of_node_put(np);

	/* Initialize device instance name with pdev->name */
	pdev->dev.init_name = pdev->name;
	/* Get the i2c clock reference from DT pdev */
	i2c_data->clk = devm_clk_get(&pdev->dev, NULL);
	/* Validation for i2c clock */
	if (!IS_ERR(i2c_data->clk)) {
		pr_info("[I2C-%d]: CLK enable ok\n", pd->id);
		/* Prepare the clock */
		clk_prepare(i2c_data->clk);
		/* Enable the clock */
		clk_enable(i2c_data->clk);
	}

/* The below code initializes the pinmux of relative I2C adap */
#ifdef DRIVER_OUTOF_TREE
	/* The below function is used when the driver is used as
	 * kernel module, which do the pinmux for I2C through DT pdev.
	 */
	pin_ctrl_bind_pins(&pdev->dev);
#else
	/* The below function is used when the driver is part of
	 * kernel, which do the pinmux for I2C through DT pdev.
	 */
	pinctrl_bind_pins(&pdev->dev);
#endif
}
/************{ clk and pinmux hacking }****************/


	/* I2C structure initialization */
	i2c_data->bus_freq	= pdata->frequency;
	i2c_data->irq		= irq;
	i2c_data->bus_num	= pdata->bus_num;
	i2c_data->status	= I2C_XFER_IDLE;
	i2c_data->suspend_flag	= 0;

	/* I2C adapter initialization */ 
	strlcpy(i2c_data->adap.name, "sun7i-i2c", sizeof(i2c_data->adap.name));
	snprintf(i2c_data->adap.name, sizeof(i2c_data->adap.name),
			"sun7i-i2c.%u", i2c_data->adap.nr);
	i2c_data->adap.owner   = THIS_MODULE;
	i2c_data->adap.nr      = pdata->bus_num;
	i2c_data->adap.retries = 2;
	i2c_data->adap.timeout = 5*HZ;/* HZ = 100 */
	i2c_data->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c_data->adap.dev.parent = &pd->dev;


	/* I2C ALGORITHM initialization */
	i2c_data->adap.algo = &i2c_sunxi_algorithm;
	i2c_data->adap.algo_data  = i2c_data;

	/* Initilize the spinlock */
	spin_lock_init(&i2c_data->lock);
	/* initialize wait_queue */
	init_waitqueue_head(&i2c_data->wait);
	/* Initialize the h/w bus and speed */
	i2c_sunxi_hw_init(i2c_data);

	/* Create a new I2C bus with num specified in adap.nr feild */ 
	ret = i2c_add_numbered_adapter(&i2c_data->adap);
	if (ret < 0) {
		i2c_dbg("[I2C-%d]: Failed to add bus\n", pd->id);
		goto exit_adapt;
	}

	/* Set the i2c_data to platform driver driverdata member */
	platform_set_drvdata(pd, i2c_data);

	/* Request the irq */
	ret = request_irq(irq, i2c_sun7i_handler, IRQF_TRIGGER_NONE, i2c_data->adap.name,i2c_data);
	if (ret) {
		pr_info("[I2C-%d]: Error in IRQ Request\n", pd->id);
		goto exit_irq;
	}

	pr_info("[I2C-%d]: %s: adapter Registerd Successfully.\n", pd->id, dev_name(&i2c_data->adap.dev));


	return 0;

exit_irq:
	i2c_del_adapter(&i2c_data->adap);

exit_adapt:
	if (!IS_ERR(i2c_data->clk)) {
		clk_disable(i2c_data->clk);
		clk_unprepare(i2c_data->clk);
	}

	return ret;
}

/* Remove routine *******************************************/
static int sun7i_i2c_remove(struct platform_device *pd)
{
	struct sunxi_i2c *i2c_data = platform_get_drvdata(pd);
	pr_info("[I2C-%d]: Remove starting...\n", pd->id);

	/* Nullyfy the platform driver data */
	platform_set_drvdata(pd, NULL);

	/* Unregister I2C Adapter */
	i2c_del_adapter(&i2c_data->adap);

	/* Free the IRQ line */
	free_irq(i2c_data->irq, i2c_data);

	/* disable h/w bus */
	i2c_sunxi_hw_exit(i2c_data); 

	/* Disable the clock */
	if (!IS_ERR(i2c_data->clk)) {
		clk_disable(i2c_data->clk);
		clk_unprepare(i2c_data->clk);
	}
	
	return 0;
}
/*******************{ STEP-4 }*****************************/

/*******************{ STEP-2 }*****************************/
/* sun7i i2c adapters common platform driver information */

/*
TODO: DT information

static const struct of_device_id mv64xxx_i2c_of_match_table[] = {
        { .compatible = "allwinner,sun7i-a20-i2c",},
        {}
};
MODULE_DEVICE_TABLE(of, mv64xxx_i2c_of_match_table);
*/

/* Platform Driver ****************************************/
static struct platform_driver sun7i_i2c_driver = {
	.probe  = sun7i_i2c_probe,
	.remove = sun7i_i2c_remove,
	.driver = {
		.name   = SUN7I_I2C_CTLR_NAME,
/* TODO		.of_match_table = mv64xxx_i2c_of_match_table,*/
	},
};
/* TODO: module_platform_driver(sun7i_i2c_driver); */

/*******************{ STEP-2 }*****************************/

/*******************{ STEP-1 }*****************************/
/* sun7i i2c adapters platform device information */

/* Resource Information ***********************************/
static struct resource sunxi_i2c0_resources[] = {
	{
		.start  = I2C0_ADAP_PA,
		.end    = I2C0_ADAP_PA + I2C_MEM_SIZE,
		.flags  = IORESOURCE_MEM,
	}, {
		.start  = I2C0_IRQ_NUM,
		.flags  = IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	},
};
static struct resource sunxi_i2c1_resources[] = {
	{
		.start  = I2C1_ADAP_PA,
		.end    = I2C1_ADAP_PA + I2C_MEM_SIZE,
		.flags  = IORESOURCE_MEM,
	}, {
		.start  = I2C1_IRQ_NUM,
		.flags  = IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	},
};

/* Platform Data ******************************************/

static struct sunxi_i2c_platform_data sunxi_i2c0_pdata[] = {
	{
		.bus_num   = 0,
		.frequency = I2C_TRANSFER_SPEED_400K,
	},
};
static struct sunxi_i2c_platform_data sunxi_i2c1_pdata[] = {
	{
		.bus_num   = 1,
		.frequency = I2C_TRANSFER_SPEED_400K,
	},
};

/* Platofrm Device ****************************************/

static struct platform_device sun7i_i2c_device[] = {
	[0] = {
		.name           = "sun7i-i2c",
		.id                 = 0,
		.resource       = sunxi_i2c0_resources,
		.num_resources  = ARRAY_SIZE(sunxi_i2c0_resources),
		.dev = {
			.platform_data = sunxi_i2c0_pdata,
		},
	},
	[1] = {
		.name           = "sun7i-i2c",
		.id                 = 1,
		.resource       = sunxi_i2c1_resources,
		.num_resources  = ARRAY_SIZE(sunxi_i2c1_resources),
		.dev = {
			.platform_data = sunxi_i2c1_pdata,
		},
	},
};
/*******************{ STEP-1 }*****************************/

/*******************{ STEP-3 }*****************************/

/* Init routine  ******************************************/
static int i2c_adap_sun7i_init(void)
{
	int chips;

	/* Registering all adapters as platform devices */
	for (chips = 0; chips < NUM_I2C_CHIPS; chips++)
		platform_device_register(&sun7i_i2c_device[chips]);

	/* Registering platform driver for all adapters */
	platform_driver_register(&sun7i_i2c_driver);

	return 0;
}
module_init(i2c_adap_sun7i_init);

/* Exit routine *******************************************/
static void i2c_adap_sun7i_exit(void)
{
	int chips;

	/* Unregistering platform driver for all adapters */
	platform_driver_unregister(&sun7i_i2c_driver);

	/* Unregistering all adapters as platform devices */
	for (chips = 0; chips < NUM_I2C_CHIPS; chips++)
		platform_device_unregister(&sun7i_i2c_device[chips]);

}
module_exit(i2c_adap_sun7i_exit);
/*******************{ STEP-3 }*****************************/

MODULE_LICENSE("GPL");
