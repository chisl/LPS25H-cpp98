/*
 * name:        LPS25H
 * description: 260-1260 hPa absolute digital output barometer
 * manuf:       STMicroelectronics
 * version:     Version 0.1
 * url:         http://www.st.com/resource/en/datasheet/lps25h.pdf
 * date:        2018-01-04
 * author       https://chisl.io/
 * file:        LPS25H.hpp
 */

/*                                                                                                       *
 *                                   THIS FILE IS AUTOMATICALLY CREATED                                  *
 *                                    D O     N O T     M O D I F Y  !                                   *
 *                                                                                                       */

#include <cinttypes>

/* Derive from class LPS25H_Base and implement the read and write functions! */

/* LPS25H: 260-1260 hPa absolute digital output barometer */
class LPS25H_Base
{
public:
	/* Pure virtual functions that need to be implemented in derived class: */
	virtual uint8_t read8(uint16_t address, uint16_t n=8) = 0;  // 8 bit read
	virtual void write(uint16_t address, uint8_t value, uint16_t n=8) = 0;  // 8 bit write
	virtual uint16_t read16(uint16_t address, uint16_t n=16) = 0;  // 16 bit read
	virtual void write(uint16_t address, uint16_t value, uint16_t n=16) = 0;  // 16 bit write
	virtual uint32_t read32(uint16_t address, uint16_t n=32) = 0;  // 32 bit read
	virtual void write(uint16_t address, uint32_t value, uint16_t n=32) = 0;  // 32 bit write
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG REF_P                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG REF_P:
	 * 7.1-3
	 * Reference pressure
	 * The full reference pressure value is composed
	 * by REF_P_XL, REF_P_H & REF_P_L and is represented as 2’s complement. The
	 * reference pressure value can also be used to detect a measured pressure beyond
	 * programmed limits (see INT_CFD at 23h), and for Autozero function (see RESET_AZ
	 * bit, at 20h).
	 */
	struct REF_P
	{
		static const uint16_t __address = 8;
		
		/* Bits REF_P: */
		struct REF_P_
		{
			/* MODE rw */
			static const uint32_t dflt = 0b000000000000000000000000; // 24'd0
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register REF_P */
	void setREF_P(uint32_t value)
	{
		write(REF_P::__address, value, 24);
	}
	
	/* Get register REF_P */
	uint32_t getREF_P()
	{
		return read32(REF_P::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG WHO_AM_I                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG WHO_AM_I:
	 * 7.4
	 * Contains the device ID, BDh
	 */
	struct WHO_AM_I
	{
		static const uint16_t __address = 15;
		
		/* Bits WHO_AM_I: */
		struct WHO_AM_I_
		{
			/* MODE r */
			static const uint8_t dflt = 0b10111101; // 8'b10111101
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register WHO_AM_I */
	void setWHO_AM_I(uint8_t value)
	{
		write(WHO_AM_I::__address, value, 8);
	}
	
	/* Get register WHO_AM_I */
	uint8_t getWHO_AM_I()
	{
		return read8(WHO_AM_I::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG RES_CONF                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RES_CONF:
	 * 7.5
	 * Pressure and temperature internal average configuration.
	 */
	struct RES_CONF
	{
		static const uint16_t __address = 16;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0000; // 4'b0
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits AVGT: */
		/* select the pressure internal average.  */
		struct AVGT
		{
			/* MODE rw */
			static const uint8_t dflt = 0b01; // 2'b1
			static const uint8_t mask = 0b00001100; // [2,3]
			static const uint8_t INTERNAL_AVG_8 = 0b00; // 8 Nr. internal average
			static const uint8_t INTERNAL_AVG_16 = 0b01; // 16 Nr. internal average
			static const uint8_t INTERNAL_AVG_32 = 0b10; // 32 Nr. internal average
			static const uint8_t INTERNAL_AVG_64 = 0b11; // 64 Nr. internal average
		};
		/* Bits AVGP: */
		/* select the temperature internal average.  */
		struct AVGP
		{
			/* MODE rw */
			static const uint8_t dflt = 0b01; // 2'b1
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t INTERNAL_AVG_8 = 0b00; // 8 Nr. internal average
			static const uint8_t INTERNAL_AVG_32 = 0b01; // 32 Nr. internal average
			static const uint8_t INTERNAL_AVG_128 = 0b10; // 128 Nr. internal average
			static const uint8_t INTERNAL_AVG_512 = 0b11; // 512 Nr. internal average
		};
	};
	
	/* Set register RES_CONF */
	void setRES_CONF(uint8_t value)
	{
		write(RES_CONF::__address, value, 8);
	}
	
	/* Get register RES_CONF */
	uint8_t getRES_CONF()
	{
		return read8(RES_CONF::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG1                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG1:
	 * 7.6
	 * Control register 1.
	 */
	struct CTRL_REG1
	{
		static const uint16_t __address = 32;
		
		/* Bits PD: */
		/*
		 * Power-down control.
		 * PD bit allows to turn on the device. The device is in power-down mode when PD = ‘0’
		 * (default value after boot). The device is active when PD is set to ‘1’.
		 */
		struct PD
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t POWER_DOWN_MODE = 0b0; // power-down mode
			static const uint8_t ACTIVE_MODE = 0b1; // active mode
		};
		/* Bits ODR: */
		/*
		 * ODR2- ODR1 - ODR0 bits allow to change the output data rates of pressure and temperature samples.
		 * The default value is “000” which corresponds to “one shot configuration” for both pressure and
		 * temperature output.
		 */
		struct ODR
		{
			/* MODE rw */
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b01110000; // [4,5,6]
			static const uint8_t ONE_SHOT = 0b00; // Power down / One shot mode enabled
			static const uint8_t F_1_HZ = 0b01; // p:  1 Hz,   T:  1 Hz
			static const uint8_t F_7_HZ = 0b10; // p:  7 Hz,   T:  7 Hz
			static const uint8_t F_12_5_HZ = 0b11; // p: 12.5 Hz, T: 12.5 Hz
			static const uint8_t F_25_HZ = 0b100; // p: 12.5 Hz, T: 12.5 Hz
			static const uint8_t reserved_0 = 0b101; // 
			static const uint8_t reserved_1 = 0b110; // 
			static const uint8_t reserved_2 = 0b111; // 
		};
		/* Bits DIFF_EN: */
		/*
		 * Interrupt circuit enable.
		 * The DIFF_EN bit is used to enable the computing of differential pressure output.
		 * It is recommended to enable DIFF_EN after the configuration of REF_P_H (0Ah),
		 * REF_P_L (09h), REF_P_XL (08h), THS_P_H (31h) and THS_P_L (30h).
		 */
		struct DIFF_EN
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // interrupt generation disabled
			static const uint8_t ENABLE = 0b1; // interrupt generation enabled
		};
		/* Bits BDU: */
		/*
		 * Block data update.
		 * BDU bit is used to inhibit the output registers update between the reading of upper and
		 * lower register parts. In default mode (BDU = ‘0’), the lower and upper register parts are
		 * updated continuously. If it is not sure to read faster than output data rate, it is
		 * recommended to set BDU bit to ‘1’. In this way, after the reading of the lower (upper)
		 * register part, the content of that output registers is not updated until the upper (lower)
		 * part is read too.
		 * This feature avoids reading LSB and MSB related to different samples.
		 */
		struct BDU
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t CONTINUOUS_UPDATE = 0b0; // continuous update;
			static const uint8_t NOT_UPDATED_UNTIL_READ = 0b1; // output registers not updated until MSB and LSB reading
		};
		/* Bits RESET_AZ: */
		/*
		 * Reset AutoZero function. Reset REF_Preg, set pressure to default value in RPDS register (@0x39/A).
		 * RESET_AZ bit is used to Reset AutoZero function. Reset REF_P reg (@0x08..0A) set pressure
		 * reference to default value RPDS reg (0x39/3A). RESET_AZ is self cleared. See AutoZero function.
		 */
		struct RESET_AZ
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t RESET = 0b1; // 
		};
		/* Bits SIM: */
		/* SPI Serial Interface Mode selection.  */
		struct SIM
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t SPI_4_WIRE = 0b0; // 4-wire interface
			static const uint8_t SPI_3_WIRE = 0b1; // 3-wire interface
		};
	};
	
	/* Set register CTRL_REG1 */
	void setCTRL_REG1(uint8_t value)
	{
		write(CTRL_REG1::__address, value, 8);
	}
	
	/* Get register CTRL_REG1 */
	uint8_t getCTRL_REG1()
	{
		return read8(CTRL_REG1::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG2                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG2:
	 * 7.7
	 * Control register 2
	 */
	struct CTRL_REG2
	{
		static const uint16_t __address = 33;
		
		/* Bits BOOT: */
		/*
		 * Reboot memory content.
		 * BOOT bit is used to refresh the content of the internal registers stored in the
		 * Flash memory block. At the device power-up the content of the Flash memory block
		 * is transferred to the internal registers related to trimming functions to permit
		 * a good behavior of the device itself. If for any reason, the content of the trimming
		 * registers is modified, it is sufficient to use this bit to restore the correct values.
		 * When BOOT bit is set to ‘1’ the content of the internal Flash is copied inside the
		 * corresponding internal registers and is used to calibrate the device. These values
		 * are factory trimmed and they are different for every device. They permit good behavior
		 * of the device and normally they should not be changed. At the end of the boot process
		 * the BOOT bit is set again to ‘0’ by hardware. BOOT bit takes effect after one ODR
		 * clock cycle.
		 */
		struct BOOT
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t REBOOT_MEMORY = 0b1; // reboot memory content
		};
		/* Bits FIFO_EN: */
		/* FIFO enable.  */
		struct FIFO_EN
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits WTM_EN: */
		/* Enable FIFO Watermark level use.  */
		struct WTM_EN
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits FIFO_MEAN_DEC: */
		/*
		 * Enable 1Hz ODR decimation
		 * 
		 */
		struct FIFO_MEAN_DEC
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
		};
		/* Bits SWRESET: */
		/*
		 * Software reset.
		 * SWRESET is the software reset bit. The device is reset to the power on configuration
		 * if the SWRESET bit is set to ‘1’ and BOOT is set to ‘1’.
		 */
		struct SWRESET
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t SOFTWARE_RESET = 0b1; // 
		};
		/* Bits AUTOZERO: */
		/*
		 * Autozero enable.
		 * AUTO_ZERO, when set to ‘1’, the actual pressure output is copied in the
		 * REF_P_H & REF_P_L & REF_P_XL and kept as reference and the
		 * PRESS_OUT_H & PRESS_OUT_L & PRESS _OUT_XL is the difference between this reference and
		 * the pressure sensor value.
		 */
		struct AUTOZERO
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t AUTOZERO_ENABLED = 0b1; // 
		};
		/* Bits ONE_SHOT: */
		/*
		 * One-shot enable.
		 * ONE_SHOT bit is used to start a new conversion when ODR2..0 bits in CTRL_REG1 are set
		 * to “000”. Write ‘1’ in ONE_SHOT to trigger a single measurement of pressure and temperature.
		 * Once the measurement is done, ONE_SHOT bit will self-clear and the new data are available
		 * in the output registers, and the STATUS_REG bits are updated.
		 */
		struct ONE_SHOT
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t WAITING = 0b0; // waiting for start of conversion
			static const uint8_t NEW_DATASET = 0b1; // astart for a new dataset
		};
	};
	
	/* Set register CTRL_REG2 */
	void setCTRL_REG2(uint8_t value)
	{
		write(CTRL_REG2::__address, value, 8);
	}
	
	/* Get register CTRL_REG2 */
	uint8_t getCTRL_REG2()
	{
		return read8(CTRL_REG2::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG3                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG3:
	 * 7.8
	 * Control register 3 - INT_DRDY pin control register
	 * The device features one fully-programmable interrupt sources (INT1) that can be configured to
	 * trigger different pressure events. Figure 12 shows the block diagram of the interrupt generation
	 * block and output pressure data.
	 * The device may also be configured to generate, through interrupt pins, a Data Ready signal (Drdy)
	 * which indicates when a new measured pressure data is available, thus simplifying data synchronization
	 * in digital systems or to optimize the system power consumption.
	 * See Fig. 12.
	 */
	struct CTRL_REG3
	{
		static const uint16_t __address = 34;
		
		/* Bits INT_H_L: */
		/* Interrupt active-high/low.  */
		struct INT_H_L
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t ACTIVE_HIGH = 0b0; // active high
			static const uint8_t ACTIVE_LOW = 0b1; // active low
		};
		/* Bits PP_OD: */
		/* Push-pull/open drain selection on interrupt pads.  */
		struct PP_OD
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t PUSH_PULL = 0b0; // push-pull
			static const uint8_t OPEN_DRAIN = 0b1; // open drain
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0000; // 4'b0
			static const uint8_t mask = 0b00111100; // [2,3,4,5]
		};
		/* Bits INT_S: */
		/* Data signal on INT1 pad control bits.  */
		struct INT_S
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t DATA_SIGNAL = 0b00; // Data signal (see CTRL_REG4)
			static const uint8_t PRESSURE_HIGH = 0b01; // Pressure high (P_high)
			static const uint8_t PRESSURE_LOW = 0b10; // Pressure low (P_low)
			static const uint8_t PRESSURE_LOW_OR_HIGH = 0b11; // Pressure low OR high
		};
	};
	
	/* Set register CTRL_REG3 */
	void setCTRL_REG3(uint8_t value)
	{
		write(CTRL_REG3::__address, value, 8);
	}
	
	/* Get register CTRL_REG3 */
	uint8_t getCTRL_REG3()
	{
		return read8(CTRL_REG3::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG4                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG4:
	 * Interrupt configuration
	 */
	struct CTRL_REG4
	{
		static const uint16_t __address = 35;
		
		/* Bits reserved_0: */
		/* keep these bits at 0  */
		struct reserved_0
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0000; // 4'b0
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits P1_EMPTY: */
		/* Empty signal on INT1 pin.  */
		struct P1_EMPTY
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits P1_WTM: */
		/* Watermark signal on INT1 pin.  */
		struct P1_WTM
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits P1_OVERRUN: */
		/* Overrun signal on INT1 pin.  */
		struct P1_OVERRUN
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits P1_DRDY: */
		/* Dataready signal on INT1 pin.  */
		struct P1_DRDY
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
	};
	
	/* Set register CTRL_REG4 */
	void setCTRL_REG4(uint8_t value)
	{
		write(CTRL_REG4::__address, value, 8);
	}
	
	/* Get register CTRL_REG4 */
	uint8_t getCTRL_REG4()
	{
		return read8(CTRL_REG4::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG INTERRUPT_CFG                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG INTERRUPT_CFG:
	 * 7.1
	 * Interrupt configuration
	 */
	struct INTERRUPT_CFG
	{
		static const uint16_t __address = 36;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00000; // 5'b0
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits LIR: */
		/* Latch interrupt request into INT_SOURCE register.  */
		struct LIR
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NOT_LATCHED = 0b0; // interrupt request not latched
			static const uint8_t LATHED = 0b1; // interrupt request latched
		};
		/* Bits PLE: */
		/* Enable interrupt generation on differential pressure low event.  */
		struct PLE
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // disable interrupt request 
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value lower than preset threshold
		};
		/* Bits PHE: */
		/* Enable interrupt generation on differential pressure high event.  */
		struct PHE
		{
			/* MODE rw */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t DISABLE = 0b0; // disable interrupt request
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value higher than preset threshold
		};
	};
	
	/* Set register INTERRUPT_CFG */
	void setINTERRUPT_CFG(uint8_t value)
	{
		write(INTERRUPT_CFG::__address, value, 8);
	}
	
	/* Get register INTERRUPT_CFG */
	uint8_t getINTERRUPT_CFG()
	{
		return read8(INTERRUPT_CFG::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG INT_SOURCE                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG INT_SOURCE:
	 * 7.15
	 * Interrupt source
	 * INT_SOURCE register is cleared by reading it.
	 */
	struct INT_SOURCE
	{
		static const uint16_t __address = 37;
		
		/* Bits reserved_0: */
		/* keep these bits at 0.  */
		struct reserved_0
		{
			/* MODE r */
			static const uint8_t dflt = 0b00000; // 5'd0
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits IA: */
		/* Interrupt active.  */
		struct IA
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // one or more interrupt events have been generated
		};
		/* Bits PL: */
		/* Differential pressure Low.  */
		struct PL
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // Low differential pressure event has occurred
		};
		/* Bits PH: */
		/* Differential pressure High.  */
		struct PH
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // High differential pressure event has occurred
		};
	};
	
	/* Set register INT_SOURCE */
	void setINT_SOURCE(uint8_t value)
	{
		write(INT_SOURCE::__address, value, 8);
	}
	
	/* Get register INT_SOURCE */
	uint8_t getINT_SOURCE()
	{
		return read8(INT_SOURCE::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG STATUS_REG                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG STATUS_REG:
	 * 7.12
	 * Status register.
	 * This register is updated every ODR cycle, regardless of the BDU value in CTRL_REG1.
	 */
	struct STATUS_REG
	{
		static const uint16_t __address = 39;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			/* MODE r */
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits P_OR: */
		/*
		 * Pressure data overrun.
		 * P_OR bit is set to '1' whenever new pressure data is available and P_DA was set in
		 * the previous ODR cycle and not cleared. P_OR is cleared when PRESS_OUT_H (2Ah)
		 * register is read.
		 */
		struct P_OR
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // new data for pressure has overwritten the previous one
		};
		/* Bits T_OR: */
		/*
		 * Temperature data overrun.
		 * T_OR is set to ‘1’ whenever new temperature data is available and T_DA was set in the previous
		 * ODR cycle and not cleared. T_OR is cleared when TEMP_OUT_H (2Ch) register is read.
		 */
		struct T_OR
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // a new data for temperature has overwritten the previous one
		};
		/* Bits unused_1: */
		struct unused_1
		{
			/* MODE r */
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00001100; // [2,3]
		};
		/* Bits P_DA: */
		/*
		 * Pressure data available.
		 * P_DA is set to 1 whenever a new pressure sample is available. P_DA is cleared when
		 * PRESS_OUT_H (2Ah) register is read.
		 */
		struct P_DA
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for pressure is not yet available;
			static const uint8_t NEW_DATA = 0b1; // new data for pressure is available
		};
		/* Bits T_DA: */
		/*
		 * Temperature data available.
		 * T_DA is set to 1 whenever a new temperature sample is available. T_DA is cleared
		 * when TEMP_OUT_H (2Ch) register is read.
		 */
		struct T_DA
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for temperature is not yet available
			static const uint8_t NEW_DATA = 0b1; // new data for temperature is available
		};
	};
	
	/* Set register STATUS_REG */
	void setSTATUS_REG(uint8_t value)
	{
		write(STATUS_REG::__address, value, 8);
	}
	
	/* Get register STATUS_REG */
	uint8_t getSTATUS_REG()
	{
		return read8(STATUS_REG::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG PRESS_OUT                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PRESS_OUT:
	 * 7.13-15
	 * The PRESS_OUT_XL register contains the lowest part of the pressure output value,
	 * that is the difference between the measured pressure and the reference pressure
	 * (REF_P registers). See AUTOZERO bit in CTRL_REG2.
	 * The full reference pressure value is composed by PRESS_OUT_H/_L/_XL and is represented
	 * as 2’s complement. Pressure Values exceeding the operating pressure Range (see Table 3)
	 * are clipped.
	 * Pressure output data: Pout(hPa) = PRESS_OUT / 4096
	 * Example: P_OUT = 0x3ED000 LSB = 4116480 LSB = 4116480/4096 hPa= 1005 hPa Default value
	 * is 0x2F800 = 760 hPa
	 */
	struct PRESS_OUT
	{
		static const uint16_t __address = 40;
		
		/* Bits PRESS_OUT: */
		struct PRESS_OUT_
		{
			/* MODE r */
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register PRESS_OUT */
	void setPRESS_OUT(uint32_t value)
	{
		write(PRESS_OUT::__address, value, 24);
	}
	
	/* Get register PRESS_OUT */
	uint32_t getPRESS_OUT()
	{
		return read32(PRESS_OUT::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG TEMP_OUT                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG TEMP_OUT:
	 * 7.16-17
	 * Temperature output value
	 * The TEMP_OUT_L register contains the low part of the temperature output value.
	 * Temperature data are expressed as TEMP_OUT_H & TEMP_OUT_L as 2’s complement numbers.
	 * Temperature output data:
	 * T(°C) = 42.5 + (TEMP_OUT / 480)
	 * If TEMP_OUT = 0 LSB then Temperature is 42.5 °C
	 */
	struct TEMP_OUT
	{
		static const uint16_t __address = 43;
		
		/* Bits TEMP_OUT: */
		struct TEMP_OUT_
		{
			/* MODE r */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register TEMP_OUT */
	void setTEMP_OUT(uint8_t value)
	{
		write(TEMP_OUT::__address, value, 8);
	}
	
	/* Get register TEMP_OUT */
	uint8_t getTEMP_OUT()
	{
		return read8(TEMP_OUT::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG FIFO_CTRL                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_CTRL:
	 * 7.18
	 * FIFO control register. The FIFO_CTRL registers allows to control the FIFO functionality.
	 * 
	 */
	struct FIFO_CTRL
	{
		static const uint16_t __address = 46;
		
		/* Bits F_MODE: */
		/*
		 * FIFO mode selection.
		 * FIFO_MEAN_MODE: The FIFO can be used for implementing a HW moving average on the pressure
		 * measurements. The number of samples of the moving average can be 2, 4, 8, 16 or 32 samples,
		 * by selecting the watermark levels as per Table 21. Different configuration are not
		 * guaranteed.
		 * When using the FIFO_MEAN_MODE it is not possible to access the FIFO.
		 */
		struct F_MODE
		{
			/* MODE rw */
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b11100000; // [5,6,7]
			static const uint8_t BYPASS = 0b00; // Bypass mode
			static const uint8_t FIFO = 0b01; // FIFO mode. Stops collecting data when full
			static const uint8_t STREAM = 0b10; // Stream mode: Keep the newest measurements in the FIFO
			static const uint8_t STREAM_TO_FIFO = 0b11; // Stream mode until trigger deasserted, then change to FIFO MODE
			static const uint8_t BYPASS_TO_STREAM = 0b100; // Bypass mode until trigger deasserted, then change to STREAM MODE
			static const uint8_t reserved_0 = 0b101; // Reserved for future use
			static const uint8_t FIFO_MEAN_MODE = 0b110; // FIFO is used to generate a running average filtered pressure
			static const uint8_t BYPASS_TO_FIFO = 0b111; // BYPASS mode until trigger deasserted, then change to FIFO MODE
		};
		/* Bits WTM_POINT: */
		/*
		 * FIFO threshold (watermark) level selection.
		 * Please note that when using the FIFO Mean mode it is not possible to access the FIFO content.
		 */
		struct WTM_POINT
		{
			/* MODE rw */
			static const uint8_t dflt = 0b00000; // 5'b0
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
			static const uint8_t SAMPLE_2 = 0b001; // 2-sample moving average
			static const uint8_t SAMPLE_4 = 0b011; // 4-sample moving average
			static const uint8_t SAMPLE_8 = 0b111; // 8-sample moving average
			static const uint8_t SAMPLE_16 = 0b1111; // 16-sample moving average
			static const uint8_t SAMPLE_32 = 0b11111; // 32-sample moving average
		};
	};
	
	/* Set register FIFO_CTRL */
	void setFIFO_CTRL(uint8_t value)
	{
		write(FIFO_CTRL::__address, value, 8);
	}
	
	/* Get register FIFO_CTRL */
	uint8_t getFIFO_CTRL()
	{
		return read8(FIFO_CTRL::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG FIFO_STATUS                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_STATUS:
	 * 7.19
	 * FIFO status
	 */
	struct FIFO_STATUS
	{
		static const uint16_t __address = 47;
		
		/* Bits WTM_FIFO: */
		/* Watermark status.  */
		struct WTM_FIFO
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t FIFO_BELOW = 0b0; // FIFO level lower than watermark level
			static const uint8_t FIFO_AT_OR_ABOVE = 0b1; // FIFO is equal or higher than watermark level
		};
		/* Bits FULL_FIFO: */
		/* Overrun bit status.  */
		struct FULL_FIFO
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t FIFO_NOT_FULL = 0b0; // 
			static const uint8_t FIFO_FULL = 0b1; // 
		};
		/* Bits EMPTY_FIFO: */
		/* Empty FIFO bit.  */
		struct EMPTY_FIFO
		{
			/* MODE r */
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t FIFO_NOT_EMPTY = 0b0; // 
			static const uint8_t FIFO_EMPTY = 0b1; // 
		};
		/* Bits DIFF_POINT: */
		/* FIFO stored data level.  */
		struct DIFF_POINT
		{
			/* MODE r */
			static const uint8_t dflt = 0b00000; // 5'b0
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
		};
	};
	
	/* Set register FIFO_STATUS */
	void setFIFO_STATUS(uint8_t value)
	{
		write(FIFO_STATUS::__address, value, 8);
	}
	
	/* Get register FIFO_STATUS */
	uint8_t getFIFO_STATUS()
	{
		return read8(FIFO_STATUS::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG THS_P                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG THS_P:
	 * 7.20-21
	 * Threshold value for pressure interrupt generation.
	 * The complete threshold value is given by THS_P and is expressed as unsigned number.
	 * P_ths (hPa) = (THS_P)/16.
	 */
	struct THS_P
	{
		static const uint16_t __address = 48;
		
		/* Bits THS: */
		/* Refer to Section 10.2: "THS_P_L (0Ch)"  */
		struct THS
		{
			/* MODE rw */
			static const uint16_t dflt = 0b0000000000000000; // 16'b0
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register THS_P */
	void setTHS_P(uint16_t value)
	{
		write(THS_P::__address, value, 16);
	}
	
	/* Get register THS_P */
	uint16_t getTHS_P()
	{
		return read16(THS_P::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG RPDS                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RPDS:
	 * 7.22-23
	 * Pressure offset.
	 * This register contains the pressure offset value after soldering,for differential pressure computing.
	 * The value is expressed as signed 2 complement value.
	 */
	struct RPDS
	{
		static const uint16_t __address = 57;
		
		/* Bits RPDS: */
		struct RPDS_
		{
			/* MODE rw */
			static const uint16_t dflt = 0b0011100000000000; // 16'b11100000000000
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register RPDS */
	void setRPDS(uint16_t value)
	{
		write(RPDS::__address, value, 16);
	}
	
	/* Get register RPDS */
	uint16_t getRPDS()
	{
		return read16(RPDS::__address, 16);
	}
	
};
