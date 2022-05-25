#define BASE_ADDR		0x40000000
#define GPIO_PL_0		(0x40020000 - BASE_ADDR)

#define ADC1_RAM		0x40000000
#define ADC2_RAM		0x40010000

#define ADC1_GPIO		0x40040000
#define ADC2_GPIO		0x40030000

#define ADC1_DMA		0x40050000
#define ADC2_DMA		0x40060000

#define ADC1_TMR		0x40050014
#define ADC2_TMR		0x40060014

#define ADC1_CTRL		(0x40050018 - BASE_ADDR)
#define ADC2_CTRL		(0x40060018 - BASE_ADDR)

#define ADC1_CADDR		(0x40050020 - BASE_ADDR)
#define ADC2_CADDR		(0x40060020 - BASE_ADDR)

#define ADC1_ON			0x100
#define ADC2_ON			0x400

#define ADC1_OFF		0x200
#define ADC2_OFF		0x800

#define ADC_RAM_SIZE	 		0x8000
#define ADC_READY_RAM_SIZE	0x7E00
#define DATA_BUF_SIZE 		0x4000

#define DATA_ARRAY_SIZE		0x4000	// adc 16384 of 16 bit elements per channel

#define DATA_FILE_PATH 		"/mnt/data/myfile/"

#define MAP_SIZE 		0x70000
#define MAP_MASK 		(MAP_SIZE - 1)


#define SETUP_FILE_NAME 	"/home/nikolai/Test_GUI/server/pitaya_server/settings.cfg"

#define DEV_ID			0xFA

#define CMD_EXIT 		0xFF
#define CMD_SET			0x02
#define CMD_GET			0x04
#define CMD_ADC_RUN		0x08
#define CMD_ADC_STOP 		0x0a

#define REG_ALL			0xFF
#define REG_CFG			0xFA
#define REG_STATE		0xFB
#define REG_SYNC		0x02
#define REG_TIME		0x04
#define REG_ADC_DATA		0x06
#define REG_ADC_DATA_16		0x07
#define REG_TIMEOUT		0x08
#define REG_COUNTER		0x09
#define REG_FILE_RUN 	0x0a
#define REG_RECORDS		0x0b
#define REG_AVERAGE		0x0c
#define REG_CH1_NAME		0x0d
#define REG_CH2_NAME		0x0e

#define SYNC_EXT		0xFF
#define SYNC_INT		0x01
#define SYNC_SOURCE_BIT		0x02
#define SYNC_INT_BIT		0x01
#define SYNC_INT_DELAY		50  		// intrnal sync delay (ms)
#define SYS_IDLE		1		// system_idle (ms)
#define SYNC_PULSE_WIDTH	50 		// usec 


// config file defines

#define ADC_RUN_ADDR 		0x00
#define FILE_REC_RUN_ADDR 	0x01
#define REC_PER_FILE_ADDR 	0x02
#define DATA_AVERAGE_ADDR 	0x04
#define SYNC_SOURCE_ADDR 	0x06
#define ADC_TIMEOUT_ADDR 	0x08
#define TOTAL_PULSES_ADDR 	0x0a
#define CH_NAMES_ADDR 		0x12

