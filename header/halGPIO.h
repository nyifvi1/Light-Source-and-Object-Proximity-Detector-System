#ifndef _halGPIO_H_
#define _halGPIO_H_

#include  "../header/bsp.h"    		// private library - BSP layer
#include  "../header/app.h"    		// private library - APP layer


extern enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable

extern int command_flag;
extern int UART_ACK;
extern int telemeter_angle;
extern char *Flash_ptr; 
extern int fileIndex;
extern int SetFileStage;
extern int get_file_done;
extern int PB0_Counter;
extern int show_names_flag;
extern int d;
extern int st9_deg_scan_flag;
extern int finish_servo_scan;
extern int start_servo_scan;
extern int st9_servo_scan_flag;

extern void sysConfig(void);
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void enable_interrupts();
extern void disable_interrupts();
extern void ServoDeg(int degree);
extern void ServoTimerCfg(int movement_strgth_flag);
extern void UltrasonicSamp();
extern void SendUltrasonicSamp(int angle);
extern int LDRSamp(int LDRid);
extern void SendLightSamp(int angle, int LDR0_val, int LDR1_val);
extern void DelayOf10Xms(int x);
extern void enablePB0_IE();
extern void disablePB0_IE();
extern void SendUltrasonicAndLightSamp(int angle, int LDR0_val, int LDR1_val);
extern void init_flash_write(int addr);
extern void disable_flash_write();
extern void write_with_addr_flash_char(char value, int addr);
extern void write_flash_char(char value);
extern void write_with_addr_flash_int(int value, int addr);
extern void send_UART_ack(char a);
extern void enablePB0_PB1();
extern void disablePB0_PB1();
extern void inc_lcd(unsigned char x);
extern void dec_lcd(unsigned char x);
extern void rra_lcd(unsigned char x);

extern void lcd_init(void);
extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);
extern __interrupt void PBs_handler(void);


typedef struct {
    int *numFiles;
    char *filesType[10];
    char *filesName[10];
    int *filesSize[10];
    int *filesLocation[10];
} fileManager;

extern fileManager FilesInst;
#endif







