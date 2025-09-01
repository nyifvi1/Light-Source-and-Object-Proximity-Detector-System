#include  "../header/halGPIO.h"     // private library - HAL layer
#include <stdlib.h>
#include "stdio.h"

int EchoCap_flag = 0,RisingEchoVal=0,FallingEchoVal=0,Ultrasonic_sample=0;
int UART_ready_flag=1,UART_ACK=0,UART_TX_Buffer_Size;
int LDR_val=0,telemeter_angle;
unsigned char LDR_sample_compressed[4]; 
unsigned char UART_transmit_buffer[10];
char *Flash_ptr; 
int fileIndex;
char tmp[4];   
int SetFileStage,write_offset;
int get_file_done,PB0_Counter,show_names_flag,d=50,st9_deg_scan_flag,st9_servo_scan_flag,start_servo_scan,finish_servo_scan;
char i_str[4];


char input_str[22];
int input_index = 0;
int command_flag = 1; 
unsigned int TXindex;
//--------------------------------------------------------------------
//             Flash pointers Configuration  
//--------------------------------------------------------------------
fileManager FilesInst = {
    .numFiles = (int*)0xF200,    //2 bytes

    .filesType = {                //single byte each           
        (char*)0xF401,
        (char*)0xF402,
        (char*)0xF403,
        (char*)0xF404,
        (char*)0xF405,
        (char*)0xF406,
        (char*)0xF407,
        (char*)0xF408,
        (char*)0xF409,
        (char*)0xF40A
    },

    .filesName = {                //16 bytes each
        (char*)0xF40C,
        (char*)0xF41C,
        (char*)0xF42C,
        (char*)0xF43C,
        (char*)0xF44C,
        (char*)0xF45C,
        (char*)0xF46C,
        (char*)0xF47C,
        (char*)0xF48C,
        (char*)0xF49C
    },

    .filesSize = {            // 2 bytes each
        (int*)0xF4AC,
        (int*)0xF4AE,
        (int*)0xF4B0,
        (int*)0xF4B1,
        (int*)0xF4B2,
        (int*)0xF4B4,
        (int*)0xF4B6,
        (int*)0xF4B8,
        (int*)0xF4BA,
        (int*)0xF4BC
    },

  .filesLocation = {            // 4 bytes each
      (int*)0xF4BE,
      (int*)0xF4C2,
      (int*)0xF4C6,
      (int*)0xF4CA,
      (int*)0xF4CE,
      (int*)0xF4D2,
      (int*)0xF4D6,
      (int*)0xF4DA,
      (int*)0xF4DE,
      (int*)0xF4E2
}
};
//--------------------------------------------------------------------
//             System Configuration  
//--------------------------------------------------------------------
void sysConfig(void){ 
	GPIOconfig();
	ServoConfig();
  UltrasConfig();
	ADCconfig();
	lcd_init();
	UARTconfig();
  FlashConfig();
}
//---------------------------------------------------------------------
//            Polling based Delay function
//---------------------------------------------------------------------
void delay(unsigned int t){  // t[msec]
	volatile unsigned int i;
	
	for(i=t; i>0; i--);
}
//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01) 
	  _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02) 
	  _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
	  _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04) 
	  _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}
//---------------------------------------------------------------------
//            Enable interrupts
//---------------------------------------------------------------------
void enable_interrupts(){
  _BIS_SR(GIE);
}
//---------------------------------------------------------------------
//            Disable interrupts
//---------------------------------------------------------------------
void disable_interrupts(){
  _BIC_SR(GIE);
}
//---------------------------Servo Drivers--------------------------------
void ServoDeg(int degree){
	//insert degree to TA1CRR1
	TA1CCR1 = 550 + degree*8;  // 630 = degree 0, 2610 = degree 180
	TA1R = 0;
	TA1CCTL1 = OUTMOD_6;               //Toggle set
	TA1CTL |= MC_1;
	//start TA0 to count 100m
	TA0R = 0;
	TA0CCTL0 = CCIE;
	TA0CTL |= MC_1; 
	LPM0; //wait 100ms
	//finish PWM output
 	TA1CCTL1 = OUTMOD_5;               //reset
  //halt TA0
  TA0CTL &= ~MC_1;                   
	TA0CCTL0 &= ~CCIE;
}
void ServoTimerCfg(int movement_strgth_flag){   //movement_strgth_flag=1 means big movement offset 
//PWM generation using TA1
	TA1CTL = TASSEL_2+MC_0;            //Smclk+HaltTimer
	TA1CCTL1 = OUTMOD_5;               //reset
	TA1CCR0 = 26400;					//Freq of 40Hz PWM
	TA1CCR1 = 630;				   //Period of '1' output (degree=0)
//100ms delay using TA0
	TA0CTL = TASSEL_2+ID_3+MC_0;            //Smclk+Div8+HaltTimer
  if (movement_strgth_flag==1){
    	TA0CCR0 = 0xFFFF;                       // set time between servo samples 
  }
  else{
	    TA0CCR0 = 0x2000;                       // set time between servo samples 0x2000
  }
}
//---------------------------Ultrasonic Drivers--------------------------------
void UltrasonicSamp(){
  EchoCap_flag = 0;
//Pulse generation using TA1
	TA1CTL = TASSEL_2+MC_0;            //Smclk+HaltTimer
	TA1CCTL2 = OUTMOD_5;               //reset
	TA1CCR0 = 62915;					//Period of 60ms PWM 
	TA1CCR2 = 60;				   //Period of '1' output (more than 10us)
//Capture cfg
  TA1CCTL1 = CM_3+CCIS_1+SCS+CAP;  //CaptureOnBothRisAndFall+CCI1B+SynCap+CaptureMode
  TA1CCTL1 |= CCIE;  //start capture
//start pulse
	TA1R = 0;
	TA1CCTL2 = OUTMOD_6;               //Toggle set
	TA1CTL |= MC_1;
  while(EchoCap_flag != 2){
      LPM0; // wait for falling edge echo sample
  } 
  TA1CCTL2 = OUTMOD_5;               //reset
  Ultrasonic_sample=FallingEchoVal-RisingEchoVal;
  //maybe flag to check if 60ms passed 
}
void SendUltrasonicSamp(int angle){
  UART_ACK = 0;
      while(UART_ACK==0){
        if(state == state9 && st9_deg_scan_flag == 0 && st9_servo_scan_flag == 0){
            break;
        }
        UART_transmit_buffer[1] = (unsigned char) angle;
        UART_transmit_buffer[2] = (unsigned char)(Ultrasonic_sample & 0xFF); 
        UART_transmit_buffer[3] = (unsigned char)((Ultrasonic_sample >> 8) & 0xFF); 
        UART_transmit_buffer[4] = 256 - ((UART_transmit_buffer[1]+UART_transmit_buffer[2]+UART_transmit_buffer[3]) % 256);
        UART_transmit_buffer[5] = '$';
        TXindex = 0;
        UART_TX_Buffer_Size = 4; 
        UART_ready_flag = 0;
        IE2 |= UCA0TXIE;                   // Enable USCI_A0 TX interrupt
        UCA0TXBUF = UART_transmit_buffer[0];           // Start transmission 
        while(UART_ready_flag != 1){
          DelayOf10Xms(1);         
        } 
        //timer 0.5s
        TA0CTL = TASSEL_2+ID_3+MC_0;            //Smclk+Div8+disableTA
        TA0CCR0 = 0xFFFF;      // value of 0.5 sec
        TA0R=0x0000;
        //startTA0
        TA0CTL |= MC_1;    //Up Mode
        TA0CCTL0 |= CCIE;
        LPM0;
        //haltTA0
        TA0CTL &= ~MC_1; 
        TA0CCTL0 &= ~CCIE;
      }
}
//---------------------------LightSourceDetector Drivers--------------------------------
int LDRSamp(int LDRid){
  ADC10CTL0 = ADC10SHT_2+ADC10ON+ADC10IE;            //16xADC10CLKS+ADC10ON+ADC10IE
  if (LDRid==0){
    	ADC10CTL1 = INCH_0+ADC10SSEL_0+CONSEQ_0;           //ChannelA0+ADC10OSCreference+SingleChannelSingleConversion
  }
  else{
    	ADC10CTL1 = INCH_3+ADC10SSEL_0+CONSEQ_0;           //ChannelA3+ADC10OSCreference+SingleChannelSingleConversion
  }
	ADC10CTL0 |= ADC10SC+ENC;
  LPM0;
  ADC10CTL0 &= ~(ADC10ON+ENC+ADC10IE);  //turn off ADC10
  return ADC10MEM;    //return read LDR value  
}
void SendLightSamp(int angle, int LDR0_val, int LDR1_val){         
    UART_ACK = 0;
      while(UART_ACK==0){
        UART_transmit_buffer[1] = (unsigned char) angle;
        UART_transmit_buffer[2] = (unsigned char)(LDR0_val & 0x00FF); 
        UART_transmit_buffer[3] = (unsigned char)(((LDR1_val & 0x003F) << 2) | ((LDR0_val & 0x0300) >> 8) ); 
        UART_transmit_buffer[4] = (unsigned char)(((LDR1_val & 0x03C0) >> 6)); 
        UART_transmit_buffer[5] = 256 - ((UART_transmit_buffer[1]+UART_transmit_buffer[2]+UART_transmit_buffer[3]+UART_transmit_buffer[4]) % 256);
        UART_transmit_buffer[6] = '$';
        TXindex = 0;
        UART_TX_Buffer_Size = 5; 
        UART_ready_flag = 0;
        IE2 |= UCA0TXIE;                   // Enable USCI_A0 TX interrupt
        UCA0TXBUF = UART_transmit_buffer[0];           // Start transmission 
        while(UART_ready_flag != 1){
          DelayOf10Xms(1);         
        } 
        //timer 0.5s
        TA0CTL = TASSEL_2+ID_3+MC_0;            //Smclk+Div8+disableTA
        TA0CCR0 = 0xFFFF;      // value of 0.5 sec
        TA0R=0x0000;
        //startTA0
        TA0CTL |= MC_1;    //Up Mode
        TA0CCTL0 |= CCIE;
        LPM0;
        //haltTA0
        TA0CTL &= ~MC_1; 
        TA0CCTL0 &= ~CCIE;
      }
}
//---------------------------Bonus Drivers--------------------------------
void SendUltrasonicAndLightSamp(int angle, int LDR0_val, int LDR1_val){
  UART_ACK = 0;
      while(UART_ACK==0){
        UART_transmit_buffer[1] = (unsigned char) angle;
        UART_transmit_buffer[2] = (unsigned char)(Ultrasonic_sample & 0xFF); 
        UART_transmit_buffer[3] = (unsigned char)((Ultrasonic_sample >> 8) & 0xFF); 
        UART_transmit_buffer[4] = (unsigned char)(LDR0_val & 0x00FF); 
        UART_transmit_buffer[5] = (unsigned char)(((LDR1_val & 0x003F) << 2) | ((LDR0_val & 0x0300) >> 8) ); 
        UART_transmit_buffer[6] = (unsigned char)(((LDR1_val & 0x03C0) >> 6)); 
        UART_transmit_buffer[7] = 256 - ((UART_transmit_buffer[1]+UART_transmit_buffer[2]+UART_transmit_buffer[3]+UART_transmit_buffer[4]+UART_transmit_buffer[5]+UART_transmit_buffer[6]) % 256);        
        UART_transmit_buffer[8] = '$';
        TXindex = 0;
        UART_TX_Buffer_Size = 7; 
        UART_ready_flag = 0;
        IE2 |= UCA0TXIE;                   // Enable USCI_A0 TX interrupt
        UCA0TXBUF = UART_transmit_buffer[0];           // Start transmission 
        while(UART_ready_flag != 1){
          DelayOf10Xms(1);         
        } 
        //timer 0.5s
        TA0CTL = TASSEL_2+ID_3+MC_0;            //Smclk+Div8+disableTA
        TA0CCR0 = 0xFFFF;      // value of 0.5 sec
        TA0R=0x0000;
        //startTA0
        TA0CTL |= MC_1;    //Up Mode
        TA0CCTL0 |= CCIE;
        LPM0;
        //haltTA0
        TA0CTL &= ~MC_1; 
        TA0CCTL0 &= ~CCIE;
      }  
}
//---------------------------File mode Drivers--------------------------------
void inc_lcd(unsigned char x){
  int val = x;
  int i=0;
  while(i<=val){
    lcd_clear();
    lcd_home();
    sprintf(i_str, "%d", i);  
		lcd_puts(i_str);
    DelayOf10Xms(d);
    i++;
  }
}
void dec_lcd(unsigned char x){
  int i=x;
  while(i>=0){
    lcd_clear();
    lcd_home();    
    sprintf(i_str, "%d", i);  
		lcd_puts(i_str);
    DelayOf10Xms(d);
    i--;
  }
}
void rra_lcd(unsigned char x){
  int i=0,k;
  for(i;i<32;i++){
      lcd_clear();
      lcd_home();
      for (k = 0; k < 16; k++){
        if(k==i){
          lcd_data(x);
        }
        else{
          lcd_data(' ');
        }
      }
      lcd_new_line;
      for (k = 16; k < 32; k++){
        if(k==i){
          lcd_data(x);
        }
        else{
          lcd_data(' ');
        }
      }
      DelayOf10Xms(d);
  }

}

//---------------------------General Drivers--------------------------------
void DelayOf10Xms(int x){ 
	int temp = x*10;  
  TA0CTL = TASSEL_2+ID_3+MC_0;            //Smclk+Div8+disableTA
  while(temp > 500){
      TA0CCR0 = 0xFFFF;      // value of 0.5 sec
      TA0R=0x0000;
      //startTA0
      TA0CTL |= MC_1;    //Up Mode
      TA0CCTL0 |= CCIE;
      LPM0;
      //haltTA0
      TA0CTL &= ~MC_1; 
      TA0CCTL0 &= ~CCIE;
      temp -= 500;
 	}
  if (temp != 0){
      temp = temp<<7;    //multiply temp by 1024 
      TA0CCR0 = temp;
      TA0R=0x0000;
      //startTA0
      TA0CTL |= MC_1;    //Up Mode
      TA0CCTL0 |= CCIE;
      LPM0;
      //haltTA0
      TA0CTL &= ~MC_1; 
      TA0CCTL0 &= ~CCIE;
  }
}
void enablePB0_IE(){
	P2IE |= PB0;
}
void disablePB0_IE(){
	P2IE &= ~PB0;
}
void enablePB0_PB1(){
  P2IE |= PB0+PB1;
}
void disablePB0_PB1(){
  P2IE &= ~(PB0+PB1);
}
void send_UART_ack(char a){
    UART_transmit_buffer[1] = (unsigned char) a;
    UART_transmit_buffer[2] = 256 - (UART_transmit_buffer[1] % 256);
    UART_transmit_buffer[3] = '$';
    TXindex = 0;
    UART_TX_Buffer_Size = 2; 
    UART_ready_flag = 0;
    IE2 |= UCA0TXIE;                   // Enable USCI_A0 TX interrupt
    UCA0TXBUF = UART_transmit_buffer[0];           // Start transmission 
    while(UART_ready_flag != 1){
      asm(" nop");  //wait for flag without using timers        
    } 
}
//---------------------------Flash Drivers--------------------------------
void init_flash_write(int addr){
    Flash_ptr = (char *) addr;                // Initialize Flash pointer  
    FCTL1 = FWKEY + ERASE;                    // Set Erase bit
    FCTL3 = FWKEY;                            // Clear Lock bit
    *Flash_ptr = 0;                           // Dummy write to erase Flash segment
    FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
}

void disable_flash_write(){
    FCTL1 = FWKEY;                            // Clear WRT bit
    FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

void write_with_addr_flash_char(char value, int addr){

    Flash_ptr = (char *) addr;
    FCTL1 = FWKEY;                      // Set WRT bit for write operation
    FCTL3 = FWKEY;                            // Clear Lock bit
    FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

    *Flash_ptr = (char)value;
}

void write_flash_char(char value){
    *Flash_ptr++ = (char)value;               // Write value to flash
}

void write_with_addr_flash_int(int value, int addr){

    Flash_ptr = (char *) addr;
    write_with_addr_flash_char((char)(value & 0xFF), Flash_ptr);
    *Flash_ptr++;
    write_with_addr_flash_char((char)((value >> 8) & 0xFF), Flash_ptr);
    
}
//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(){
  
	char init_value;

	if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
        else init_value = 0x3F;
	
	LCD_RS_DIR(OUTPUT_PIN);
	LCD_EN_DIR(OUTPUT_PIN);
	LCD_RW_DIR(OUTPUT_PIN);
        LCD_DATA_DIR |= OUTPUT_DATA;
        LCD_RS(0);
	LCD_EN(0);
	LCD_RW(0);
        
	DelayMs(15);
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	DelayMs(5);
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	DelayUs(200);
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	
	if (LCD_MODE == FOURBIT_MODE){
		LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
		LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
		lcd_strobe();
		lcd_cmd(0x28); // Function Set
	}
        else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots 
	
	lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
	lcd_cmd(0x1); //Display Clear
	lcd_cmd(0x6); //Entry Mode
	lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){
  
	LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

	if (LCD_MODE == FOURBIT_MODE)
	{
		LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
		lcd_strobe();
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
    		LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
		lcd_strobe();
	}
	else
	{
		LCD_DATA_WRITE = c;
		lcd_strobe();
	}
}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){
        
	LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

	LCD_DATA_WRITE &= ~OUTPUT_DATA;       
	LCD_RS(1);
	if (LCD_MODE == FOURBIT_MODE)
	{
    		LCD_DATA_WRITE &= ~OUTPUT_DATA;
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;  
		lcd_strobe();		
                LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
		LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET; 
		lcd_strobe();
	}
	else
	{
		LCD_DATA_WRITE = c;
		lcd_strobe();
	}
          
	LCD_RS(0);   
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){
  
	while(*s)
		lcd_data(*s++);
}
//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){
  
	unsigned char i;
        for(i=cnt ; i>0 ; i--) asm(" nop"); // tha command asm("nop") takes raphly 1usec
	
}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){
  
	unsigned char i;
        for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec
	
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm(" nop");
  asm(" nop");
  LCD_EN(0);
}
//*********************************************************************
//            Port1 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT2_VECTOR
  __interrupt void PBs_handler(void){

    delay(debounceVal);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------
    if(PBsArrIntPend & PB0){
      PBsArrIntPend &= ~PB0;
        if(state == state8 || state == state9){
          PB0_Counter++;
        }
        LPM0_EXIT;
    }
    if(PBsArrIntPend & PB1){
      PBsArrIntPend &= ~PB1;
        if(state == state8 || state == state9){
          show_names_flag = 1-show_names_flag;
          PB0_Counter = 0;   
        }
        LPM0_EXIT;
    }


}
//*********************************************************************
//            TA1 Interrupt Service Rotine
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TA1IV,0x0A))
  {
      case  TA1IV_NONE: break;              // Vector  0:  No interrupt
      case  TA1IV_TACCR1:                   // Vector  2:  TACCR1 CCIFG
            if(EchoCap_flag == 0){
               RisingEchoVal = TA1CCR1; 
               EchoCap_flag = 1;
            }
            else{
               FallingEchoVal = TA1CCR1;   
               EchoCap_flag = 2;  
               TA1CCTL1 &= ~CCIE;  //end capture
            }
            LPM0_EXIT;
            break;           
      case  TA1IV_TACCR2: break;            // Vector  4:  TACCR2 CCIFG
      case  TA1IV_TAIFG: break;             // Vector 10:  TAIFG
      default: 	break;
  }
} 
//*********************************************************************
//            TimerA0 Interrupt Service Rotine
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
  LPM0_EXIT;
}
//---------------------------------------------------------------------
//            USCI A0/B0 Transmit ISR
//---------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if ((TXindex+1) == UART_TX_Buffer_Size){           // TX over?
      UART_ACK = 0;
      IE2 &= ~UCA0TXIE;                       // Disable USCI_A0 TX interrupt
      UCA0TXBUF = UART_transmit_buffer[TXindex+1];
      UART_ready_flag = 1;
    } 
  else{
	    TXindex++;
	    UCA0TXBUF = UART_transmit_buffer[TXindex];  
    }
}
//---------------------------------------------------------------------
//            USCI A0/B0 Receive ISR
//---------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  input_str[input_index] = UCA0RXBUF;
  input_index++;
  if ((input_str[input_index-1] == '$' && input_str[input_index-2] == '$') || (input_index == 19 && input_str[18] != '$'))                      
  {
      if(command_flag){                     //command_flag='1' (command)      command_flag='0' (data)

          switch(input_str[0]){
              case '0' :
                  state = state0;
                  break;            
              case '1' :
                  state = state1;
                  break;
              case '2' :
                  state = state2;
                  break;
              case '3' :
                  state = state3;
                  break;
              case '4' :
                  state = state4;
                  break;
              case '5' :
                  state = state5;
                  break;
              case '6' :
                  state = state6;
                  break;
              case '7' :
                  state = state7;
                  break;
              case '8' :
                  state = state8;
                  break;
              case '9' :
                  state = state9;
                  break;  
              case 'A':
                  state = state10;
                  break;

              case 'T' :
                  UART_ACK=1;
                  break;
              case 'F' :
                  UART_ACK=0;
                  break;
              case 'D':
                  st9_deg_scan_flag = 0;
                  break;
              case 'S':
                  finish_servo_scan = 1;
                  break;
              case 'U':
                  start_servo_scan=1;
                  break;
              default:
                  break;
          }
          LPM0_EXIT;
      }
      else{                                   
          switch(state){
              case state2:
                  tmp[0] = input_str[0];
                  tmp[1] = input_str[1];
                  tmp[2] = input_str[2];
                  tmp[3] = '\0';
                  telemeter_angle = atoi(tmp); //convert telemeter angle to int
                  LPM0_EXIT;
                  command_flag = 1;
                  break; 
              case state5:
                  if (SetFileStage == 0){       //get file type
                      write_with_addr_flash_char(input_str[0],FilesInst.filesType[fileIndex]);
                      disable_flash_write();
                      SetFileStage = 1;
                  }
                  else if(SetFileStage == 1){  //get file name
                      int i=0;
                      for(i;i<16;i++){
                        write_with_addr_flash_char(input_str[i],FilesInst.filesName[fileIndex]+i);
                        disable_flash_write();
                      }
                      SetFileStage = 2;
                      write_offset = 0;
                      LPM0_EXIT;
                  } 
                  else if (SetFileStage == 2){
                    if(input_str[input_index-1] == '$' && input_str[input_index-2] == '$'){   //last chunk of data
                      int i=0;
                      for(i;i<(input_index - 2);i++){
                        write_with_addr_flash_char(input_str[i],*FilesInst.filesLocation[fileIndex]+write_offset);
                        disable_flash_write();
                        write_offset++;
                      }
                       write_with_addr_flash_int(write_offset,FilesInst.filesSize[fileIndex]);    //set size
                       disable_flash_write();
                       get_file_done = 1;
                       LPM0_EXIT; 
                    }
                    else{ 
                    int i=0;                                                                        //mid chunk of data
                    for(i;i<(input_index);i++){
                        write_with_addr_flash_char(input_str[i],*FilesInst.filesLocation[fileIndex]+write_offset);
                        disable_flash_write();
                        write_offset++;
                    }
                    LPM0_EXIT;
                    } 
                  }
                  break;    
              default:
                  break;
          }
      	}
    input_index = 0;
    //IE2 |= UCA0TXIE;                        // Enable USCI_A0 TX interrupt
  }
}
//*********************************************************************
//            ADC10 Interrupt Service Rotine
//*********************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
  LPM0_EXIT;
}
