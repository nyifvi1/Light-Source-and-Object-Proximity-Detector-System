#include  "../header/bsp.h"    // private library - BSP layer

//-----------------------------------------------------------------------------  
//           GPIO congiguration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
 // volatile unsigned int i; // in case of while loop usage
  
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
   
  // LCD Setup
  LCD_DATA_WRITE &= 0x0F;   //bic Px.7-Px.4
  LCD_DATA_DIR |= 0xF0;     //bis Px.7-Px.4
  LCD_DATA_SEL &= 0x0F;
  LCD_CNTL_SEL &= ~0xE0;    //bic Px.7-Px.5
  
  
   // PushButtons Setup          PB0,PB1 configuration
  PBsArrPortSel &= ~(PB0+PB1); 
  PBsArrPortDir &= ~(PB0+PB1);
  PBsArrIntEdgeSel |= (PB0+PB1);  	// pull-up mode
  PBsArrIntEn &= ~(PB0+PB1);        //disable IE
  PBsArrIntPend &= ~(PB0+PB1);      // clear pending interrupts 
  
  _BIS_SR(GIE);                     // enable interrupts globally
}                             
//------------------------------------------------------------------------------------- 
//            Servo ports configuration 
//-------------------------------------------------------------------------------------
void ServoConfig(void){ // P2.1 output compare (PWM) Timer1_TACCR1 => Servo angle input                            
        P2DIR |= 0x02;   //bis BIT1  
        P2SEL |= 0x02;   //bis BIT1
        P2SEL2 &= ~0x02; //bic BIT1
} 
//------------------------------------------------------------------------------------- 
//            Ultrasonic ports configuration 
//-------------------------------------------------------------------------------------
void UltrasConfig(void){    // P2.4 output comapre (PWM) Timer1_TACCR2 => Ultrasonic Trigger 
                            // P2.2 input capture Timer1_TACCR1 => Ultrasonic Echo
        P2DIR |= 0x10;   //bis BIT4  
        P2DIR &= ~0x04;  //bic BIT2  
        P2SEL |= 0x14;   //bis BIT4+BIT2
        P2SEL2 &= ~0x14; //bic BIT4+BIT2
} 
//------------------------------------------------------------------------------------- 
//            ADC configuration 
//-------------------------------------------------------------------------------------
void ADCconfig(void){     // (LDR2 - A3 P1.3, LDR1 - A0 - P1.0)
      ADC10AE0 |= BIT3+BIT0;   //enable A3,A0 as analog input  
      //ADC10CTL1 = INCH3;  // INCH3 - select A3 as input channel
      CAPD &= ~(CAPD3+CAPD0);     //disable comparator port for A3,A0
} 
//------------------------------------------------------------------------------------- 
//            UART configuration 
//-------------------------------------------------------------------------------------
void UARTconfig(void){
    if (CALBC1_1MHZ==0xFF){					  // If calibration constant erased											
      while(1);                       // do not load, trap CPU!!	
    }
    DCOCTL = 0;                       // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;            // Set DCO
    DCOCTL = CALDCO_1MHZ;
    UARTArrPortSel |= BIT1 + BIT2;    // Select peripheral function for UART pins
    UARTArrPortSel2 |= BIT1 + BIT2;   // Select secondary peripheral function for UART pins
    UARTArrPortDir |= BIT1 + BIT2;    // Set UART pins as outputs
    UARTArrPortOUT &= ~(BIT1 + BIT2); // Clear output bits 
    UCA0CTL1 |= UCSSEL_2;             // SMCLK as clock source
    UCA0BR0 = 104;                    // Set baud rate to 9600 (assuming 1 MHz clock)
    UCA0MCTL = UCBRS0;                // Set modulation  
    UCA0BR1 = 0x00;                   // baud rate 9600 
    UCA0CTL1 &= ~UCSWRST;             // Initialize USCI 
    IE2 |= UCA0RXIE;                  // Enable USCI_A0 RX interrupt
}

//------------------------------------------------------------------------------
//                             Flash configuration
//------------------------------------------------------------------------------
            
void FlashConfig(){
    // WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer
    //  if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
    //  {
    //    while(1);                               // do not load, trap CPU!!
    //  }
    //  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    //  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
    //  DCOCTL = CALDCO_1MHZ;
     FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
}



           
             

 
             
             
            
  

