#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer

enum FSMstate state;
enum SYSmode lpm_mode;


void main(void){
  
  state = state0;  // start in idle state on RESET
  lpm_mode = mode0;     // start in idle state on RESET
  sysConfig();
  
  while(1){
	switch(state){
	  case state0:
        enterLPM(lpm_mode);
		break;
		 
	  case state1:   //Object Detector
		st1();
		break;
		 
	  case state2:  //Telemeter 
		st2();
		break;

	  case state3:	//Light Source Detector
		st3();
	  	break;

	  case state4:
      	st4();     //bonus - Light Source Detector and Object Detector
		break;

	  case state5:
      	st5();    //send files from PC to MCU
		break;

	 case state6:
      	st6();      //calibration of LDR state
		break;

	 case state7:
      	st7();      //reset file memory flash segments
		break;

	 case state8:	//read text files (show on LCD)
		st8();
		break;

	 case state9:	//run script files 
		st9();
		break;
		
	 case state10:	//sync LDR0,LDR1 values from flash to PC Side
		st10();
		break;
	}
  }
}
  
  
  
  
  
  