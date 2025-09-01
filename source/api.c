#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"


int st1_counter_int=0;
char st1_counter_str[17];
int LDR0_val=0,LDR1_val=0;
int LDR0_calib=0,LDR1_calib=0;
char i_str_st6[3];
int relevant_file_index[10];
int chosen_file_index;
//-------------------------------------------------------------
//            state1 function
//-------------------------------------------------------------
void st1(){
	DegScan(0,180,1);  //scan ultrasonic mode
	state=state0;
}
//--------------------------------------------------------------------
//            state2 function
//--------------------------------------------------------------------            
void st2(){
	command_flag = 0; // wait for data 
	LPM0; //wait for telemeter angle from PC Side
	while (state==state2)
	{
		DegScan(telemeter_angle,telemeter_angle+1,1);   //scan ultrasonic mode
	}
}
//--------------------------------------------------------------------
//            state3 function
//--------------------------------------------------------------------            
void st3(){
	DegScan(0,180,2);  //scan LDR mode
	state=state0;
}
//--------------------------------------------------------------------
//            state4 function
//--------------------------------------------------------------------            
 void st4(){
	DegScan(0,180,3);  //scan ultrasonic and LDR mode
	state=state0;
}
//--------------------------------------------------------------------
//            state5 function
//--------------------------------------------------------------------      
void st5(){
	fileIndex = *FilesInst.numFiles;
	if (fileIndex<0 || fileIndex>9){
		fileIndex = 0;
	}
	if (fileIndex == 10){
		lcd_clear();
		lcd_home();
		lcd_puts("flash is full");
		return;
	}
	if(fileIndex == 0){
		write_with_addr_flash_int(0xF600,FilesInst.filesLocation[0]);   //set file 0 address to 0xF600 
		disable_flash_write();
	}
	else{
		write_with_addr_flash_int(*FilesInst.filesLocation[fileIndex-1] + *FilesInst.filesSize[fileIndex-1], FilesInst.filesLocation[fileIndex]); //set file i address
		disable_flash_write();
	}
	SetFileStage = 0;
	command_flag = 0; 
	get_file_done = 0;
	while(get_file_done == 0){
		LPM0; //recieving file using UART and saving to flash
		send_UART_ack('a');
	}
	init_flash_write(FilesInst.numFiles); //init relevant segment
	write_with_addr_flash_int(fileIndex+1,FilesInst.numFiles); //add +1 to file number
	disable_flash_write();
	//num of files ++, ACK
	command_flag = 1;   
	state = state0;
	
}
//--------------------------------------------------------------------
//            state6 function
//--------------------------------------------------------------------      
void st6(){
	ServoTimerCfg(1);  //1 for big movement offset
	ServoDeg(90);      // set servo 90 degree
	int i=1;
	int* LDR0_calib_addr = 0x1000;
	int* LDR1_calib_addr = 0x1014;
	init_flash_write(0x1000);     //init segment D (LDR0,LDR1 calibration values will be stored there)
	for(i;i<=10;i++){
		lcd_clear();
		lcd_home();
		lcd_puts("Press PB0");
		lcd_new_line;
		sprintf(i_str_st6, "%d", 5*i);
		lcd_puts(i_str_st6);
		enablePB0_IE();
		LPM0;   //wait for PB0 press to capture
		disablePB0_IE();
		LDR0_calib = LDRSamp(0);
		LDR1_calib = LDRSamp(1);
		write_with_addr_flash_int(LDR0_calib, LDR0_calib_addr+(i-1));
		write_with_addr_flash_int(LDR1_calib, LDR1_calib_addr+(i-1));
		SendLightSamp(i, LDR0_calib, LDR1_calib);	
	}
	lcd_clear();
	state=state0;
}
//--------------------------------------------------------------------
//            state7 function
//--------------------------------------------------------------------      
void st7(){
	int i=0xFC00;
	for(i;i>=0xF200;i=i-0x200){   //reset segments 1 to segment 6
		init_flash_write(i);
	}
	state = state0;
}
//--------------------------------------------------------------------
//            state8 function
//--------------------------------------------------------------------      
void st8(){
	show_names_flag = 1; // 1 - choose file name, 0 - scroll specific file
	int j=0,num_of_text_files=0;
	for(j;j<*FilesInst.numFiles;j++){
		if (*(FilesInst.filesType[j])=='t'){
			relevant_file_index[num_of_text_files] = j;
			num_of_text_files++;
		}					
	}
	PB0_Counter = 0;   
	while(state==state8){
		switch(show_names_flag){
		case 1:
			lcd_clear();
			lcd_home();
			int i=0;
			for(i;i<16;i++){
				lcd_data(*(FilesInst.filesName[relevant_file_index[(PB0_Counter % num_of_text_files)]]+i));
			}
			lcd_new_line;
			i=0;
			for(i;i<16;i++){
				lcd_data(*(FilesInst.filesName[relevant_file_index[((PB0_Counter+1) % num_of_text_files)]]+i));
			}
			enablePB0_PB1();
			LPM0; //wait for PB0 / PB1 push or switch state
			disablePB0_PB1(); 
			chosen_file_index = relevant_file_index[(PB0_Counter % num_of_text_files)]; 
			int addr = *FilesInst.filesLocation[chosen_file_index];
			break;
		case 0:
			lcd_clear();
			lcd_home();
			int k;
			PB0_Counter = PB0_Counter % (*(FilesInst.filesSize[chosen_file_index]));
			for (k = 0; k < 16; k++){
				if(PB0_Counter+k > *FilesInst.filesSize[chosen_file_index]){
					break;
				}
				lcd_data(*((char*)addr + PB0_Counter + k)); 
			}
			lcd_new_line;
			for (k = 16; k < 32; k++){
				if(PB0_Counter+k > *FilesInst.filesSize[chosen_file_index]){
					break;
				}
				lcd_data(*((char*)addr + PB0_Counter + k)); 
			}
			enablePB0_PB1();
			LPM0; //wait for PB0 / PB1 push or switch state
			disablePB0_PB1(); 
			break;
		default:
			break;
		}
	}
	lcd_clear();
}
//--------------------------------------------------------------------
//            state9 function
//--------------------------------------------------------------------
void st9(){
	show_names_flag = 1; // 1 - choose file name, 0 - scroll specific file
	int j=0,num_of_text_files=0;
	for(j;j<*FilesInst.numFiles;j++){
		if (*(FilesInst.filesType[j])=='s'){
			relevant_file_index[num_of_text_files] = j;
			num_of_text_files++;
		}					
	}
	PB0_Counter = 0;   
	while(state==state9){
		switch(show_names_flag){
		case 1:
			lcd_clear();
			lcd_home();
			int i=0;
			for(i;i<16;i++){
				lcd_data(*(FilesInst.filesName[relevant_file_index[(PB0_Counter % num_of_text_files)]]+i));
			}
			lcd_new_line;
			i=0;
			for(i;i<16;i++){
				lcd_data(*(FilesInst.filesName[relevant_file_index[((PB0_Counter+1) % num_of_text_files)]]+i));
			}
			enablePB0_PB1();
			LPM0; //wait for PB0 / PB1 push or switch state
			disablePB0_PB1(); 
			chosen_file_index = relevant_file_index[(PB0_Counter % num_of_text_files)]; 
			int addr = *FilesInst.filesLocation[chosen_file_index];
			break;
		case 0:
			lcd_clear();
			lcd_home();
			lcd_puts("running file:");
			lcd_new_line;
			i=0;
			for(i;i<16;i++){
				lcd_data(*(FilesInst.filesName[chosen_file_index]+i));
			}
			DelayOf10Xms(50); 	//wait 0.5 sec 
			i=0;
			unsigned char OPC,arg1,arg2;
			addr = *FilesInst.filesLocation[chosen_file_index];  //address of first char of file
			for(i;i<*FilesInst.filesSize[chosen_file_index];){
				OPC = *((char*)addr + i);
				i++;
				switch (OPC){
					case 0x01:             //inc_lcd
						arg1 = *((char*)addr + i);
						i++;
						inc_lcd(arg1); 
						break;
					case 0x02:            //dec_lcd
						arg1 = *((char*)addr + i);
						i++;
						dec_lcd(arg1); 
						break;
					case 0x03:           //rra_lcd
						arg1 = *((char*)addr + i);
						i++;
						rra_lcd(arg1);
						break;
					case 0x04:			//set_delay
						arg1 = *((char*)addr + i);
						i++;
						d = arg1;	
						break;
					case 0x05:			//lcd_clear
						lcd_clear();	
						break;									
					case 0x06:			//servo_deg
						arg1 = *((char*)addr + i);
						i++;
						st9_deg_scan_flag = 1;
						send_UART_ack('d');   //d means servo_deg
						while (st9_deg_scan_flag == 1){
							DegScan(arg1,arg1+1,1);   //scan ultrasonic mode
						}	
						break;					
					case 0x07:			//servo_scan
						arg1 = *((char*)addr + i);
						i++;
						arg2 = *((char*)addr + i);
						i++;
						st9_servo_scan_flag = 1;
						start_servo_scan = 0;
						send_UART_ack('s');   //s means servo_scan
						while(start_servo_scan == 0){
							DelayOf10Xms(1);
						}
						DegScan(arg1,arg2+1,1);  //scan ultrasonic mode
						finish_servo_scan = 0;
						send_UART_ack('e');  // e means end servo_scan
						while(finish_servo_scan == 0){
							DelayOf10Xms(1);
						}
						st9_servo_scan_flag = 0;
						//ServoDeg(arg1);	
						break;
					case 0x08:		    //sleep
						state = state0;
						break;
					default:
						break;							
				}
			}	
			break;
		default:
			break;
		}
	}
	lcd_clear();
	state = state0;	
	send_UART_ack('f'); 		//f means finish state9
}
//--------------------------------------------------------------------
//            state10 function
//--------------------------------------------------------------------
void st10(){
	int i=1;
	int* LDR0_calib_addr = 0x1000;
	int* LDR1_calib_addr = 0x1014;
	for(i;i<=10;i++){
		LDR0_calib = *(LDR0_calib_addr+(i-1));	
		LDR1_calib = *(LDR1_calib_addr+(i-1));
		SendLightSamp(i, LDR0_calib, LDR1_calib);	
	}
	state=state0;
}
//--------------------------------------------------------------------
//            General functions
//--------------------------------------------------------------------    
void DegScan(int angle1,int angle2,int flag){
	int i=angle1;
	for (i; i < angle2; i++){
		if(i==angle1){
			ServoTimerCfg(1);  //1 for big movement offset
		}
		else{
			ServoTimerCfg(0);  //0 for small movement offset
		}
		ServoDeg(i);
		switch (flag){
			case 1:
					UltrasonicSamp();
					SendUltrasonicSamp(i);
					break;
			case 2:
					LDR0_val = LDRSamp(0);
					LDR1_val = LDRSamp(1);
					SendLightSamp(i, LDR0_val, LDR1_val);				
					break;
			case 3:
					UltrasonicSamp();
					LDR0_val = LDRSamp(0);
					LDR1_val = LDRSamp(1);
					SendUltrasonicAndLightSamp(i,LDR0_val, LDR1_val);
					break;
		}
	}
	//halt all timers
}









 
