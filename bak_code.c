/*****************************************************/	
/************* STEP4: get chip temperature************/		
/*****************************************************/			 
/*     T3CLRI = 0xAA;      //feed dog			 
		 delay(1000);
		 ADCCP   = 0x10;				// conversion on temperature
		 delay(1000);
		 ADCCON  = 0x06E3;       // ADC Config: fADC/2, acq. time = 8 clocks => ADC Speed = 1MSPS  single conversion   0x6E3  
		 ADCCON &= 0xFF7F;      // clear enable
		 while (!ADCSTA)        // wait for end of conversion
		 {
			 delay(100);
		 }
			b = (ADCDAT >> 16);		// To calculate temperature  use the formula:
			a = 0x525 - b;				// ((Temperature = 0x525 - Sensor Voltage) / 1.3)	
      if(a>0)  
      {
				sign = 0x0;
			}
      else
      {
				sign = 0x1;
			}
			a /= 1.3;		
      a = 10*a;			 
		  b = floor(a);		
      if(sign == 0x0)	
			{
				  Temperature = b;
			}
			else
			{
					 b  = 0xFFFF - b;
				   b |= 0x8000;
				  Temperature = b ;
			}  */

/*****************************************************/	
/************* STEP6: get NTC value      *************/		
/*****************************************************/			 
/*		 T3CLRI = 0xAA;      //feed dog						
		 delay(1000);
		 ADCCP   = 0x00;				// conversion on ADC0
		 sum = 0;
		 for(i=0;i<8;i++)
		 {
			 delay(1000);
			 ADCCON  = 0x06E3;       // ADC Config: fADC/2, acq. time = 8 clocks => ADC Speed = 1MSPS  single conversion
			 ADCCON &= 0xFF7F;      // clear enable
			 while (!ADCSTA)        // wait for end of conversion
			 {
				 delay(100);
			 }
			 ADC4_Result[i] = (ADCDAT >> 16);	
		   sum +=ADC4_Result[i];
		 }		
		 avg4 = sum/8;  */  