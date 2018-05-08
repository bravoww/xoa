/***************************************************************************

 Author        : ADI - Apps                    www.analog.com/MicroConverter

 Date          : May. 2007

 File          : I2C_Slave.c

 Hardware      : Applicable to ADuC702x rev H or I silicon
                 Currently targetting ADuC7028.

 Description   : I2C Slave to demonstrate with I2C_Master.
				 
				 Operates in two modes, read & write (called recieve and 
				 transmit here). At the begining of an I2C transmission, the 
				 Master sends an address. The LSB of this address determines 
				 if the Master is to read (1) or write (0).
//Master clock = 41.78Mhz    T = 24ns    delay(42) = 1us  delay(100) = 4us
//button1:Adjust input(make sure Pin = -15dbm)
//button2:Adjust output
//add auto Adjust:
//step1:Make sure input = -15dbm;
//step2:Make sure button is pushed;
//step3:Make sure output = 5dbm;(IIC cmd:FF 87 88 88 )
//step4:Auto Adjust(IIC cmd:FF 88)
***************************************************************************/

#include<ADuC7020.h> 
#include<math.h>
#define  u8    unsigned char
#define  u16   unsigned short int
#define  u32   unsigned int
#define  s16   short int
#define  DACMAX   3250  // 2.5*(2048/4096)= 1.25v  2250=300mA
#define  _24C02_ADD_WR   0xA0
#define  _24C02_ADD_RD   0xA1
#define  V_0             500     //Voltage(mv) at 0℃
#define  Tc              1       //Voltage(mv) vs Temparature(℃) (actual Temperature Coefficient / 10) 
//#define SDA_IN()      {GP1DAT &= 0xF7FFFFFF;}    //set input  on  P1.3(SDA)  
//#define SDA_OUT()     {GP1DAT |= 0x08000000;}    //set output on  P1.3(SDA)  
//#define IIC_SDA=1     {GP1SET  = 0x00080000;}    //SDA=1
//#define IIC_SDA=0     {GP1CLR  = 0x00080000;}    //SDA=0
//#define IIC_SCL=1     {GP1SET  = 0x00040000;}    //SCL=1
//#define IIC_SCL=0     {GP1CLR  = 0x00040000;}    //SCL=0

u8 indx,j,Update;
u8 status;      //lst = 1 Enable LD; lst = 0 Disable LD
u8 data[8];
u16 ADC0_Result[8],ADC1_Result[8],ADC2_Result[8],ADC3_Result[8];
u16 APC,Power_i,Power_o,Current;//  APC:target APC Power
u32 sum ;
u16 avg0,avg1,avg2,avg3 ;
float a;
s16 b,Temperature;
u16  Temp;
//double Res_o = 60;          //Output sample Resistor  60k    can be combined with Coe_o
//double Res_i = 180;         //Input  sample Resistor 180k    can be combined with Coe_i
double Rth  = 1.0;          //Current sample Resistor  1.0R
//double Coe_o = 0.85;        //PD effiency (A/W)
//double Coe_i = 0.85;        //PD effiency (A/W)
double Coe = 1.0;           //Adjust to comply with PID.SetPoint  normally should be 1.0
//double TAP_i = 17.656;      //input  TAP offset(dB)
//double TAP_o = 14.0;        //output TAP

//Pin = -6.6635dbm(0.2156mw)  v = 1.332
//COE_i = v/Pin = 1.332/0.2156 = 6.178
double COE_i = 5.50;      //total input coe
//Pout = 8.517dbm(7.108mw)  v = 1.497v
//COE_o = v/Pout = 1.497/7.108 = 0.2106
double COE_o = 0.0766;      //total output coe

typedef struct PID { 
double SetPoint;      // Desired value 
double Proportion;    // Proportional Const 
double Integral;      // Integral Const 
double Derivative;    // Derivative Const 
double LastError;     // Error[-1] 

double PrevError;    // Error[-2] 
double SumError;    // Sums of Errors 
} PID; 

void My_IRQ_Handler(void);
void ADCPoweron(int);
void delay(int);
double dbm2mw(double);
u16 mw2dbm(double);
u16 Getdbm_input(u16 x);
u16 Getdbm_output(u16 x);
u16 Getcurrent(u16 x);
double PIDCalc( PID *pp, double NextPoint );
void PIDInit (PID *pp); 
void SetDAC(double rDelta);
void SetPdbm(u16 x,PID *pp);
void WriteEEPROM(u8 add, u16 data);
u16 ReadEEPROM(u8 add);
u16 GetTemp(u16 x);

void IIC_Start(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);		    
void IIC_NAck(void);			  
void IIC_Send_Byte(u8 txd); 
u8 IIC_Read_Byte(unsigned char ack);
u8 AT24CXX_ReadOneByte(u8 ReadAddr);
void AT24CXX_WriteOneByte(u8 WriteAddr,u8 DataToWrite);

int main()
{
	u8  pin;
	u8  i;
  u8  sign = 0;      //temperature > 0
	u16 tmp;
	PID sPID; 				// PID Control Structure 
	double rOut; 			// PID Response (Output) 
	double rIn; 			// PID Feedback (Input) 
	double rDAC = 0;      // Last DAC Value 
	PIDInit ( &sPID ); // Initialize Structure 	
	APC = 50;          // overide sPID.SetPoint;
	
	POWKEY1 = 0x01;
	POWCON  = 0x00;		   			// 41.78MHz
	POWKEY2 = 0xF4;
	
	// I2C on P1.0/P1.1/P1.2/P1.3 
 	  GP1CON = 0x22;
	//GP1CON = 0x2222;
	
	// Set IO Direction
	GP4DAT |= 0x04000000;    //set output on P4.2 (A1@MAX4734)	
	GP2DAT |= 0x01000000;    //set output on P2.0 (Big or Small feedback@MAX4644)	
	GP1DAT |= 0x6C000000;    //set output on P1.2 (SCL) P1.3(SDA)  P1.5(En_MAX4734/NR?)  P1.6(INT) 
	GP0DAT |= 0x80000000;    //set output on P0.7 (LOS)
	
  //default:APC Mode  +  Big feedback + Disable LD
	GP4CLR  = 0x00040000;    //P4.2=A1=0(APC Mode)
	GP2CLR  = 0x00010000;    //Big   feedback	
  GP1SET  = 0x000C0000;    //SDA=1 SCL=1

//GP2SET  = 0x00010000;    //Small feedback		
//GP4SET  = 0x00040000;    //A1=1(ACC Mode)
//GP1SET  = 0x00040000;    //A0=1(ACC Mode)	
//GP4CLR  = 0x00040000;    //A1=0(APC Mode)
//GP1CLR  = 0x00040000;    //A0=0(APC Mode)	


	IRQ=My_IRQ_Handler;
	IRQEN = 0x200;					// I2C0 Slave Interupt
	
	I2C0CFG = 0x01;		  		// Slave Enable
	I2C0ID0 = 0xA0;					// Slave ID
	I2C0STX = 0xFF;	        // Full IIC FIFO
	I2C0STX = 0xFF;         // Full IIC FIFO
	I2C0STX = 0xFF;	        // Full IIC FIFO
	I2C0STX = 0xFF;         // Full IIC FIFO
	
	// I2C-Master setup
//	I2C1CFG = 0x82;		  			// Master Enable & Enable Generation of Master Clock
//	I2C1DIV = 0x283C;				  // 0x283C = 400kHz  0xCFCF = 100kHz
	
	// ADC and DAC configuration
  ADCPoweron(2000);		// Power on ADC		
	DAC0CON = 0x12;				  // 0-VREF(2.5V)  AGND-AVDD range
	DAC0DAT = 0x00000000;
	REFCON  = 0x01;				  // 0x01 = internal 2.5V reference    0x00 = external 2.5V  -wrong

 	indx = 0;
	i = 0;
	j = 0;
	Update = 0;	
	status  = 0x03;	
	pin = 0;
	sum = 0;
	avg0= 0;
	avg1= 0;
	avg2= 0;
	for(i=0;i<8;i++)
	{
		ADC0_Result[i]=0;
		ADC1_Result[i]=0;
		ADC2_Result[i]=0;
	}
/*********	NEVER TURN IT ON!!!  **********/
/*********	NEVER TURN IT ON!!!  **********/
/*********	NEVER TURN IT ON!!!  **********/
if((GP0DAT&0x00000010) == 0x0)  //read P0.4
	{
		T3CON  = 0x0;    	  		    // Disable watchdog 
	}
else
	{
    T3CLRI = 0xAA;
    T3LD   = 0x1000;			  		// 0x1000/32768 = 125ms
    T3CON  = 0xE0;    	  		  // IRQ instead of reset, can NOT be 0xF0		
	}

	while (1)                    //loop time = 4.5ms
	{ 		
		  AT24CXX_WriteOneByte(0x00, 0x19);
			APC = AT24CXX_ReadOneByte(0x00);
		
		 if(pin)  
		 {
			 pin = 0;
			 GP2CLR = 0x00010000;    //Big   feedback
		 }
		 else
		 {
			 pin = 1;
			 GP2SET = 0x00010000;    //Small feedback	
		 }
     T3CLRI = 0xAA;      //feed dog
		 indx = 0;

		 if(status == 0x03)	
		 {
		 //  GP1CLR |= 0x00080000;    //Enable LD		
			 delay(8000);
		 }
		 else	
		 {			 
			 //GP1SET |= 0x00080000;    //Disable LD
			 DAC0DAT = 0x00000000;    //Clear DAC
			 rDAC = 0;                // Last DAC Value 
			 PIDInit ( &sPID );       // Initialize Structure 
			 delay(8000);
		 }		 		 
		
		 delay(1000);
		 ADCCP   = 0x00;				// conversion on ADC0
		 sum = 0;
		 for(i=0;i<8;i++)
		 {			 
			 delay(1000);
			 ADCCON  = 0x17E3;       // ADC Config: fADC/2, acq. time = 8 clocks => ADC Speed = 1MSPS  single conversion
			 ADCCON &= 0xFF7F;      // clear enable	
			 while (!ADCSTA)        // wait for end of conversion
			 {
				 delay(100);
			 }		  
			 ADC0_Result[i] = (ADCDAT >> 16);	
			 sum +=ADC0_Result[i];
		 }		
		 avg0 = sum/8;
		 Power_o = Getdbm_output(avg0);				 
		 			  
     if(status == 0x03)		    //PID works when LD enabled
		 {		
			rIn  = avg0; //avg0;//ADC0_Result[0]; // Read Input 
			SetPdbm(APC,&sPID) ;
			rOut = PIDCalc (&sPID,rIn ); // Perform PID Interation 
			rDAC += rOut;
			SetDAC ( rDAC ); // Effect Needed Changes 	
		//SetDAC ( DACMAX ); // Effect Needed Changes 						 
		 }	
			
		
		 T3CLRI = 0xAA;      //feed dog	 
		 delay(1000);
		 ADCCP   = 0x01;				// conversion on ADC1
		 sum = 0;
		 for(i=0;i<8;i++)
		 {
			 delay(1000);
			 ADCCON  = 0x17E3;       // ADC Config: fADC/2, acq. time = 8 clocks => ADC Speed = 1MSPS  single conversion
			 ADCCON &= 0xFF7F;      // clear enable
			 while (!ADCSTA)        // wait for end of conversion
			 {
				 delay(100);
				}
			 ADC1_Result[i] = (ADCDAT >> 16);	
			 sum +=ADC1_Result[i];
		 }		
		 avg1 = sum/8;				 
     Power_i = Getdbm_input(avg1);
	// Power_i = 0x802d;
	// status  = 0x03;
		 if(Power_i&0x8000)
		 {
			  tmp = Power_i&0x7FFF;
			  if(tmp > 260)              //input < -30dbm
				{
					 status &= 0x01; 
				}
				else
				{
					 status |= 0x02;         //input > -30dbm
				}
		 }
		 else
		 {
			     status |= 0x02;         //input > -30dbm
		 }
		
		 
		 T3CLRI = 0xAA;      //feed dog						
		 delay(1000);
		 ADCCP   = 0x02;				// conversion on ADC2
		 sum = 0;
		 for(i=0;i<8;i++)
		 {
			 delay(1000);
			 ADCCON  = 0x17E3;       // ADC Config: fADC/2, acq. time = 8 clocks => ADC Speed = 1MSPS  single conversion
			 ADCCON &= 0xFF7F;      // clear enable
			 while (!ADCSTA)        // wait for end of conversion
			 {
				 delay(100);
			 }
			 ADC2_Result[i] = (ADCDAT >> 16);	
		   sum +=ADC2_Result[i];
		 }		
		 avg2 = sum/8;		 			 
     Current = Getcurrent(avg2);		
	  /*
     if(status==0x03)		    //PID works when LD enabled
		 {		
			rIn  = ADC2_Result[0]; //avg0;//ADC0_Result[0]; // Read Input 		
			rOut = PIDCalc (&sPID,rIn ); // Perform PID Interation 
			rDAC += rOut;
			SetDAC ( rDAC ); // Effect Needed Changes 			 
		 }	
			*/
		 
     T3CLRI = 0xAA;      //feed dog			 
		 delay(1000);
		 ADCCP   = 0x10;				// conversion on temperature
		 delay(1000);
		 ADCCON  = 0x17E3;       // ADC Config: fADC/2, acq. time = 8 clocks => ADC Speed = 1MSPS  single conversion   0x6E3  
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
			}
			
			
		 T3CLRI = 0xAA;      //feed dog						
		 delay(1000);
		 ADCCP   = 0x0F;				// conversion on ADC15
		 sum = 0;
		 for(i=0;i<8;i++)
		 {
			 delay(1000);
			 ADCCON  = 0x17E3;       // ADC Config: fADC/2, acq. time = 8 clocks => ADC Speed = 1MSPS  single conversion
			 ADCCON &= 0xFF7F;      // clear enable
			 while (!ADCSTA)        // wait for end of conversion
			 {
				 delay(100);
			 }
			 ADC3_Result[i] = (ADCDAT >> 16);	
		   sum +=ADC3_Result[i];
		 }		
		 avg3 = sum/8;	  		 
     Temp = GetTemp(avg3);
	} 
}


/*************************************************/
/*************************************************/
/************	IRQ Service Routine  *************/
/*************************************************/
/*************************************************/


void My_IRQ_Handler()
{
	int k;
	u8  data_tmp;
  T3CLRI = 0xAA;      //feed dog
// Slave Recieve
	if ((I2C0SSTA & 0x08)==0x08)   // Slave Recieve IRQ
	{	 
		  data[indx] = I2C0SRX;         //data[0]=Subaddress(1B)  data[1:2]=Content
		  indx++;
		  if(indx > 2) 	indx = 0;					
		  if(data[0] == 0xFF) 
			{
				data_tmp = 0x01 & data[2];
				if(data_tmp == 0x01)
				{
					status |= 0x01;
				}
				else
				{
					status &= 0x02;
				}
			}
			else if(data[0] == 0xFE)
			{
				APC  = data[1] ;
				APC  = APC << 8;
				APC |= data[2] ;       			
			}
	}
	// Slave Transmit
	 if ((I2C0SSTA & 0x04)==0x04)   // Slave Transmit IRQ
	{
		indx = 0;                    //clear?
		if(I2C0SRX == 0x00)          //1. Send SN:7601 (7:2017 / 7:July / 01:serial number)
		  {		
			  I2C0STX = 0x77;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  				
				I2C0STX = 0x01;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          } 
			}   
		else if(I2C0SRX == 0x01)    //2.ADC0 - Output Power_Out
			{		
			  I2C0STX = ((Power_o & 0xFF00) >> 8);
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
				I2C0STX = Power_o;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
			} 			
		else if(I2C0SRX == 0x02)    //3.ADC1 - Input Power
			{		
			  I2C0STX = ((Power_i & 0xFF00) >> 8);
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
				I2C0STX = Power_i;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
			}	
		else if(I2C0SRX == 0x03)   //4.ADC2 - Driver Current
			{		
			  I2C0STX = ((Current & 0xFF00) >> 8);
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
				I2C0STX = Current;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
			}		
		else if(I2C0SRX == 0x04)   //5.Sensor(Temperature)  OR  AIR(Temp) 
			{		
			  I2C0STX = ((Temp & 0xFF00) >> 8);   //Temperature
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
				I2C0STX = Temp;  //Temperature
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
			  } 
    else if(I2C0SRX == 0x05)  //6.XOA status
			{		
			  I2C0STX = 0x00;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
				I2C0STX = status;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
			  }   
	/*	else if(I2C0SRX == 0x06)  //6.debug avg1 status
			{		
			  I2C0STX = ((avg1 & 0xFF00) >> 8);
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
				I2C0STX = avg1;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }  
			  } 	*/	
     else
		 {
			 I2C0STX = 0x00;
				k = 100;
				while(((I2C0SSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
					{
            k--;
          }    
		 }	
	}	
}

void ADCPoweron(int time)
{
	ADCCON = 0x620;	 					// Power_Out-on the ADC
	while (time >=0)	  				// wait for ADC to be fully Power_Outed on
   time--;
}

void delay(int length)
{	
	while (length >=0)	  				
    length--;
}

//mw = 10^(dbm/10)
double dbm2mw(double x)
{
	 double y;
   y = x/10;
   return pow(10,y);	
}

//dbm = 10logmw
//most significant bit = 1: Pdbm < 0;
//most significant bit = 0: Pdbm > 0;
u16 mw2dbm(double x)
{
	double y,z;
	u16 t;
	if(x > 0)
	{
		y = 10*(log10(x));
		z = 10*y;   //amplify by 10x	
		if(z>=0)
		{
			t = (u16)z;
			t &= 0x7FFF;  //set most significant bit = 0
		}
		else
		{
			t = (u16)(-z);
			t &= 0x7FFF;
			t |= 0x8000;  //set most significant bit = 1
		}		
	}	
	else
	{
		t = 0xFFFF;
	}
	return t;	
}

u16 Getdbm_input(u16 x)
{
	double y,z;
	y = ((double)x/4096)*2.5;
	z = y/(COE_i);
  return (mw2dbm(z));
}
u16 Getdbm_output(u16 x)
{
	double y,z;
	y = ((double)x/4096)*2.5;
	z = y/(COE_o);	
  return (mw2dbm(z));
}

u16 Getcurrent(u16 x)
{
	double y,z;
	u16 t;
	y = (x*2.5)/4096;
	z = y/2;          //opa2317 amplify 2x
	y = (z/Rth)*1000;
	t = (u16) y;
	return t;
}

/*==================================================================================================== 
						PID Algorithm
=====================================================================================================*/ 
double PIDCalc( PID *pp, double NextPoint ) 
{ 
	double dError, Error; 
	Error = pp->SetPoint - NextPoint;           // derivate
	pp->SumError += Error;                      // sum
	dError = Error - pp->LastError;             // delta
	pp->PrevError = pp->LastError; 
	pp->LastError = Error; 
	return ((pp->Proportion )* Error 	+ (pp->Integral )* (pp->SumError) + (pp->Derivative) * dError ); 
} 
/*==================================================================================================== 
			Initialize PID Structure  
=====================================================================================================*/ 
//1. 50/0.1/0
//2. 1.0/0.05/0.05
//3. 1.0/0.1/0.03    ?
//4. 0.1/0.01/0.03
void PIDInit (PID *pp) 
{ 
//	memset(pp,0,sizeof(PID));    //failed ?
	pp->SetPoint   = 330;     // Desired value  //327; 
	pp->Proportion = 0.50;    // Proportional Const   1.0
	pp->Integral   = 0.50;      // Integral Const       
	pp->Derivative = 0.10;    // Derivative Const 
	pp->LastError  = 0;     // Error[-1]
	pp->PrevError  = 0;    // Error[-2] 
	pp->SumError   = 0;    // Sums of Errors 
} 
/*==================================================================================================== 
				SetDAC Function 
=====================================================================================================*/ 
void SetDAC(double rDelta) 
{
	u16 i;
	if (rDelta < 0)
		 i = 0;
	else if(rDelta > DACMAX)
		 i = DACMAX;
	else
	   i = (u16) rDelta;	
	DAC0DAT = i << 16;
}
/*==================================================================================================== 
		Set Output Power Function 
1. mw * Coe(A/W) * Res_o(xxk) = mw * COE_o = v
=====================================================================================================*/ 
void SetPdbm(u16 x,PID *pp)
{ 
	double y,z;
	if((x&0x8000) == 0x8000)
		y = -(x & 0x00FF);
	else					   
		y =   x & 0x00FF; 
	if((y>=0) && (y<=100))     //power = (0dbm,10dbm)
	{
		y = dbm2mw(y/10);
	  z = COE_o * y;	  
	  y = (z*4096)/2.5;	
    z = y * Coe;
	  pp->SetPoint = z;
	}
}

void WriteEEPROM(u8 add, u16 data)
{
	 int k;
	/* I2C1MTX = _24C02_ADD_WR;
	 k = 100;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   } */
	 I2C1ADR = 0xA0;
		 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
	 I2C1MTX = 0x11;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
	 I2C1MTX = 0x66;//(data & 0xFF00) >> 8;  
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
// I2C1MTX = data;
	 delay(100);
/*	 I2C1MTX = _24C02_ADD_WR;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }*/
/*	 I2C1ADR = 0xA0;
	 I2C1MTX = add+1;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }*/
// I2C1MTX = (data & 0xFF00) >> 8;  
/*	 I2C1MTX = 0x66;//data;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }*/
}	

u16 ReadEEPROM(u8 add)
{
	u16 data;
	int k;
/*	I2C1MTX = _24C02_ADD_RD;
	while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }*/
	I2C1ADR = 0xA1;
		 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
	I2C1MTX = add;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
	 while(((I2C1MSTA & 0x08) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
	data = I2C1MRX;
//	data = data << 8;
//	delay(100);
/*	I2C1MTX = _24C02_ADD_RD;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }*/
/*	I2C1ADR = 0xA0;
	I2C1MTX = add+1;
	 while(((I2C1MSTA & 0x04) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
	 while(((I2C1MSTA & 0x08) == 0x00) && (k > 0))  //Wait for end of sending
	 {
     k--;
   }
	data |= I2C1MRX;*/
	return data;
}

u16 GetTemp(u16 x)          //Result = 10 * Temperature
{
	double y,z;
	u16 t;
	y = (x*2.5*1000)/4096;
	z = (y-V_0)/Tc;           //Tc = (actual Temperature Coefficient / 10)  apmlify by 10x
  if(z<0)
	{
		y = -z;        
		t = (u16) y;
		t |= 0x8000;
	}
	else
	{
		t = (u16) z;
	}
	return t;
}


//产生IIC起始信号
//START:when CLK is high,DATA change form high to low
void IIC_Start(void)
{
	GP1DAT |= 0x08000000;       //sda线输出
	GP1SET  = 0x000C0000;	  	  //SDA=1 SCL=1
	delay(100);
	GP1CLR  = 0x00080000;	     //SDA=0
	delay(100);
	GP1CLR  = 0x00040000;//钳住I2C总线，准备发送或接收数据                        //SCL=0
}	  
//产生IIC停止信号
//STOP:when CLK is high DATA change form low to high 
void IIC_Stop(void)
{
	GP1DAT |= 0x08000000;    //sda线输出
	GP1CLR  = 0x000C0000;    //SDA=0 SCL=0 	
 	delay(100);
	GP1SET  = 0x00040000;                           //SCL=1
	delay(50);
	GP1SET  = 0x00080000;//发送I2C总线结束信号      //SDA=1
	delay(100);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	GP1DAT &= 0xF7FFFFFF;   //SDA设置为输入 
	GP1SET  = 0x00080000;   //SDA=1
	delay(25);	
	GP1SET  = 0x00040000;   //SCL=1
	delay(25);	 
	while((GP1DAT&0x00000008)==0x00000008) 
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	GP1CLR  = 0x00040000;   //时钟输出0 	     //SCL=0
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	GP1CLR  = 0x00040000;   //SCL=0
	GP1DAT |= 0x08000000;   //sda线输出
	GP1CLR  = 0x00080000;   //SDA=0
	delay(50);
	GP1SET  = 0x00040000;   //SCL=1
	delay(50);
	GP1CLR  = 0x00040000;   //SCL=0
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	GP1CLR  = 0x00040000;   //SCL=0
	GP1DAT |= 0x08000000;   //sda线输出
	GP1SET  = 0x00080000;   //SDA=1
	delay(50);
	GP1SET  = 0x00040000;   //SCL=1
	delay(50);
	GP1CLR  = 0x00040000;   //SCL=0
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  GP1DAT |= 0x08000000;  //sda线输出    
    GP1CLR  = 0x00040000;  //拉低时钟开始数据传输    //SCL=0
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			GP1SET  = 0x00080000;       //SDA=1
		else
			GP1CLR  = 0x00080000;       //SDA=0
		txd<<=1; 	  
		delay(50);  
		GP1SET  = 0x00040000;         //SCL=1
		delay(50); 
		GP1CLR  = 0x00040000;	        //SCL=0
		delay(50);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	GP1DAT &= 0xF7FFFFFF;   //SDA设置为输入
    for(i=0;i<8;i++ )
	{
        GP1CLR  = 0x00040000;      //SCL=0
        delay(50);
		    GP1SET  = 0x00040000;      //SCL=1
        receive<<=1;
        if((GP1DAT&0x00000008)==0x00000008) receive++;   
		   delay(25); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

u8 AT24CXX_ReadOneByte(u8 ReadAddr)
{				  
	u8 temp=0;		  	    																 
  IIC_Start();  
	IIC_Send_Byte(_24C02_ADD_WR);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);  //发送地址
	IIC_Wait_Ack();		
	IIC_Start();  	 	   
	IIC_Send_Byte(_24C02_ADD_RD);           //进入接收模式			   
	IIC_Wait_Ack();	 
  temp=IIC_Read_Byte(0);		   
  IIC_Stop();//产生一个停止条件	    
	return temp;
}

void AT24CXX_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
  IIC_Start();  
	IIC_Send_Byte(_24C02_ADD_WR);	    //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//发送高地址
 	IIC_Wait_Ack();	   
  IIC_Send_Byte(DataToWrite);     //发送字节							   
	IIC_Wait_Ack();  		    	   
  IIC_Stop();//产生一个停止条件 
	delay(42000);	  //1ms
	delay(42000);   //1ms
	delay(42000);   //1ms
	delay(42000);   //1ms
	delay(42000);   //1ms
}
