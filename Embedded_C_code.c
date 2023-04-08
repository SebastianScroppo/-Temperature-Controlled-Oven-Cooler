#include "io430.h"
#include "math.h"


#define ON 1 
#define OFF 0 
#define DELAY 20000
#define ASCII_CR 0x0D 
#define ASCII_LF 0x0A
#define BUTTON P1IN_bit.P3

#define GREEN_LED P1OUT_bit.P0 
#define RED_LED P1OUT_bit.P6

#define NPOINTS 200
float d_temp = 0;
float accum_error = 0;
//-------------------------------------------------------- 
//GlobalVariables 
//----------------------------------------------------'
int v[200];





//-------------------------------------------------------- 
//Miscellaneous Functions: 

void delay (unsigned long d)
{
  while (d--);
}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
    GREEN_LED = OFF;
    P1IFG_bit.P3 = 0;    // clear the interrupt request flag 
} 


//-------------------------------------------------------- 
//UART Module 
//--------------------------------------------------------

void Init_UART(void)
{ 

//initialize the USCI
//RXD is on P1.1 
//TXD is on P1.2

//configure P1.1and P1.2 for secondary peripheral function
P1SEL_bit.P1 = 1; 
P1SEL2_bit.P1 = 1; 
P1SEL_bit.P2 = 1; 
P1SEL2_bit.P2 = 1;

// divide by  104 for 9600b with  1MHz clock
// divide by 1667 for 9600b with 16MHz clock
// divide by  9 for 115200b with 16MHz clock

UCA0BR1 = 0; 
UCA0BR0 = 9;

// use x16 clock
UCA0MCTL_bit.UCOS16 = 1;

//select UART clock source 
UCA0CTL1_bit.UCSSEL1 = 1; 
UCA0CTL1_bit.UCSSEL0 = 0;

//release UART RESET 
UCA0CTL1_bit.UCSWRST = 0;
}

unsigned char getc(void)
{
  while (!IFG2_bit.UCA0RXIFG);
  return (UCA0RXBUF);
}

void putc( int c)
{
  while (!IFG2_bit.UCA0TXIFG);
  UCA0TXBUF = c;
}

void puts(char *s)
{
  while (*s) putc(*s++);
}

void newline(void)
{
  putc(ASCII_CR);
  putc(ASCII_LF);
}

void itoa(unsigned int n)
{
  unsigned int i;
  char s[6] = "    0";
  i = 4;
  while (n)
  {
    s[i--] = (n % 10) + '0';
    n = n / 10;
  }
  puts(s);
}


//-------------------------------------------------------- 
//ADCModule 
//--------------------------------------------------------
void Init_ADC(void) // TO COMPLETE
{
//initialize 10-bit ADC using input channel 4 on P1.4 
  
ADC10CTL1 = INCH_4 + CONSEQ_2; 


ADC10AE0 |= BIT4; // enable analog input channel 4

//select sample-hold time, multisample conversion and turn on the ADC
ADC10CTL0 |= ADC10SHT_0 + MSC + ADC10ON;

// start ADC
ADC10CTL0 |= ADC10SC + ENC;

 
}

void Sample(int n) // TO COMPLETE
{
// Write a simple program to enable the MCU to digitize an analog input signal by
// reading the content of the ADC register in a loop. 
  int i = 0;
  while (i<n-1) {
    v[i]=ADC10MEM>>2;
    i+=1;}
}

void Send(int n) // TO COMPLETE
{
// Write a simple program to enable the MCU to to transmit the sampled data
// via the USB interface.
  int i = 0;
  while(i<n-1) {
    putc(v[i]);
    i+=1;
  }



}

//-------------------------------------------------------- 
//Initialization 
//--------------------------------------------------------

void Init(void) 
{ 
//Stop watchdog timer to prevent timeout reset 
WDTCTL = WDTPW + WDTHOLD;

DCOCTL = CALDCO_16MHZ; 
BCSCTL1 = CALBC1_16MHZ;

P1REN = 0x08; //enable output resistor 
//P1OUT = 0x08; //enable P1.3 pullup resistor 
//P1DIR = 0x41; //setup LEDs as output 
P1IE_bit.P3 = 1; //enable interrupts on P1.3 input
}
void PWM_control(void)
{
P1DIR |= BIT6; //Set pin 1.6 to the output direction.
P1SEL |= BIT6; //Select pin 1.6 as our PWM output.

TA0CCTL1 = OUTMOD_7;
TA0CTL = TASSEL_2 + MC_1; //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.
P2DIR |= BIT2;
}


void PWM (float set_temp, float kp, float ki, float kd) {
double x = ADC10MEM>>2;
float current_temp = -9*pow(10,-6)*pow(x,3)+0.0043*pow(x,2)-(0.8951*x)+74.794;  //convert digital volatge to temperature
float error = (set_temp-current_temp);

accum_error+=error;

float P_term = kp*error;
float I_term = ki*accum_error;


float D_term = kd*(d_temp-error);
d_temp = error;
//float d=500;
float d = (P_term+I_term+D_term);
if(d<0)
{
  d=-d;   //duty cycle is always positive
}
if (d>1000)   
{
  d=1000;             //max duty cycle is 1000
}
/*if (d=0)
{
  d=10;
}
*/
TA0CCR0 = 1000; //Set the period in the Timer A0 Capture/Compare 0 register to 1000 us.



if(error>0)   //heating
{
  d=1001-d;          //inverted PWM for heating 
  P2OUT_bit.P2=1;
}
if(error<0)  //cooling
{
  P2OUT_bit.P2=0;
}
TA0CCR1 = d;  //set duty cycle 

}
float* get_parameters(void)
{
static float m[4];
 m[0]=(float)getc();
 m[1]=(float)getc();
 m[2]=(float)getc();
 m[3]=(float)getc();
return (m);
}

void main(void) 
{
    Init(); 
    Init_UART(); 
    Init_ADC();
    PWM_control();
    
    float* a = get_parameters();
    float temp = a[0];
    float P = a[1];
    float I =  a[2];
    float D =  a[3]; 
    
while(1) 
{
    GREEN_LED = ON; 
    //putc(ADC10MEM>>2); 
   
    GREEN_LED = OFF;  
    PWM(temp,P,I,D);
    //Sample(200);
    //Send(200);
    putc(ADC10MEM>>2); 
} 

}
