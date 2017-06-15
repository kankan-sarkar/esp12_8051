
//******************************************************************************
//
//  ACLK = REFO = 32768Hz, MCLK = DCODIV = SMCLK = 8MHz.
//
//                MSP430FR2433
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |     
//            |                 |
//            |     P2.6/UCA1TXD|----> RX (ESP8266)
//            |     P2.5/UCA1RXD|<---- TX
//            |---------------- |
//   ver 1.0
//   Pradhan
//   Gill Instruments Inc.
//   oct 2016
//   Built with IAR Embedded Workbench v6.50 
//******************************************************************************

#include <msp430fr2632.h>
#include <string.h>
#define ROI 36
#define WATCH_TIME 120

#define uint8_t unsigned int
unsigned char  mqtt_message[127]; // byte array to store Mqtt packet
int MqttMessageLength = 0,RxAck=0,Rx_Buffer_Len=0,delay_cnt=0,del=WATCH_TIME;

#define CALADC_15V_30C  *((unsigned int *)0x1A1F)                 // Temperature Sensor Calibration-30 C
                                                                  // See device datasheet for TLV table memory mapping
#define CALADC_15V_85C  *((unsigned int *)0x1A2A)                 // Temperature Sensor Calibration-85 C

volatile long temp;
volatile long IntDegF;
volatile long IntDegC;
void clean(char *var);

unsigned int motor_stat=0,PAC_ID=0,er_net=0,state=0;
char test[10], Rx_Buffer[100];
void Modem_Config();
int Check(char *,char*,char*);
void Board_Init();
void Board_Init_115200();
int cmp_result=0,cmp_result1=0,cmp_result2=0;
void Init_GPIO();
void Delay(int);
void Tx_Uart(char * str);
void Tx_Uart_Char(char n);
void Mqtt_Connect(char * ,char * ,char * ,unsigned char *);
void Mqtt_Subscribe(unsigned char *, char *);
void Mqtt_Disconnect(unsigned char *);
void Mqtt_Publish(unsigned char *, char *,char *);
void Mqtt_Unsubscribe(unsigned char *, char *);
void Mqtt_Ping(unsigned char *);void Tcp_Init(char *, char *  ,int );
void Tcp_Packet(unsigned char * , int );
void Tcp_Length(int);
void Tcp_Disconnect();
void Tx_Integer(int);
void Int_To_String(char *,int);
void Int_To_String(char *,int);
void ON_OFF();
int Ping();
void Attention();
//==============================================================================

int main(void)
{ PAC_ID=0;  
   Board_Init(); 
    Delay(4);
 while(1)                              
 {                                     
   //P1OUT^=BIT0;
   switch(state)                      
   {
   case 1:
          PAC_ID=0;
          Mqtt_Connect("MQTTCLIENT","iot.eclipse.org","1883",mqtt_message);// Connect the module with broker using mqtt
          Delay(1);
          Mqtt_Subscribe(mqtt_message,"i_switch");
          Delay(1);
          state=2;
          break;
   
   case 2:
	  //Temperature=TempOut();
	  Int_To_String(test,IntDegC);       
          PAC_ID=0;                  // during ping searchfor \n\r
          Attention();               // flush buffefxdxr
          while(Ping());             // wait here till resolve network
          PAC_ID=1;                  // during PUB or SUB search for #
          Delay(5);
          Mqtt_Publish(mqtt_message,"Room_Temp",test);
          //Delay(5);
          if(motor_stat)
          Mqtt_Publish(mqtt_message,"Motor_stat","ON");
          else
          Mqtt_Publish(mqtt_message,"Motor_stat","OFF");   
          Delay(5);
        
          Mqtt_Ping(mqtt_message);    // Indicate broker that connection is alive
          Delay(20);
          break;
   
   
                    // follow case 3 when there is NO response from ping
   case 3:PAC_ID=0;
              
          Mqtt_Unsubscribe(mqtt_message,"i_switch");
          Delay(1);
          Mqtt_Disconnect(mqtt_message);
          Delay(1);
          state=1;
          break;
   default:state=1;
   }
  }                       
}
//=========================== END OF MAIN=======================================

  void Attention()  //
  {
    Tx_Uart("AT");
    Tx_Uart_Char('\r');
    Tx_Uart_Char('\n');
    __delay_cycles(10000);
   }


//------------------------------------------------------------------------------

void Board_Init_115200()
{
  WDTCTL = WDTPW | WDTHOLD;                // Stop watchdog timer
                         // Enable WDT interrupt
  // Configure UART 115200
  UCA1CTLW0 |= UCSWRST;
  UCA1CTLW0 |= UCSSEL__SMCLK;
  UCA1BR0 = 8;                     
  UCA1BR1 = 0x00;
  UCA1MCTLW = 0xD600;
  UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
  UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
   // Configure GPIO
   Init_GPIO();
  PM5CTL0 &= ~LOCKLPM5;                    // Disable the GPIO power-on default high-impedance mode
 WDTCTL = WDT_ADLY_1000;                   // WDT 32ms, SMCLK, interval timer
   SFRIE1 |= WDTIE;
_BIS_SR(GIE);
   // __bis_SR_register(GIE); 
  //_BIS_SR(LPM3_bits+GIE); 
}

void Board_Init()
{   Tx_Uart("AT+CIOBAUD=9600");
           WDTCTL = WDTPW | WDTHOLD;                // Stop watchdog timer
  // Configure GPIO
   Init_GPIO();
   PM5CTL0 &= ~LOCKLPM5;                    // Disable the GPIO power-on default high-impedance mode
                                           // to activate 1previously configured port settings

  __bis_SR_register(SCG0);                 // disable FLL
  CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
  CSCTL0 = 0;                              // clear DCO and MOD registers
  CSCTL1 &= ~(DCORSEL_7);                  // Clear DCO frequency select bits first
  CSCTL1 |= DCORSEL_3;                     // Set DCO = 8MHz
  CSCTL2 = FLLD_0 + 243;                   // DCODIV = 8MHz
  __delay_cycles(3);
  __bic_SR_register(SCG0);                 // enable FLL
  while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked

  CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;                   // Disable the GPIO power-on default high-impedance mode
 WDTCTL = WDT_ADLY_1000;                   // WDT 32ms, SMCLK, interval timer
   SFRIE1 |= WDTIE;               
  // Configure UART 9600
  UCA1CTLW0 |= UCSWRST;
  UCA1CTLW0 |= UCSSEL__SMCLK;
  UCA1BR0 = 52;                     
  UCA1BR1 = 0x00;
  UCA1MCTLW = 0x4900 | UCOS16 | UCBRF_1;
  UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
  UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
_BIS_SR(GIE);
   // __bis_SR_register(GIE); 
  //_BIS_SR(LPM3_bits+GIE); 
}

//---------------------to chk internet connectivity ---------------------------------------------------------

int Ping ()
  {
    RxAck=1;
    Tx_Uart("AT+PING=\"iot.eclipse.org\"");
    Tx_Uart_Char('\r');
    Tx_Uart_Char('\n');
  
  while(RxAck!=1);           // must be waiting here until recive complete packet
       
  
     //  Tx_Uart("[");           // used for debug
     //  Tx_Uart(Rx_Buffer);    // used for debug
     //  Tx_Uart("]");         // used for debug

       cmp_result = strcmp((const char*)("ERROR"),(const char*)(Rx_Buffer));

    if(cmp_result==0)
    {
      er_net++;
    if(er_net>10)
    {
    RxAck=1;
    er_net=0;
    Modem_Config();
    Board_Init();
    clean(Rx_Buffer);
    state=3;          // if error more than 10 times reset modem go for state 3
    }
    return 1;        // go 
  }
else
  {
   // clean (Rx_Buffer);
  state=2;
  return 0;
  } 
  }

//------------------------------------------------------------------------------
void Modem_Config()
{
first:Tx_Uart("AT+RST");        // Reset modem
	Tx_Uart_Char('\r');     // enter
	Tx_Uart_Char('\n');     // enter
	Delay(5);
	Tx_Uart("ATE0");        // refer ESP manual 
	Tx_Uart_Char('\r');
	Tx_Uart_Char('\n');
	Delay(1);
	RxAck=0;                // clear flag
        Tx_Uart("AT");          // 
	Tx_Uart_Char('\r');
	Tx_Uart_Char('\n');
	if(Check("OK","OK","OK")!=1)
	{
		goto first;
	}
	Delay(1);
}

    void clean(char *var) 
       { int i = 0;
    while(var[i] != '\0') 
      {  var[i] = '\0';
        i++;}
        }

//------------------------------------------------------------------------------
int Check(char * jello, char * jello1,char * jello2)
{
                 while(RxAck!=1);
		 Tx_Uart("[");
		 Tx_Uart(Rx_Buffer);
		 Tx_Uart("]");
		 cmp_result = strcmp((const char*)(jello),(const char*)(Rx_Buffer));
                  cmp_result1 = strcmp((const char*)(jello1),(const char*)(Rx_Buffer));
                   cmp_result2 = strcmp((const char*)(jello2),(const char*)(Rx_Buffer));
                  
		 if(cmp_result==0| cmp_result1==0|cmp_result2==0)
		 {
			 return 1;
		 }
		 else
		 {
			 return 0;
		 }
}
//------------------------------------------------------------------------------

void Int_To_String(char str[], int num)
{
    int i, rem, len = 0, n;

    n = num;
    while (n != 0)
    {
        len++;
        n /= 10;
    }
    for (i = 0; i < len; i++)
    {
        rem = num % 10;
        num = num / 10;
        str[len - (i + 1)] = rem + '0';
    }
    str[len] = '\0';
}
//------------------------------------------------------------------------------

void Delay(int sec)
{
           delay_cnt=sec;
           SFRIE1 |= WDTIE;                        // Enable WDT interrupt 
	   _BIS_SR(LPM3_bits);                // Enter LPM0
	  // SFRIE1  &= ~WDTIE;                   // Enable WDT interrupt
           _NOP();
}
//------------------------------------------------------------------------------
void Tx_Uart(char * str)
{
    int i = 0;
    for(i = 0; i < strlen(str); i++)
    {
        while (!(UCA1IFG&UCTXIFG));
        UCA1TXBUF = str[i];
    }
}
//------------------------------------------------------------------------------
void Tx_Uart_Char(char n)
{
        while (!(UCA1IFG&UCTXIFG));
        UCA1TXBUF = n;
}
//------------------------------------------------------------------------------
void Tcp_Init( char * URL , char * PORT ,int ch) 

{int j=0;
	if(ch==1) // for Mqtt connection
	{
	one:    Tx_Uart("ATE0");
                Tx_Uart_Char('\r');
                Tx_Uart_Char('\n');
                Delay(2);
                RxAck=1;
                  while(RxAck!=1);   
		Tx_Uart("AT+CIPSTART=\"TCP\",");
		Tx_Uart("\""); // ? "\""
		Tx_Uart(URL);
		Tx_Uart("\",");
		Tx_Uart(PORT);
		Tx_Uart_Char('\r');
		Tx_Uart_Char('\n');
                Tx_Uart("TCP Data");
		if(!strcmp((const char*)("OK"),(const char*)(Rx_Buffer))) //?
		{
			P1OUT|= BIT0;
			Delay(2);
		}
			else
		{
			Tcp_Disconnect();
			P1OUT&= BIT0;
			Delay(2);
			j++;
                        
                        if(j>5)
			{
			   j=0;
			   Modem_Config();
			}
			goto one;
		}
	}

	if(ch==2) // for http connection
	{
		// do something for HTTP
	}
}

//------------------------------------------------------------------------------
//  function for disconnect from tcp server

     void Tcp_Disconnect()

      {
	Tx_Uart("AT+CIPCLOSE");
	Tx_Uart_Char('\r');
	Tx_Uart_Char('\n');
      }

//------------------------------------------------------------------------------

// ----------------------function to transmit tcp packet to the server
        
    void Tcp_Packet(unsigned char * Mqtt_message, int length)

     {
	for(int P=0;P<length;P++)
	{
		Tx_Uart_Char(Mqtt_message[P]);
	}
	Tx_Uart_Char('\r');
	Tx_Uart_Char('\n');
     }
//------------------------------------------------------------------------------
   void Tx_Integer(int tem)
     {
	char time[3];
	if(tem>9)
	{
	time[0]=tem/10+'0';
	time[1]=tem%10+'0';
	time[2]='\0';
	Tx_Uart(time);
	}
	else
	{
	time[0]=tem%10+'0';
	time[1]='\0';
	Tx_Uart(time);
	}
      }
//------------------------------------------------------------------------------
//        fucntion to send the length of the data packet

    void Tcp_Length(int length)
  {
     
  
     //  Tx_Uart("[");           // used for debug
     //  Tx_Uart(Rx_Buffer);    // used for debug
     // Tx_Uart("]");      
	Tx_Uart("AT+CIPSEND=");
	Tx_Integer(length);
	//Delay(1);
	Tx_Uart_Char('\r');
	Tx_Uart_Char('\n');
	Delay(1);
  RxAck=1;
  while(RxAck!=1);
  if(!strcmp((const char*)(">"),(const char*)(Rx_Buffer))) //?
  {
    Tx_Uart("Got Data>");
  }
  
		
  
  }
//------------------------------------------------------------------------------
//       function to ping the Mqtt server

    void Mqtt_Ping(unsigned char * Mqtt_message)
    {
	Mqtt_message[0] = 192;                         // Mqtt Message Type PING
	Mqtt_message[1] = 0;   // Remaining length
	Tcp_Length(2);
	Tcp_Packet(Mqtt_message,2);
    
    }

//------------------------------------------------------------------------------
//       function 

    void Mqtt_Disconnect(unsigned char * Mqtt_message)
    {
	Mqtt_message[0] = 224;                         // Mqtt Message Type PING
	Mqtt_message[1] = 0;   // Remaining length
	Tcp_Length(2);
	Tcp_Packet(Mqtt_message,2);
    }
//------------------------------------------------------------------------------

 void Mqtt_Connect(char * client_id,char * url,char * port,unsigned char * Mqtt_message)
    {
	uint8_t i = 0;
	uint8_t client_id_length = strlen(client_id);
	Mqtt_message[0] = 16;                      // Mqtt Message Type CONNECT
	Mqtt_message[1] = 14 + client_id_length;   // Remaining length of the message
	Mqtt_message[2] = 0;                       // Protocol Name Length MSB
	Mqtt_message[3] = 6;                       // Protocol Name Length LSB
	Mqtt_message[4] = 77;                      // ASCII Code for M
	Mqtt_message[5] = 81;                      // ASCII Code for Q
	Mqtt_message[6] = 73;                      // ASCII Code for I
	Mqtt_message[7] = 115;                     // ASCII Code for s
	Mqtt_message[8] = 100;                     // ASCII Code for d
	Mqtt_message[9] = 112;                     // ASCII Code for p
	Mqtt_message[10] = 3;                      // Mqtt Protocol version = 3
	Mqtt_message[11] = 0;                      // conn flags
	Mqtt_message[12] = 0X00;                      // Keep-alive Time Length MSB
	Mqtt_message[13] = 0x3C;                     // Keep-alive Time Length LSB
	Mqtt_message[14] = 0;                      // Client ID length MSB
	Mqtt_message[15] = client_id_length;       // Client ID length LSB
	
        for(i=0;i<client_id_length;i++)
	{
		Mqtt_message[16+i]=client_id[i];
	}
	Tcp_Init(url,port,1);
	MqttMessageLength=16+ strlen(client_id);
	Tcp_Length(MqttMessageLength);
	Tcp_Packet(Mqtt_message,MqttMessageLength);
	Delay(2);
    }
//------------------------------------------------------------------------------
// publish message starts from here..................................

    void Mqtt_Publish(unsigned char * Mqtt_message, char * topic, char * message)
    {
	uint8_t topic_length = strlen(topic);
	uint8_t message_length = strlen(message);
	MqttMessageLength=4+topic_length+message_length;
	Mqtt_message[0]=48;                                  // Mqtt message type publish
	Mqtt_message[1]=2+topic_length+message_length;		  // Mqtt message length
	Mqtt_message[2]=0;									  // CLIENT ID LSB
	Mqtt_message[3] = topic_length; 					  // CLIENT ID MSB
    
        for(int T = 0; T < topic_length; T++)
        {
	  Mqtt_message[4 + T] = topic[T];
	}

	for(int T = 0; T < message_length; T++)
        {
	  Mqtt_message[4 + topic_length + T] = message[T];
	}
	Tcp_Length(MqttMessageLength);
	Tcp_Packet(Mqtt_message,MqttMessageLength);
	Delay(2);
    }
//------------------------------------------------------------------------------

     void Mqtt_Subscribe(unsigned char * Mqtt_message, char * topic)
      {
	uint8_t topic_length=strlen(topic);
	MqttMessageLength=topic_length;
	Mqtt_message[0] =128;
	Mqtt_message[1] =4+topic_length+1;
	Mqtt_message[2] =0;
	Mqtt_message[3] =0;
	Mqtt_message[4] =0;
	Mqtt_message[5] =topic_length;
	for(int T=0;T<topic_length;T++)
	{
		Mqtt_message[6+T]=topic[T];
	}
	Mqtt_message[6+topic_length]=0;
	MqttMessageLength=7+topic_length;
	Tcp_Length(MqttMessageLength);
	Tcp_Packet(Mqtt_message,MqttMessageLength);
	Delay(2);
      }
//------------------------------------------------------------------------------ 
  void Mqtt_Unsubscribe(unsigned char * Mqtt_message, char * topic)
      {
	uint8_t topic_length=strlen(topic);
	MqttMessageLength=topic_length;
	Mqtt_message[0] =160;
	Mqtt_message[1] =4+topic_length;
	Mqtt_message[2] =0;
	Mqtt_message[3] =0;
	Mqtt_message[4] =0;
	Mqtt_message[5] =topic_length;
	
        for(int T=0;T<topic_length;T++)
	{
		Mqtt_message[6+T]=topic[T];
	}
	MqttMessageLength=6+topic_length;
	Tcp_Length(MqttMessageLength);
	Tcp_Packet(Mqtt_message,MqttMessageLength);
	Delay(2);
      }

//-----------------------------------------------------------------------------
 void ON_OFF()
      {    

	if(Rx_Buffer[Rx_Buffer_Len-1]=='8')
	 {
	  P1OUT &=~BIT6;
          P2OUT &=~BIT2;
           motor_stat=0;   
          
	    }
	if(Rx_Buffer[Rx_Buffer_Len-1]=='9')
           {
	  P1OUT |=BIT6;
          P2OUT |=BIT2;
           motor_stat=1;
            
               }
      }
//-----------------------------------------------------------------------------

// Watchdog Timer interrupt service routine
/*
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) WDT_ISR (void)
#else
#error Compiler not supported!
#endif
*/

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
    //  P1OUT ^= BIT0;                          // Toggle P1.0 (LED)
/*delay_cnt--;
if (delay_cnt==0)
{
    _BIC_SR_IRQ(LPM0_bits);
   __bic_SR_register_on_exit(GIE);
}*/
          delay_cnt--;
          del--;
          if(del==0)
          {
            del=WATCH_TIME;
            state=3;
          }
        //  del--;
      //  if(del==0)
      //  {
       //   del=15;
      //    Mqtt_Ping(mqtt_message); 
          
      //     while(RxAck!=1);           // must be waiting here until recive complete packet
       
  
    //   Tx_Uart("[");           // used for debug
    //   Tx_Uart(Rx_Buffer);    // used for debug
    //   Tx_Uart("]");         // used for debug
      //  }
      if (delay_cnt==0)
      { 
      _BIC_SR_IRQ(LPM3_bits);
    __bic_SR_register(LPM3_bits);
      }
}


//------------------------------------------------------------------------------
/*#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_UART_UCRXISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:

      while(!(UCA1IFG&UCTXIFG));

*/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_UART_UCRXISR(void)
{
    	if(PAC_ID==0)                   // PAC_ID=0 WILL search for \n \r
        {    if(UCA1RXBUF=='>')
        {
          Rx_Buffer[Rx_Buffer_Len]=UCA1RXBUF;
          Rx_Buffer[1]='\0';
          RxAck=1;
        }// PAC_ID=1 WILL search for #
	else if(UCA1RXBUF!='\n')
	  { 
	    if(UCA1RXBUF!='\r')
	    {
	    Rx_Buffer[Rx_Buffer_Len]=UCA1RXBUF;
	    Rx_Buffer_Len++;
	    }
	   RxAck=0;
	  }
      	  else
	  {
	    Rx_Buffer[Rx_Buffer_Len]='\0';
	    Rx_Buffer_Len=0;
	    RxAck=1;
	  
           }
	}
          if(PAC_ID==1)
	{
          if(UCA1RXBUF!='#')
	  {
	    Rx_Buffer[Rx_Buffer_Len]=UCA1RXBUF;
	    Rx_Buffer_Len++;
            cmp_result = strcmp((const char*)("hi"),(const char*)(Rx_Buffer));
             RxAck=0;
	  }
          else //if(UCA1RXBUF=='#')
          {

          ON_OFF();
          Rx_Buffer[Rx_Buffer_Len]='\0';
          RxAck=1;

          Rx_Buffer_Len=0;
          }
	  /*else
	  {
            ON_OFF();
   	    Rx_Buffer[Rx_Buffer_Len]='\0';
	    RxAck=1;

          Rx_Buffer_Len=0;

	  }*/
	}
     //   __no_operation();
      //break;
    //case USCI_UART_UCTXIFG: break;
    //case USCI_UART_UCSTTIFG: break;
    //case USCI_UART_UCTXCPTIFG: break;
    //default: break;
  }

//-----------------------------------------------------------------------------

// ADC interrupt service routine
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
//#else
//#error Compiler not supported!
//#endif
{
   
            temp = ADCMEM0;
            // Temperature in Celsius
            // The temperature (Temp, C)=
            IntDegC = (temp-CALADC_15V_30C)*(85-30)/(CALADC_15V_85C-CALADC_15V_30C)+30;

            // Temperature in Fahrenheit
            // Tf = (9/5)*Tc | 32
            IntDegF = 9*IntDegC/5+32;
              //__bis_SR_register(LPM3_bits+GIE);        // Exit LPM0

          //  break;
        //default:
          //  break;
    //}
}
//-----------------------------------------------------------------------------------------
// Timer A0 interrupt service routine
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
//#else
//#error Compiler not supported!
//#endif
{
    ADCCTL0 |= ADCENC | ADCSC;                                    // Sampling and conversion start
}


void Init_GPIO()
{
    P1DIR = 0xFF; P2DIR = 0xFF; P3DIR = 0xFF;
    P1REN = 0xFF; P2REN = 0xFF; P3REN = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00; P3OUT = 0x00;
    // Configure UART pins
    P2SEL0 |= BIT5 | BIT6;                    // set 2-UART pin as second function
    TA0CCTL0 |= CCIE;                                             // TACCR0 interrupt enabled
    TA0CCR0 = 65535;
    TA0CTL = TASSEL__ACLK | MC__UP;                               // ACLK, UP mode

    // Configure ADC - Pulse sample mode; ADCSC trigger
    ADCCTL0 |= ADCSHT_8 | ADCON;                                  // ADC ON,temperature sample period>30us
    ADCCTL1 |= ADCSHP;                                            // s/w trig, single ch/conv, MODOSC
    ADCCTL2 |= ADCRES;                                            // 10-bit conversion results
    ADCMCTL0 |= ADCSREF_1 | ADCINCH_12;                           // ADC input ch A12 => temp sense
    ADCIE |=ADCIE0;                                               // Enable the Interrupt request for a completed ADC_B conversion

    // Configure reference
    PMMCTL0_H = PMMPW_H;                                          // Unlock the PMM registers
    PMMCTL2 |= INTREFEN | TSENSOREN;                              // Enable internal reference and temperature sensor

}
