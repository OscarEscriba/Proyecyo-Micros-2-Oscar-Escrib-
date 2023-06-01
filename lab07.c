/*
 * File:   lab07.c
 * Author: Oscar Escriba
 *
 * Created on 12 de abril de 2023, 10:18 PM
 */
     #pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
     #pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
     #pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
     #pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
     #pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
     #pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
     #pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
     #pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
     #pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
     #pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
     // CONFIG2
     #pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
     #pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits Write protection off)
            
     #include <xc.h>
     #include <stdint.h>
     #include <stdio.h> 
     void USART_send(const char datas);  
     void USART_print(const char *string);
     #define _XTAL_FREQ 1000000
     #define _tmr0_value 237
     #define IN_MIN_POT 0                
     #define IN_MAX_POT 255              
     #define OUT_MIN_PWM 16               
     #define OUT_MAX_PWM 80        
     #define LED_PIN RC0 
     //valores a comparar
     unsigned short CCPR = 0;        // Interpolacion lineal
     unsigned short CCPR_2 = 0;    
     uint8_t valor1 =0; 
     uint8_t valor2 =0; 
     //variables globales
     uint8_t adress=0x00;  
     int IADC=0;  
     int state_flag =0; 
     uint8_t opt_sel; 
     uint8_t portb_char;  
     uint8_t pot1; 
const char datas =65;  
char mensaje[]="hola mundo";   
     unsigned char vacio = '\r';  
     //valores del CCPR 
    uint8_t servo_position=16; 
    uint8_t servo_position2=80; 
     //proto funciones
     void setup(void); 
     void item_list(void); 
     void enter(int a);  
 
     unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
                 unsigned short out_min, unsigned short out_max); 
     
     uint8_t EEPROM_read(uint8_t adress); 
     void EEPROM_write(uint8_t adress, uint8_t data);  
     //FUNCION PARA ENVIAR LA COMUNICACION SERIAL 
   void USART_print(const char *string)
{
    int i = 0;
    
    for(i; string[i] != '\0'; i++)
    {
        USART_send(string[i]);
    }
}  
void USART_send(const char datas)
{
    while(!TRMT);
    TXREG = datas;
}   
//FUNCION PARA ENTERS DE CANTIDAD ESPECIFICADA  
void enter(int a) { 
    while (a>0) { 
        a--; 
        __delay_us(40); 
        TXREG= vacio; 
    }
} 
     void __interrupt() isr (void){
       if (INTCONbits.RBIF) { 
           if (PORTBbits.RB0==0) {  
                PORTCbits.RC0=0;
               IADC=1; 
           }else if(PORTBbits.RB1==0){ 
               IADC=0; 
               PORTCbits.RC0=1; 
           }else if (PORTBbits.RB2==0){
                PORTCbits.RC3=1; 
                PORTCbits.RC4=1;
           }else if(PORTBbits.RB3==0){ 
               PORTCbits.RC3=0; 
               PORTCbits.RC4=0;
           }
           INTCONbits.RBIF=0;
       } 
            if (PIR1bits.ADIF) { 
                   if (ADCON0bits.CHS==0){ 
                       valor1=map(ADRESH, IN_MIN_POT, IN_MAX_POT, OUT_MIN_PWM, OUT_MAX_PWM); 
                   } else if (ADCON0bits.CHS==1){ 
                       valor2=map(ADRESH, IN_MIN_POT, IN_MAX_POT, OUT_MIN_PWM, OUT_MAX_PWM); 
                   }else if (ADCON0bits.CHS==2){ 
                           CCPR_2 = map(ADRESH, IN_MIN_POT, IN_MAX_POT, OUT_MIN_PWM, OUT_MAX_PWM); 
                           CCPR2L = (uint8_t)(CCPR_2>>2);    
                           CCP2CONbits.DC2B0 = CCPR_2 & 0b11; 
                   }else if (ADCON0bits.CHS==3) { 
                       pot1=ADRESH; 
                   }
                   PIR1bits.ADIF=0; 
               }
       if(PIR1bits.RCIF) { 
             if (state_flag ==0) {
            opt_sel=RCREG; 
            RCREG=0;
        } else if (state_flag ==1){ 
            state_flag =0;
            portb_char=RCREG; 
            TXREG = portb_char; 
            PORTD= portb_char; 
            RCREG=0; 
            enter(1); 
            item_list(); 
        }
       }
         return;
     }

     void main(void) {
         setup();
         item_list(); 
         while(1){
             //conversion adc, 3 canales b1,b2,b3 
             if(ADCON0bits.GO == 0b00){                 
                 if(ADCON0bits.CHS == 0){   
                     ADCON0bits.CHS = 1;    // Cambio de canal
                 }
                 else if(ADCON0bits.CHS == 1){
                 ADCON0bits.CHS = 2;
                 }else if (ADCON0bits.CHS==2) { 
                     ADCON0bits.CHS=3; 
                 }else if (ADCON0bits.CHS==3) { 
                     ADCON0bits.CHS=0; 
                 }
                     __delay_us(1000);                
                     ADCON0bits.GO = 1;            
             }     
               if(IADC ==1){ 
             //se comparan los valores de las entradas mapeadas. 
             if (valor1 > valor2){ 
                 CCPR1L = (servo_position>>2);    
                 CCP1CONbits.DC1B0 = servo_position & 0b11;
             }else if(valor1 < valor2) {
                  CCPR1L = (servo_position2>>2);    
                  CCP1CONbits.DC1B0 = servo_position2 & 0b11;
             }
             }else if (IADC ==0){ 
                   int data_to_write=80; 
                   int read_data; 
                     //mediante la funcion de eeprom write, se escriben los datos 
                     //en la direccion de memoria y el dato guardado en la variable. 
                     EEPROM_write(adress, data_to_write);
                     //se lee el dato que vayamos a utilizar por medio de la funcion de la funcione y la variable
                     read_data=EEPROM_read(adress); 
                     CCPR1L = (uint8_t)(read_data>>2);    
                     CCP1CONbits.DC1B0 = read_data & 0b11;
             } 
                     if (opt_sel == 0x61){
                         opt_sel = 0x00; 
                         USART_print("valor del potenciometro: "); 
                          __delay_ms(500); 
                         TXREG = pot1; 
                         __delay_ms(500); 
                         item_list();
                  } else if(opt_sel ==0x62) {
                         state_flag=1; 
                         USART_print("Ingrese el caracter en ASCCI: "); 
                         opt_sel =0x00; 
        }
         } 
         return;
     }

     void setup(void){
         //registros A como analogicos B como digitales. 
         ANSELH=0x00;  
         ANSEL=0b00001111; 
         // se declaran si son entradas o salidas los puertos. 
         TRISA = 0b00000111; //los primros 3 puertos como entradas.  
         TRISB = 0b00001111; //pone el primer puerto del B como entrada digital, activar el modo automatico.  
         //inicializa los puertos. 
         PORTA = 0b00001111; //pone el PORTA y lo inicializa asi  
         PORTB = 0b00000000; //primeros dos pines como entrada e inicializados 1  
         //configuracion salida comunicacion serial
         PORTD =0x00;  
         TRISD=0; 
         
           TRISCbits.TRISC6 = 0;   // Configura el pin RC6 como salida (TX)
           TRISCbits.TRISC7 = 1; 
        
         // CONFIGURACION RELOJ INTERNO
         OSCCONbits.IRCF = 0b011;    // 1MHz
         OSCCONbits.SCS = 1;         // Oscilador interno
         
         // CONFIGURACION ADC
         ADCON0bits.ADCS = 0b01;             // Fosc/8
         ADCON1bits.VCFG0 = 0;               // VDD
         ADCON1bits.VCFG1 = 0;               // VSS
         ADCON0bits.CHS = 0;                // AN12
        
         ADCON1bits.ADFM = 0;               
         ADCON0bits.ADON = 1;                // ADC
         __delay_us(40);                     
             
         // CONFIGURACION PWM
         TRISCbits.TRISC3 = 1;       
         TRISCbits.TRISC2 = 1;       
         TRISCbits.TRISC1 = 1;       
         PR2 = 155.25;                 
         
         //PARA LA COMUNICACION SERIAL 
         TXSTAbits.SYNC=0; 
         TXSTAbits.BRGH=1;  
         BAUDCTLbits.BRG16=1; 
         SPBRG=25; 
         SPBRGH=0;  
         RCSTAbits.SPEN=1; 
         RCSTAbits.RX9=0; 
         RCSTAbits.CREN=1; 
         TXSTAbits.TXEN=1;  
         
         
         // CONFIGURACION CCP
         CCP1CON = 0;                       
         CCP2CON = 0;                       
         CCP1CONbits.P1M = 0;               // Modo single output
         CCP2CONbits.CCP2M = 0b1100;        // Modo single output
         CCP1CONbits.CCP1M = 0b1100;        // PWM

         CCPR1L = 48>>2;
         CCPR2L = 48>>2;
         CCP1CONbits.DC1B0 = 48 & 0b11;    
         CCP2CONbits.DC2B0 = 48 & 0b11;    
         
         PIR1bits.TMR2IF = 0;       
         T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
         T2CONbits.TMR2ON = 1;    
         while(!PIR1bits.TMR2IF);    
         PIR1bits.TMR2IF = 0;       
         
         TRISCbits.TRISC2 = 0;       // Salida de PWM
         TRISCbits.TRISC1 = 0;       // Salida de PWM

         // CONFIGURACION INTERRUPCIONES
         PIR1bits.ADIF = 0;          
         PIE1bits.ADIE = 1;          // Int. de ADC
         INTCONbits.PEIE = 1;        // Int. de perifericos
         INTCONbits.GIE = 1;         // Int. globales 
         INTCONbits.RBIE=1; 
         IOCBbits.IOCB0=1; 
         IOCBbits.IOCB1=1; 
         IOCBbits.IOCB2=1;  
         IOCBbits.IOCB3=1; 
         INTCONbits.RBIF=0; 
         PIR1bits.RCIF=0; 
         PIE1bits.RCIE=1;  
         //configuracion de interrupcion en el portb 
         //configuracion pushbuttoms en portb 
         OPTION_REGbits.nRBPU=0; 
         WPUBbits.WPUB0=1; 
         WPUBbits.WPUB1=1; 
         WPUBbits.WPUB2=1; 
         WPUBbits.WPUB2=1; 
         //led para indicar el estado en el que se encuentra el robot. salida del TRISC 
         TRISCbits.TRISC0 = 0; 
         TRISCbits.TRISC3 = 0; 
         TRISCbits.TRISC4 = 0; 
     }
          void item_list() { 
    enter(2); 
    USART_print("----------Main menu----------"); 
    enter(1); 
    USART_print("a) lectura del potenciometro"); 
    enter(1); 
    USART_print("b) enviar ASCII"); 
    enter(2); 
}
     unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
                 unsigned short y0, unsigned short y1){
         return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
     }
     uint8_t EEPROM_read(uint8_t adress){ 
         EEADR == adress; 
         EECON1bits.EEPGD=0; 
         EECON1bits.RD=1; 
         return EEDAT; 
     }
     void EEPROM_write (uint8_t adress, uint8_t data){ 
         EEADR = adress; 
         EEDAT =data; 
         EECON1bits.EEPGD=0; 
         EECON1bits.WREN=1; 
         INTCONbits.GIE=0; 
         EECON2 = 0x55; 
         EECON2 = 0xAA; 
         EECON1bits.WR=1; 
         EECON1bits.WREN=0;
         INTCONbits.RBIF=0; 
         INTCONbits.GIE=1; 
     } 

     