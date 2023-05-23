/* 
 * File:   proyectofinal.c
 * Author: Luis Pedro Gonzalez 21513
 *
 * Created on 26 de abril de 2023, 21:16 PM
 */


// CONFIG1
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
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include <pic16f887.h>


#define _XTAL_FREQ 4000000  //frecuencia de 4MHZ

//////--------valores del potenciometro--------------
#define potmin 0//valor minimo del potenciometro 
#define potmax 255//valor maxiomo del potenciometro
#define pwmmin 100//valor minimo el pwm 0.4 ms para
//#define pwmmax 650//valor maximo para pwm 2.4 ms
#define pwmmax 600
//variables para controlar el ancho de pulso
#define tmr0_val 249
unsigned int CCPRA = 0; //variable para el ccpr1
unsigned int CCPRB = 0; //variable para el ccpr2
uint8_t bandera; //bandera para elegir los modos

//-----------------prototipos-----------------
void setup(void); //prototipo de setuo
void setupADC(void); //prototipo ADC
void setupPWM(void); //prototipo del PWM
void tmr0_setup(void);
void delay(unsigned int micro); //función para obtener delay variable
void manualPWM_ISR(void);



unsigned int pot3; //valor para tiempo en alto de PWM para intensidad del led
unsigned int pot4; //valor para tiempo en alto de PWM para intensidad del led


//------------mapeo de los valores potenciometro 1 y 2 -----------------
unsigned short cambiopwm(uint8_t valor, uint8_t POTMIN, uint8_t POTMAX,
        unsigned short PWMMIN, unsigned short PWMMAX);

//-----------------mapeo de valores potenciometro 3 y 4--------
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax){ //función para mapear valores
    return ((value - inputmin)*(outmax-outmin)) / (inputmax-inputmin)+outmin;} 


////Rutina de interrupciones
void __interrupt() isr(void){
 ////interrupcion para primer potecniometro
    if (PIR1bits.ADIF){ //chequea interrupcion de adc
        if (ADCON0bits.CHS == 0b0000){ //revisa el canal 1 an0
            CCPRA = cambiopwm(ADRESH, potmin, potmax, pwmmin, pwmmax);//se mapean los valores 
            CCPR1L = (uint8_t)(CCPRA>>2);//asigna los 8 bits mas significativos a cpr1l
            CCP1CONbits.DC1B = CCPRA & 0b11; //asigna a dc1b los 2 bits menos significaticos
        }
        
        else if (ADCON0bits.CHS ==  0b0010){//chequea la interrupcion del adc
            CCPRB = cambiopwm(ADRESH, potmin, potmax, pwmmin, pwmmax);//se mapean los valores 
            CCPR2L = (uint8_t)(CCPRB>>2);//asigna los 8 bits mas significativos a cpr2l
            CCP2CONbits.DC2B0 = CCPRB & 0b01; //se le asigna el primer bit menos significativo
            CCP2CONbits.DC2B1 = CCPRB & 0b10; //se le asigna el segundo bit menos significativo
        }
        
        else if (ADCON0bits.CHS == 0b0011){
            pot3 = map(ADRESH, 0, 255, 1, 10); //mapear valores para servomotor 3
        }
        
        else if (ADCON0bits.CHS == 0b0100){//an4
            pot4 = map(ADRESH, 0, 255, 1, 10); //mapear valores para servomotor 3
        }
        PIR1bits.ADIF = 0; //limpia la bandera del adc
    }
    
    //interrupcion del tmr0
    if (INTCONbits.T0IF == 1){
        manualPWM_ISR();
    }
    
    //interrupcion del portb
    if (INTCONbits.RBIF){//revisar las interrupciones del portb
        if (PORTBbits.RB0 == 0){//enceder
            PORTDbits.RD2 = 1; //limpia el bits que inica el sleeps
            PORTDbits.RD3 = 1; //limpia el bits que inica el sleeps

        }
        
        else if (PORTBbits.RB1 == 0){
            PORTDbits.RD2 = 0 ; //encender un led para indicarme que sleep esta encdedio
            PORTDbits.RD3 = 1; //limpia el bits que inica el sleeps

        }
        
        else if (PORTBbits.RB2 == 0){
            PORTDbits.RD2 = 1; //apagar led que indica si esta dormido 
            PORTDbits.RD3 = 0; //limpia el bits que inica el sleeps

        }
        INTCONbits.RBIF = 0; 

    }
    return;

}


//////////main
void main(void){
    setup();//llamar a las configuraciones genreales
    setupADC();//LLamar a la configuracion del adc
    setupPWM();
    tmr0_setup();
        while (1){
        if (ADCON0bits.GO == 0) { // Chequea si el ADC está encendido
            if (ADCON0bits.CHS == 0b0000) { // Chequea el canal 0
                ADCON0bits.CHS = 0b0010; // Cambia a canal 1// 0b0010
                 __delay_us(40);

            } else if (ADCON0bits.CHS == 0b0010) { // Chequea el canal 1
                ADCON0bits.CHS = 0b0011; // Cambia a canal analógico 0
                __delay_us(40);
            }
            
            else if (ADCON0bits.CHS == 0b0011) { // Chequea el canal 1
                ADCON0bits.CHS = 0b0100; // Cambia a canal analógico 0
                __delay_us(40);

            }
            
            else if (ADCON0bits.CHS == 0b0100){
                ADCON0bits.CHS = 0b0000; // Cambia a canal analógico 0
                __delay_us(40);
            }
            
            __delay_us(20); // Delay después de iniciar la conversión
            ADCON0bits.GO = 1; // Inicio de conversión
                
            }
        }
    return;
}


//////////////setup general

void setup(void){
    // --------------- Definir como digitales --------------- 
    ANSELH = 0; //puertos digitales 
    ANSELbits.ANS0 = 1; // ra0 como analogico
    ANSELbits.ANS2 = 1; //ra1 como analogico (era este)
    ANSELbits.ANS3 = 1; //ra1 como analogico (era este)
    ANSELbits.ANS4 = 1; //ra1 como analogico (era este)

    
    // --------------- Configura puertos --------------- 
    TRISAbits.TRISA0 = 1; //puerto A0 como entrada
    TRISAbits.TRISA2 = 1; //puerto A1 como entrada (era este)
    TRISAbits.TRISA3 = 1; //puerto A1 como entrada (era este)
    TRISAbits.TRISA4 = 1; //puerto A1 como entrada (era este)
    
    //botones
    TRISBbits.TRISB0 = 1; //rb0 como entrada
    TRISBbits.TRISB1 = 1; //rb1 como entrada 
    TRISBbits.TRISB2 = 1; //rb1 como entrada 
    
    TRISCbits.TRISC3 = 0; //puerto C3 como salida (era este)
    TRISCbits.TRISC4 = 0; //puerto C4 como salida (era este)
    TRISE = 0x00; 
    TRISD = 0x00; 

    
    // --------------- limpiar puertos --------------- 
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTE = 0;
    PORTD = 0;
    
    
    //----------pullups------------------
    OPTION_REGbits.nRBPU = 0; //habilitarr pullups
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1; 
    WPUBbits.WPUB2 = 1; 
 
    
    // --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b0110 ; // establecerlo en 4 MHz
    OSCCONbits.SCS = 1; // utilizar oscilador intern
    

    
    // --------------- INTERRUPCIONES --------------- 

    INTCONbits.GIE = 1; // habilitar interrupciones globales
    INTCONbits.PEIE = 1; // habilitar interrupciones perifericas
    INTCONbits.RBIE = 1; //habilitar interrupciones en portb
    
    IOCBbits.IOCB0 = 1; //habilitar interrupciones en rb0
    IOCBbits.IOCB1 = 1; // habilitar interrupciones en rb1
    IOCBbits.IOCB2 = 1; // habilitar interrupcion en el rb2

    
    INTCONbits.RBIF = 0; //limpirar bander de interrupcion de portb
    PIE1bits.ADIE = 1; // habilitar interrupciones de ADC
    PIR1bits.ADIF = 0; // limpiar la bandera de interrupcion del ADC
}

// --------------- Setup del ADC --------------- 
void setupADC(void){
    
    // --------------- Seleccion de reloj ---------------
    ADCON0bits.ADCS = 0b10; // Fosc/32
    
    // --------------- Seleccion voltaje referencia --------------- 
    ADCON1bits.VCFG1 = 0; // Voltaje de referencia de 0V
    ADCON1bits.VCFG0 = 0; // Voltaje de referencia de 5V
    
            
    // --------------- justificado izquierda ---------------
    ADCON1bits.ADFM = 0; 
    
    // --------------- canales --------------- 
    ADCON0bits.CHS = 0b0000; // seleccionar AN0
           
            
    //--------------- Iniciar el ADC ---------------
    ADCON0bits.ADON = 1;  //INICIA EL ADC
    __delay_ms(1);
}

//pr2,bits prescale y oscilador afectan al servo

//-------setup de PWM-------------------
void setupPWM(void){
  //--------------ccp1---------------    
    TRISCbits.TRISC1 = 1; //CCP1 como entrada
    TRISCbits.TRISC2 = 1; //CCP2 como entrada
    
    PR2 = 249 ;   //periodo de 4ms en el tmr2
    
    ////------------configuracion e ccp1------------------
    CCP1CON = 0; //APAGA CCP1 INICIALMENTE
    CCP2CON = 0; //APAGA CCP2 INICIALMENTE
    
    CCP1CONbits.P1M = 0; //modo de single output
    CCP1CONbits.CCP1M = 0b1100; //modo pwm para ccp1
    CCP2CONbits.CCP2M = 0b1100; //modo pwm para ccp2

    
    CCPR1L = 250>>2; //asiga 2 bits de 250 a ccpr1l
    CCP1CONbits.DC1B = 250 & 0b11;//asigna los bits menos sigificaticos del and a dc1b 
    CCPR2L = 250>>2;//los 2 bits desplazados se asignan a ccpr2l
    CCP2CONbits.DC2B0 = 250 & 0b01;//asigna el valor del and a dc2b0 
    CCP2CONbits.DC2B1 = 250 & 0b10; //asigna los bits a dc2b1
    //CCPR1L = 3; //valor inicla para que el servo inicie en 90 
    //CCP1CONbits.DC1B = 0b11; ///BITS menos significativos
    
    PIR1bits.TMR2IF = 0; //limpiar bandera del tmr2
    T2CONbits.T2CKPS = 0b11; //prescalr 16
    T2CONbits.TMR2ON = 1; //encender el tmr2
    
    while (!PIR1bits.TMR2IF);//ciclo de espera
    PIR1bits.TMR2IF = 0; //limpoar la bandera del tmr2
    
    TRISCbits.TRISC2 = 0; //habilitar salida en rc2
    TRISCbits.TRISC1 = 0; //habilitar salida en rc1
    return;

}

void tmr0_setup(void){
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS2 = 0;
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;
    
    INTCONbits.T0IF = 0; //limpiar bandera de interrupcion 
    TMR0 = tmr0_val;
    return;
    
}

/////////////////////////////////////////////
///////////////potenciometros, pwm y manual
////////////////////////////////////////////////////

////funcion de mapeo e valores
unsigned short cambiopwm(uint8_t valor, uint8_t POTMIN, uint8_t POTMAX,
        unsigned short PWMMIN, unsigned short PWMMAX){
    return (unsigned short)(PWMMIN+((float)(PWMMAX-PWMMIN)/(POTMAX-POTMIN))
            *(valor-POTMIN));
}


//multiplexado para controlar los dos servos adicionales 
void manualPWM_ISR(void) {
        PORTCbits.RC3 = 1; //encender led
        delay(pot3); // delay (tiempo en alto del pulso)
        PORTCbits.RC3 = 0; //apagar
        
        PORTCbits.RC4 = 1; //encender puerto
        delay(pot4); // delay (tiempo en alto del pulso)
        PORTCbits.RC4 = 0; //apagar  
        
        INTCONbits.T0IF = 0; //limpia bandera del tmr0
        TMR0 = tmr0_val;
}

//FUNCION DE DELAY VARIABLES
void delay(unsigned int micro){
    while (micro > 0){
        __delay_us(250); //delay de 0.25ms
        micro--; //decrementar variable
    }
}

//----------funciones para eeprom-----------------------

void EEPROMWRITE(uint8_t address, uint8_t data){
    EEADR = address;//asignar direccin de datos 
    EEDAT = data;//datos 
    
    EECON1bits.EEPGD = 0; //escribe en la memoria de datos
    EECON1bits.WREN = 1; // habilita escritura en eeprom 
    
    INTCONbits.GIE = 0; //deshabilita las interrupciones 
    

    //obligatorio
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //habilitar escritua
    
    EECON1bits.WREN = 0; //apagamos la escritura
    
    INTCONbits.RBIF = 0; // limpiamos bandera en el puerto b
    INTCONbits.GIE = 1; //habilita interrupciones globales 
            
}

uint8_t EEPROMREAD(uint8_t address){
    EEADR = address ;//asgina la direccin 
    EECON1bits.EEPGD = 0;//selecciona la menoria eeprom
    EECON1bits.RD = 1;//habilita lectura de eeprom
    return EEDAT;//retorna el valor de la direccion leida
}

//Funcion para mostrar texto
void cadena(char *cursor){
    while (*cursor != '\0'){//mientras el cursor sea diferente a nulo
        while (PIR1bits.TXIF == 0); //mientras que se este enviando no hacer nada
            TXREG = *cursor; //asignar el valor del cursor para enviar
            *cursor++;//aumentar posicion del cursor
    }
}