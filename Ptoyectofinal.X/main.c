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


#define _XTAL_FREQ 500000 //frecuencia de 500 kHZ
#define tmr0_val 246 //valor del timer0 para un per�odo de 20ms





//-----------------prototipos-----------------
void setup(void); //prototipo de setuo
void setupADC(void); //prototipo ADC
void setupPWM(void); //prototipo del PWM
void tmr0_setup(void);
void delay(unsigned int micro); //funci�n para obtener delay variable
void manualPWM_ISR(void);
void EEPROMWRITE(uint8_t address, uint8_t data); //lectura de eeprom
uint8_t EEPROMREAD(uint8_t address); //lectura del eeprom
void contmodo(void);
void uartsetup(void);// prototipo del uart
void serial(void);


int modo; //elegige el modo 
unsigned int SERVO1;
unsigned int SERVO2;
unsigned int SERVO3;
unsigned int SERVO4;

unsigned int pot3; //valor para tiempo en alto de PWM para intensidad del led
unsigned int pot4; //valor para tiempo en alto de PWM para intensidad del led
unsigned int address;


uint8_t uart_data; // datos recibidos del uart
uint8_t dato1; //dato para el servo 1 recibido de uart 
uint8_t dato2;//dato para el servo 2 recibido de uart 
uint8_t dato3;//dato para el servo 3 recibido de uart 
uint8_t dato4;//dato para el servo 4 recibido de uart 
uint8_t servo_selected = 0; // A�adir esta variable
uint8_t opcionservo = 0; //opcion del potenciometro

//-----------------mapeo de valores potenciometro 3 y 4--------
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax);
//prototipos 


////Rutina de interrupciones
void __interrupt() isr(void){
    
 ////interrupcion para primer potecniometro
    if (PIR1bits.ADIF){ //chequea interrupcion de adc
        PIR1bits.ADIF = 0; //limpia la bandera del adc
    }
    
    //interrupcion del tmr0
    if (INTCONbits.T0IF){
        manualPWM_ISR();
    }
    


    //interrupcion del portb
    if (INTCONbits.RBIF){//revisar las interrupciones del portb

        
        if (PORTBbits.RB1 == 0){//modo manual
            
            PORTDbits.RD4 = 1;// se enciende led para inicar que se guado
            EEPROMWRITE(0x10, SERVO1);
            EEPROMWRITE(0x11, SERVO2);
            EEPROMWRITE(0x12, pot3);
            EEPROMWRITE(0x13, pot4);


        }
        

        else if (PORTBbits.RB2 == 0){//boton para guradar valores en eeprom
            if (modo == 2){//si estamos en el modo 2 
             PORTDbits.RD1 = 1; //indica que los valores se guardaron 
             SERVO1 = EEPROMREAD(0x10); //guadar en memoria eepron el valor del prmer pot
             SERVO2 = EEPROMREAD(0x11); //guardar el valor del 2 pot
             pot3 = EEPROMREAD(0x12);//guardar el tercer pot
             pot4 = EEPROMREAD(0x13);//guadar el pot4
                }
            
        
            else {
                ;
            }
        
        }
        INTCONbits.RBIF = 0; 

    }
   
    //para modo interfaz uart
    if (modo == 3){
    if (PIR1bits.RCIF == 1){// revisar si hay interrucpcion
        serial();
    }
    }
    
    
//modo adafruit
    if (modo == 4){
         if (PIR1bits.RCIF == 1){// revisar si hay interrucpcion
            serial();
            }
       }
    

}


//////////main
void main(void){
    setup();//llamar a las configuraciones genreales
    setupADC();//LLamar a la configuracion del adc
    setupPWM();//configuracio del pwm
    tmr0_setup(); //configuraciones de tmr0
    uartsetup(); //configuraciones de uart 

    modo = 1; //modo inical es 1
    address = 0;
    
    //loop principal
        while (1){
            
            //------------------Escoger modo--------------
            if (PORTBbits.RB0 == 0){
                __delay_ms(20);
                while(PORTBbits.RB0 == 0){//antirrebote
                    ;
                }
                contmodo();//contador para elegir el modo
            }
            
            
            //modo 1 ----------------modo ual if (PORTBbits.RB0 == 0){//modo manual
            switch(modo){
                case(1):
                
                    PORTDbits.RD4 = 0;//limpia led de leer eeprom
                    PORTDbits.RD1 = 0;//limpia led de guardado
                    PORTDbits.RD2 = 1; //enciende led 
                    PORTDbits.RD3 = 1; //enciende led 
                    PORTDbits.RD5 = 0; //limpia led de adafruit
                    
                
                //-----------------canal---------------------
                    ADCON0bits.CHS = 0b0000;  // Chequea el canal 0
                    __delay_us(100);
                    ADCON0bits.GO = 1; //se inicia la conversion 
                    while (ADCON0bits.GO == 1){//espera a que se termine la conversion
                        ;
                    }
                
                    SERVO1 = map(ADRESH, 0, 255,3, 20); // mapaeo del potenciometro 1
                    CCPR1L = SERVO1; //pasar el valor del CPR1L al servo 1
                    __delay_us(100);
                
                 //---------------cana2---------------------------
                    ADCON0bits.CHS = 0b0010;  // Chequea el canal 1
                    __delay_us(100);
                    ADCON0bits.GO = 1; //se inicia la conversion 
                    while (ADCON0bits.GO == 1){//espera a que se termine la conversion
                        ;
                    }
                
                    SERVO2 = map(ADRESH, 0, 255, 3, 20); // mapaeo del potenciometro 1
                    CCPR2L = SERVO2; //pasar el valor del CPR1L al servo 1
                    __delay_us(100);
                

                //-------------canal 3-------------------
                    ADCON0bits.CHS = 0b0011; // Chequea el canal 1
                    __delay_us(100);
                    ADCON0bits.GO = 1; //se inicia la conversion 
                    while (ADCON0bits.GO == 1){//espera a que se termine la conversion
                        ;
                    }
                    pot3 = map(ADRESH, 0, 255, 2, 10); // mapaeo del potenciometro 3
                    __delay_us(100);

                
            //-------------------cana4----------
                    ADCON0bits.CHS = 0b0100;
                    __delay_us(100);
                    ADCON0bits.GO = 1; //se inicia la conversion 
                    while (ADCON0bits.GO == 1){//espera a que se termine la conversion
                        ;
                    }
                    pot4 = map(ADRESH, 0, 255, 2, 10); // mapaeo del potenciometro 4
                    __delay_us(100);
                
                    break;
                
                case(2): //modo eeprom
                    PORTDbits.RD4 = 0; //limpia led de leer eeprom
                    PORTDbits.RD1 = 0; //limpia el bits de guardado
                    PORTDbits.RD2 = 0 ; //apaga led
                    PORTDbits.RD3 = 1; //enciende led 
                    PORTDbits.RD5 = 0; //limpia led de adafruit

                    
                    //actualizar los valores de los pwm de los dos pot
                    CCPR1L = SERVO1;
                    CCPR2L = SERVO2;
                    //el valor de los otros dos siempre se esta reescriuendo 
                    
                    break;
                    
                case(3): //modo uart
                    PORTDbits.RD4 = 0; //limpia led de guadar eeprom
                    PORTDbits.RD1 = 0; //limpia led de leer eeprom
                    PORTDbits.RD2 = 1; //encinde led 
                    PORTDbits.RD3 = 0; //apaga led
                    PORTDbits.RD5 = 0; //limpia led de adafruit

   
                    break;
                    
                case(4): //modo adafruit
                    PORTDbits.RD5 = 1; //lenciende el led de adafruit
                    PORTDbits.RD4 = 0; //limpia led de guadar eeprom
                    PORTDbits.RD1 = 0; //limpia led de leer eeprom
                    PORTDbits.RD2 = 0; //encinde led 
                    PORTDbits.RD3 = 0; //apaga led       
   
                    break;
                
            }
        }

}

//////////////setup general

void setup(void){
    // --------------- Definir como digitales --------------- 
    ANSELH = 0; //puertos digitales 
    ANSELbits.ANS0 = 1; // ra0 como analogico
    ANSELbits.ANS2 = 1; //ra2 como analogico 
    ANSELbits.ANS3 = 1; //ra3 como analogico 
    ANSELbits.ANS4 = 1; //ra4 como analogico 

    
    // --------------- Configura puertos --------------- 
    TRISAbits.TRISA0 = 1; //puerto A0 como entrada
    TRISAbits.TRISA2 = 1; //puerto A1 como entrada (era este)
    TRISAbits.TRISA3 = 1; //puerto A1 como entrada (era este)
    TRISAbits.TRISA4 = 1; //puerto A1 como entrada (era este)
    
    //botones
    TRISBbits.TRISB0 = 1; //rb0 como entrada
    TRISBbits.TRISB1 = 1; //rb1 como entrada 
    TRISBbits.TRISB2 = 1; //rb2 como entrada 
    TRISBbits.TRISB3 = 1; //rb3 como entrada 
    TRISBbits.TRISB4 = 1; //rb3 como entrada 

    
    TRISCbits.TRISC3 = 0; //puerto C3 como salida (era este)
    TRISCbits.TRISC4 = 0; //puerto C4 como salida (era este)
    TRISEbits.TRISE2 = 1; //puerto C4 como salida (era este)

    
    //TRISE = 0x00; 
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
    WPUBbits.WPUB3 = 1; 
    WPUBbits.WPUB4 = 1; 

    
    // --------------- Oscilador --------------- 
    OSCCONbits.IRCF = 0b011 ; // establecerlo en 500kHz
    OSCCONbits.SCS = 1; // utilizar oscilador intern
    

    
    // --------------- INTERRUPCIONES --------------- 

    INTCONbits.GIE = 1; // habilitar interrupciones globales
    INTCONbits.PEIE = 1; // habilitar interrupciones perifericas
    INTCONbits.RBIE = 1; //habilitar interrupciones en portb
    PIE1bits.RCIE =  0; //Encender interrupciones del RECEPTOR UART
    PIR1bits.RCIF = 0; //limpiar banndaera de interrupcion del uart 
    
    IOCBbits.IOCB0 = 1; //habilitar interrupciones en rb0
    IOCBbits.IOCB1 = 1; // habilitar interrupciones en rb1
    IOCBbits.IOCB2 = 1; // habilitar interrupcion en el rb2
    IOCBbits.IOCB3 = 1; // habilitar interrupcion en el rb3
    IOCBbits.IOCB4 = 1; // habilitar interrupcion en el rb3

    INTCONbits.TMR0IE = 1;// activar interrupcion del tmr0
    //INTCONbits.T0IF = 0;
    //INTCONbits.T0IE;
    
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
    PIR1bits.ADIF = 0;
    __delay_ms(100);
}

//pr2,bits prescale y oscilador afectan al servo

//-------setup de PWM-------------------
void setupPWM(void){
  //--------------ccp1---------------    
    TRISCbits.TRISC1 = 1; //CCP1 como entrada
    TRISCbits.TRISC2 = 1; //CCP2 como entrada
    
    PR2 = 155 ;   //periodo de 4ms en el tmr2
    

    CCP1CONbits.P1M = 0b00; //modo de single output
    CCP1CONbits.CCP1M = 0b1100; //modo pwm para ccp1
    CCP2CONbits.CCP2M = 0b1111; //modo pwm para ccp2

    CCP1CONbits.DC1B = 0b11; //bis menos significativos para el tiempo en alto
    CCPR1L = 11;
    
    CCP2CONbits.DC2B0 = 0b1; //bis menos significativos para el tiempo en alto
    CCP2CONbits.DC2B1 = 0b1; 
    CCPR2L = 11; //valor asignado para oscilar para empezar en 0

    
    PIR1bits.TMR2IF = 0; //limpiar bandera del tmr2
    T2CONbits.T2CKPS = 0b11; //prescalr 16
    T2CONbits.TMR2ON = 1; //encender el tmr2
    
    while (!PIR1bits.TMR2IF){//ciclo de espera{
        ;
    }
    //PIR1bits.TMR2IF = 0; //limpoar la bandera del tmr2
    
    TRISCbits.TRISC2 = 0; //habilitar salida en rc2
    TRISCbits.TRISC1 = 0; //habilitar salida en rc1

}

void tmr0_setup(void){
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS2 = 0;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;//precaler 16
    INTCONbits.T0IF = 0; //limpiar bandera de interrupcion 
    TMR0 = tmr0_val;
    
}

/////////////////////////////////////////////
///////////////potenciometros, pwm y manual
////////////////////////////////////////////////////


//----------------------------------------mapeo de los pwm manuales
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax){ //funci�n para mapear valores
    return ((value - inputmin)*(outmax-outmin)) / (inputmax-inputmin)+outmin;} 


//---------------------------------multiplexado para controlar los dos servos adicionales 
void manualPWM_ISR(void) {
        TMR0 = tmr0_val;
        PORTCbits.RC3 = 1; //encender led
        delay(pot3); // delay (tiempo en alto del pulso)
        PORTCbits.RC3 = 0; //apagar
        
        PORTCbits.RC4 = 1; //encender puerto
        delay(pot4); // delay (tiempo en alto del pulso)
        PORTCbits.RC4 = 0; //apagar  
        
        INTCONbits.T0IF = 0; //limpia bandera del tmr0
        
}

//------------------------------------------------FUNCION DE DELAY VARIABLES
void delay(unsigned int micro){
    while (micro > 0){
        __delay_us(250); //delay de 0.25ms
        micro--; //decrementar variable
    }
}

//----------funciones para eeprom-----------------------

////----------funciones--------------

//-----------------------escritura de la eeprom---------------------------
void EEPROMWRITE(uint8_t address, uint8_t data){
    while(WR);
    EEADR = address;//asignar direccin de datos 
    EEDAT = data;//datos 
   
    EECON1bits.EEPGD = 0; //escribe en la memoria de datos
    EECON1bits.WREN = 1; // habilita escritura en eeprom 
    
    INTCONbits.GIE = 0; //deshabilita las interrupciones 

    //obligatorio
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1; //habilitar escritua
    
    while(PIR2bits.EEIF == 0);//  se pone a 1 cuando la escritura en la EEPROM ha terminado

    PIR2bits.EEIF = 0;//si se termina laescritura, limpiamos el bit EEIF

    EECON1bits.WREN = 0; //apagamos la escritura
    INTCONbits.RBIF = 0; // limpiamos bandera en el puerto b

    INTCONbits.GIE = 1;//extra
    //INTCONbits.GIE = 1; //habilita interrupciones globales 
    
}


//--------------------------lectura de la eeprom---------------------------
uint8_t EEPROMREAD(uint8_t address){
    
    // WR y RD son bits de estado que indican si una operaci�n de escritura o lectura esta pasandp}o

    while (WR||RD);
        
    EEADR = address ;//asgina la direccin 
    EECON1bits.EEPGD = 0;//selecciona la menoria eeprom
    EECON1bits.RD = 1;//habilita lectura de eeprom
    return EEDAT;//retorna el valor de la direccion leida
}

//--------------------------------contador para cambio de modo-----------------------------
void contmodo(void){
    if (modo != 4){
        modo ++; //incremeta el modo s
    }
    else{
        modo = 1;//el modo se pone en 1
    }
}

//------------------------------------setup del uart -----------------------------------------

void uartsetup(void){
    TXSTAbits.SYNC = 0;//asincrono
    TXSTAbits.BRGH = 1;//high baud rate select bit
    
    BAUDCTLbits.BRG16 = 1;//utilizar 16 bits baud rate
    
    SPBRG = 12; //configurar a 9615 era 25 y 103
    SPBRGH = 0;//estaba en 0    

    
    RCSTAbits.SPEN = 1;//habilitar la comunicacion serial
    RCSTAbits.RX9 = 0;//deshabiliamos bit de direccion
    //TXSTAbits.TX9 = 0; // se acaba de apagar--------------------
    RCSTAbits.CREN = 1;//habilitar recepcion 
    TXSTAbits.TXEN = 1;//habiliar la transmision   
}


void serial(void){
    if (RCREG == '1'){//Revisar si se envia un 1
        CCPR1L = 19; //valor maximo de brazo 1
        PIR1bits.RCIF = 0; //limpiar bandera
    }
    
    if (RCREG == '2'){//Revisar si se envia un 1
        CCPR1L = 4; //valor maximo de brazo 1
        PIR1bits.RCIF = 0; //limpiar bandera
    }
    
    if (RCREG == '3'){//Revisar si se envia un 3
        CCPR2L = 19; //valor maximo de brazo 2
        PIR1bits.RCIF = 0; //limpiar bandera
    }
    
    if (RCREG == '4'){//Revisar si se envia un 3
        CCPR2L = 4; //valor minimo de brazo 2
        PIR1bits.RCIF = 0; //limpiar bandera
    }
    
    if (RCREG == '5'){//Revisar si se envia un 3
        pot3 = 9; //valor minimo de garra 2
        PIR1bits.RCIF = 0; //limpiar bandera
    }
    
    if (RCREG == '6'){//Revisar si se envia un 3
        pot3 = 2; //valor minimo de brazo 2
        PIR1bits.RCIF = 0; //limpiar bandera
    }
    
    if (RCREG == '7'){//Revisar si se envia un 3
        pot4 = 9; //valor minimo de brazo 2
        PIR1bits.RCIF = 0; //limpiar bandera
    }
    
    if (RCREG == '8'){//Revisar si se envia un 3
        pot4 = 2; //valor minimo de brazo 2
        PIR1bits.RCIF = 0; //limpiar bandera
    }
}
    

