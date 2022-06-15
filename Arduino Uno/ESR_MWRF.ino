// Pulsed magnetic resonance of NV centers in diamond with a microcontroller
// By Giacomo Mariani, 2022
// Code for Arduino Uno board to measure the ESR spectrum with double driving of MW and RF signals

#include <util/delay.h> 
#include <SPI.h>

#define NOP __asm__ __volatile__ ("nop\n\t");             //define NOP operation, 62.5 ns delay. The same effect of the util/delay.h> library but takes more space in the sketch
//#define NOP2    NOP NOP
//#define NOP3    NOP NOP NOP
//#define NOP4    NOP NOP NOP NOP 
//#define NOP5    NOP NOP NOP NOP NOP 
//#define NOP6    NOP NOP NOP NOP NOP NOP
//#define NOP7    NOP NOP NOP NOP NOP NOP NOP 
//#define NOP8    NOP NOP NOP NOP NOP NOP NOP NOP
//#define NOP9    NOP NOP NOP NOP NOP NOP NOP NOP NOP
//#define NOP10   NOP NOP NOP NOP NOP NOP NOP NOP NOP NOP 

// defines for setting and clearing register bits. Prescaler use ADPS2,S21,S20 bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // set bit 0
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // set bit 1

// ADF4351 setting ////////////////////////////////////////////////////////////////
//  Arduino       PIN D8     ADF4351 MUXOUT
//  Arduino       PIN D9     ADF4351 LE
//  Arduino MOSI  PIN D11    ADF4351 DATA
//  Arduino SCK   PIN D13    ADF4351 CLK
#define ADF4351_LE 9    // LE pin of ADF4351
//  #define LOCK 8      // Input to detect lock-in from AD4351 (optional), a check of the lock can be performed after setting the ADF4351
uint32_t registers[6] =  {0x8F8008, 0x80080A1, 0x18005E42, 0x8004B3, 0x814004, 0x580005}; // Default values to initialize the ADF4351 registers (from Analog Devices software)

double RFout, INT, FRACF;                       // Parameters to set the range of frequency 
unsigned int long INTA, PDRFout,  MOD,  FRAC;   // Parameters to set the frequency, use the unsigned int long 32 bit to save easily on the register by using the shifts command >> <<    
byte OutputDivider;                             // Parameters to set the range of frequency    
double PFDRFout=10;                             // Default, reference frequency of the ADF4351 module on-board oscillator (10 MHz)
double RFint=2870.0;                            // Set frequency value to test ADF4351 with a spectrum analyzer, use in case of frequency check with + -


double RFstart=2779.50;             // Start microwave frequency for ESR scan in MHz
int MWstep=1;                       // Number of steps for the ESR scan
double RFstep=0.05;                 // Microwave frequency step  in MHz
// AD9834 setting ////////////////////////////////////////////////////////////////
//  Arduino       PIN A3     ADF4351 FSYNC
//  Arduino       PIN A4     ADF4351 SCLK
//  Arduino       PIN A5     ADF4351 SDATA
//  Arduino       PIN D6     ADF4351 RESET, 1 value (output off), 0 value (output on)
#define CTRL 0x2200                 // CONTROL REGISTER,Default sine wave, DB13=1 freq reg two consecutive writes, DB8 (PIN/SW=1) functions controlled by external pins, Phase and Freq registers controlled by pin, default is 0 because of pull down resistor on board
#define PHASEREG 0xC000             // PHASE0 REGISTER, default pull down resistor on board

#define FSYNC A3                    //Latch  
#define SCLK A4                     //Clock   
#define SDATA A5                    //Data
#define RES 6                       //Reset

#define SINE CTRL                   //Sine wave
#define SQUARE (CTRL||0x20)         //Square wave
#define TRIANGLE (CTRL||x2)         //Triangle wave
#define WAVE SINE
const float CRYSTAL = 67108864.0;   //Frequency of the crystal installed on the chosen module (Hz)   

unsigned long FREQ = 1000;          //Set output frequency in hz
unsigned long PHASE = 0;            //Set phase from 0-4095; for 0 value OUTB start with midscale
double factor = 1.5;                //RF Phase factor, 1.25 (-cos); 1.75 (+cos); 1.5 (+sin); 1 (-sin);

//unsigned long CCDstart = 1000000; //Start frequency in hz
//unsigned long CCDstep = 1000; //step frequnecy in hz
//unsigned long CCDpoints = 3000;

// Pulse sequence and measurement settings ////////////////////////////////////////////////////////////////
// PIN 2 - LASER PULSE
// PIN 3 - MICROWAVE PULSE
// PIN 4 - IVC102 S1
// PIN 5 - IVC102 S2
// PIN 6 - AD9834 TRIGGER (RESET)
// comment TAU on the first line for RF off or comment the second line for MW and RF on and synchronized
//#define TAU PORTD |= B01000000; _delay_us(0.1250); PORTD &= B10111111; _delay_us(0.500);   // Comment this line if the RF is on with microwave - Trigger RF throuh RESET PIN of AD9834 during waiting time of 1us before the MW PULSE
#define TAU _delay_us(0.8125);                                                               // Comment this line if the RF is off during the microwave scan

#define LP _delay_us(7.4375)        // Laser pulse duration in us, empty entry is 562.5 ns 
#define MWP _delay_us(3.8750);      // Microwave pulse duration in us, empty entry is 125.0 ns
#define TAU1                        // Delay between MW and laser pulse in us, empty entry is 62.5 ns

#define OFFALL  PORTD &= B11110011; PORTD |= B01000000;   //Define command to turn off Laser, Microwave and RF

#define smax 100    // Number of cycles for laser conditioning before the measurement
#define smid 1      // Number of cycles for laser conditioning between signal A and B during the measurement 

int rep = 2000;     // Number of measurement repetitions
#define tep 1700    // Number of cycles (gives the integration time of the IVC102), chosen to give below 5V for the ADC


// Initialize other variables
boolean LaserCond = true; //enable or disable leser conditioning before the measurement (recommended)
int i=0;
long PDA=0; long PDB=0;   //Store the output voltages of the IVC102 acquired by the ADC of Arduino
double TempPhase=0;       //Variable to calculate the RF phase of the AD9834
String datasent="";       //Output string of the data sent to the pc by serial monitor

// Setup section ////////////////////////////////////////////////////////////////
void setup() {
  
  // Initialize SPI bus
  SPI.begin();
    
  // Initialize AD9834
  pinMode(RES,OUTPUT);    pinMode(FSYNC,OUTPUT);    pinMode(SDATA,OUTPUT);    pinMode(SCLK,OUTPUT);
  digitalWrite(RES,LOW);  digitalWrite(FSYNC,HIGH); digitalWrite(SDATA,LOW);  digitalWrite(SCLK,HIGH);
  Initialize();           //Initialize AD9834
  delay(100);
  UpdateFreq(FREQ,PHASE); //Update frequency and phase of the RF signal
  digitalWrite(RES,HIGH); //Disable wave, only mid-scale DC output
    
  // Initialize ADF4351
  //pinMode(LOCK, INPUT);               // Not used because AD4351 has 3.3V output logic
  pinMode(ADF4351_LE, OUTPUT);          
  digitalWrite(ADF4351_LE, HIGH);       // Set LE high to block data trasfer
  SetADF4351(); 
  SetPower(1);                          // Set ouput power, possible values are 1,2,3,4
  PowerEnable(0);                       // Set power on (1), or off (0)
  SetFreq(RFstart);                     // Set up a starting frequency for test 
  delay(100);

// From ATmega datasheet, set prescaler to set ADC speed. Reading speed = ADC clock/Prescaler
//32 MHz (reading time tested with analogRead(A0) is about 30us)
sbi(ADCSRA,ADPS2);    // set bit 1 to ADPS2
cbi(ADCSRA,ADPS1);    // set bit 0 to ADPS1 
sbi(ADCSRA,ADPS0);    // set bit 1 to ADPS0
//16 MHz
//sbi(ADCSRA,ADPS2);  // set bit 1 to ADPS2
//cbi(ADCSRA,ADPS1);  // set bit 0 to ADPS1
//cbi(ADCSRA,ADPS0);  // set bit 0 to ADPS0
//4 MHz
//cbi(ADCSRA,ADPS2);  // set bit 0 to ADPS2
//sbi(ADCSRA,ADPS1);  // set bit 1 to ADPS1
//cbi(ADCSRA,ADPS0);  // set bit 0 to ADPS0
//8 MHz
//cbi(ADCSRA,ADPS2);  // set bit 0 to ADPS2
//sbi(ADCSRA,ADPS1);  // set bit 1 to ADPS1
//sbi(ADCSRA,ADPS0);  // set bit 1 to ADPS0

// Initialize pins for pulse sequence
DDRD   |= B00111100;    //Pin 2345 - Laser|MW|S1|S2
PORTD  |= B00010000;    //Reset mode IVC102 -  Laser off|MW off|S1 High|S2 Low
Serial.begin(250000);
}

// Loop waits to receive a character from serial monitor ////////////////////////////////////////////////////////////////
void loop() {
   if (Serial.available()>0) {
      char comm = Serial.read();
      commands(comm); // see the "void commands" function for the list of commands to start a measurement or test devices
      }
}

// Commands to write in the serial monitor to test devices or start a measurement ////////////////////////////////////////////////////////////////
void commands(char comm)
{
    if (comm == '+'){         //ADF4351 Frequency test, increased
    RFout=RFint+(double)RFstep; Serial.print("Frequency increased: "); Serial.println(RFout); delay(200); SetFreq(RFout);}
    else if (comm == '-'){    //ADF4351 Frequency test, decreased
    RFout=(RFint-(double)RFstep); Serial.print("Frequency decreased: "); Serial.println(RFout); delay(200); SetFreq(RFout);}

   else if (comm == '1'){     //Set ADF4351 power, entry 1,2,3,4 for -4 dbm, -1 dbm, +2 dbm, +5 dbm output power
   SetPower(1); Serial.println("Power -4dbm");}
   else if (comm == '2'){
   SetPower(2); Serial.println("Power -1dbm");}
   else if (comm == '3'){
   SetPower(3); Serial.println("Power +2dbm");}
   else if (comm == '4'){
   SetPower(4); Serial.println("Power +5dbm");}
   else if (comm == 'o'){
   PowerEnable(0); Serial.println("Power disabled");}
   else if (comm == 'p'){
   PowerEnable(1); Serial.println("Power enabled");}

   else if (comm == 'x'){SwitchTestON();}     //MW switch test, open
   else if (comm == 'z'){SwitchTestOFF();}    //MW switch test, close
   
   else if (comm == 'n'){                     //Start ESR measurement
   Serial.println("ESR with double driving start..."); Measure(); Serial.println("Measurement completed!");}  
   else if (comm == 'k'){                     //Laser beam test, for the alignment
   Serial.println("Laser allignement..."); LaserAllign(); } 

  }
  
// Effective 14 bit reading from oversampling, PIN A0 of Arduino connected to the output of the IVC102
void ReadB(){analogRead(A0); for (int i=0; i<256; i++){PDB+=0L+analogRead(A0);}}     
void ReadA(){analogRead(A0); for (int i=0; i<256; i++){PDA+=0L+analogRead(A0);}}     

// IVC102 commands for reset, pre-integration, and integration of the current from the photodiode
void ResetInt(){
        PORTD  &= B11011111;                                                    //S1 High|S2 Low  - Reset
        _delay_us(10.0000);  
        PORTD  |= B00100000;                                                    //S1 High|S2 High - Pre-integration
        _delay_us(10.0000);   
        PORTD  &= B11101111;                                                    //S1 Low |S2 High - Integration starts

        //Additional sequence (for safety) to remove the unwanted charges accumulated on the photodiode during laser conditioning when S1 is open
        _delay_us(5.0000);                                                       //Dummy integration of 5us
        PORTD  |= B00010000;                                                     //S1 High |S2 High - Hold
        _delay_us(10.0000);  
        PORTD  &= B11011111;                                                     //S1 High|S2 Low  - Reset
        _delay_us(10.0000); 
        PORTD  |= B00100000;                                                     //S1 High|S2 High - Pre-integration
        _delay_us(10.0000); 
        PORTD  &= B11101111;                                                     //S1 Low |S2 High - Integration starts
        }  

// Data output string, frequency, voltage with and without microwave for normalization
void dataout(){
 datasent=""; datasent=datasent+RFout+";"+PDA+";"+PDB+"\n"; Serial.print(datasent); Serial.flush();
}



// Measurement function of the ESR spectrum ////////////////////////////////////////////////////////////////
void Measure()  {
// Calculate correct RF phase for synchronization with the MW pulse since RF is triggered 0.5us before the MW pulse for stabilization.
TempPhase=(double)(factor-FREQ/(double)1000000.0*0.520)*4095;
if (TempPhase > 4095){
        if (TempPhase < 4095*2) {PHASE = (unsigned long)(TempPhase-4095);}
        else if (TempPhase > 4095*2){
                        for (int nn=2; nn<50; nn++){
                        if (TempPhase > 4095*nn && TempPhase < 4095*(nn+1)){PHASE = (unsigned long)(TempPhase-4095*nn);} 
                                                   }
                                     }
                     }
else if (TempPhase < 0){
        if (TempPhase > -4095){PHASE = (unsigned long)(TempPhase+4095);}
        else if (TempPhase < -4095  && TempPhase > -4095*2){PHASE = (unsigned long)(TempPhase+4095*2);}
        else if (TempPhase < -4095*2){
                        for (int nn=2; nn<50; nn++){
                        if (TempPhase < -4095*nn && TempPhase > -4095*(nn+1)){PHASE = (unsigned long)(TempPhase+4095*(nn+1));} 
                                                   }
                                     }
                        }
else{PHASE = (unsigned long)TempPhase;}
UpdateFreq(FREQ,PHASE); // Update the RF phase and frequency of the AD9834
delay(10);

// Preconditioning of the laser before starting the measurment
if (LaserCond==true){for (int f=0;f<smax;f++){Serial.print("Laser conditioning.."); Serial.println(f); LaserTest();}}

PowerEnable(1);           //Enable MW signal of AD4351 

// For loop for the number of repetitions
for (int r =0; r < rep; r++){
 Serial.print("Phase shift: "); Serial.print(PHASE); Serial.print("; Repetition number "); Serial.println(r+1); 
_delay_us(100000); 

              
              for (int u = 0; u< MWstep; u++){          // Scan the MW frequency of the AD4351 to measure the ESR spectrum
              RFout=RFstart+RFstep*(double)u;           // Calculate next MW frequency
              SetFreq(RFout);                           // Update frequency of the ADF4351

              cli();                                    //Disable interrupt
              
              for (int s=0;s<smid;s++){LaserTest();}    //Laser conditioning before signal A
              // SIGNAL A (MW ON)
              PDA=0; ResetInt(); PORTD |= B00000100; for (long f = 0; f < tep; f++){LP; PORTD &= B11111011; TAU; PORTD |= B00001000; MWP; PORTD &= B11110111; TAU1; PIND = B00000100;} OFFALL; PORTD  |= B00010000; ReadA(); PORTD  &= B11011111; 
              
              for (int s=0;s<smid;s++){LaserTest();}    //Laser conditioning before signal B
              // SIGNAL B (MW OFF)
              PDB=0; ResetInt(); PORTD |= B00000100; for (long f = 0; f < tep; f++){LP; PORTD &= B11111011; TAU; PORTD |= B00000000; MWP; PORTD &= B11110111; TAU1; PIND = B00000100;} OFFALL; PORTD  |= B00010000; ReadB(); PORTD  &= B11011111; 
              
              sei(); dataout();                         //Enable interrupt   

              digitalWrite(RES,HIGH);                   //Disable RF signal of AD9834
                                            }

                          }
digitalWrite(RES,HIGH);                                 //Disable RF signal of AD9834
PowerEnable(0);                                         //Disable MW signal of AD4351 
                }



// Other functions ////////////////////////////////////////////////////////////////

// Initialize the AD9834 
void Initialize(){
UpdateDDS(CTRL);
digitalWrite(RES,HIGH);
UpdateDDS(0x50C7);
UpdateDDS(0x4000);
UpdateDDS(PHASE);
digitalWrite(RES,LOW);
}

// Send data to the AD9834 
void UpdateDDS(unsigned int data){
  unsigned int pointer = 0x8000;
  digitalWrite(FSYNC,LOW);
  for (int i=0; i<16; i++){

   if ((data & pointer) > 0){
    digitalWrite(SDATA,HIGH);}
    else{digitalWrite(SDATA,LOW);}

    digitalWrite(SCLK,LOW);
    digitalWrite(SCLK,HIGH);
    pointer=pointer>>1;
    }
    digitalWrite(FSYNC,HIGH);
}

// Calculate and update the frequency and phase of the AD9834 
void UpdateFreq(long FREQ, long PHASE){
  long FTW=(FREQ*pow(2,28))/CRYSTAL;
  unsigned int MSB = (int)((FTW & 0xFFFC000)>>14);
  unsigned int LSB = (int)(FTW & 0x3FFF);

  unsigned int PBIT = (int)(PHASE & 0xFFF); // to be sure that the phase set in 12bit
  PBIT|= PHASEREG; //add the bit tp
  
  LSB |= 0x4000;
  MSB |= 0x4000;
  UpdateDDS(CTRL);
  UpdateDDS(LSB); UpdateDDS(MSB);
  UpdateDDS(PBIT);
  UpdateDDS(WAVE);
  }
  
//Optional, it can be change to improve laser stabilization during measurement depending on the laser diode and application  
void LaserTest(){  
_delay_us(100000);  
PORTD |= B00000100;                 // Laser on
for (long i = 0; i <=10000; i++){                           
_delay_us(0.5625);                  // Fill in laser pulse of 1 us
PORTD &= B11111011;                 // Laser off
_delay_us(9.0000);                  
PORTD ^= B00000100;                 // Laser on                                              
}
PORTD &= !B00001100;                // Laser OFF, MW OFF (for safety)
_delay_us(100000);  
}

//For the alignment, few seconds of laser beam with low average power to allign the spot to the sample
void LaserAllign(){ 
Serial.println("Laser on: ");
delay(100); 
cli(); 
for (long i = 0; i <= 10000; i++){   
PORTD |= B00000100;       // Laser on
_delay_us(5);                         
PORTD &= B11111011;       // Laser off
_delay_us(5000);  
}
PORTD &= B11111011;       // Laser off
sei();
delay(100);
Serial.println("Laser off: ");
}

//Turn microwave switch on, for test
void SwitchTestON(){ 
Serial.println("MW Switch ON: ");
delay(100);
cli(); 
PORTD |= B00001000;    
sei();
delay(100); 
}

//Turn microwave switch off, for test
void SwitchTestOFF(){
Serial.println("MW Switch OFF: ");
delay(100);
cli(); 
PORTD &= B11110111;   
sei();
delay(100); 
}

//Send the 32 bits of the register to the AD4351
void WriteRegister32(const uint32_t value)    // Register 32 bit
{
  SPI.setDataMode(SPI_MODE0);                 // See the AD4351 datasheet for SPI communication
  SPI.setBitOrder(MSBFIRST);                  
  digitalWrite(ADF4351_LE, LOW);              // Latch low starts the data transfer
  //delayMicroseconds(1);
  for (int i = 3; i >= 0; i--){               // Loop to send 32 bits as 8 bits four times
  SPI.transfer((value >> 8 * i) & 0xFF);      // Set offset, byte masking and send via SPI
  }
  digitalWrite(ADF4351_LE, HIGH);             // Latch high to complete the data transfer
  //delayMicroseconds(1);
}

// Program all the six registers of the AD4351
void SetADF4351()                             
{ for (int i = 5; i >= 0; i--){               // Registers writing starts with the last register
    WriteRegister32(registers[i]);
    }
delay(10);
}

//Enable or disable power of the ADF4351
void PowerEnable(byte val)
{
if (val==0){      //MW Disabled
 bitWrite (registers[4], 5, 0); 
  } 
if (val==1){      //MW Enabled
 bitWrite (registers[4], 5, 1);
  } 
SetADF4351();
}

//Set power of the ADF4351
void SetPower(byte val)
{
if (val==1){      //DB3, DB4 00 -4dbm
 bitWrite (registers[4], 4, 0);
 bitWrite (registers[4], 3, 0);  
  }
else if (val==2){ //DB3, DB4 01 -1dbm
 bitWrite (registers[4], 4, 0);
 bitWrite (registers[4], 3, 1);    
  }
else if (val==3){ //DB3, DB4 10 +2dbm
 bitWrite (registers[4], 4, 1);
 bitWrite (registers[4], 3, 0);     
  }
else if (val==4){ //DB3, DB4 11 +5dbm
 bitWrite (registers[4], 4, 1);
 bitWrite (registers[4], 3, 1);     
  }
SetADF4351();
}

// Set the output frequency of the ADF4351
void SetFreq(double val){          
RFout=val;
if (RFout > 4400 && RFout<35){}// Frequency not available  // Maximum and Minimum frequency of the ADF 
else {
    if (RFout >= 2200) {                                   //2200-4400 MHz is the recommended range with a fundamental frequency
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 2200) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 1100) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 550)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 275)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    }
    if (RFout < 137.5) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    }
    if (RFout < 68.75) {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }

    // Rfout = fpdf * (INTA +FRAC/MOD) Equation to use for the calculation
    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / RFstep);
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF);                  // Results is rounded in case of divisions with remainder

    // Register 0 is only to save FRAC and INT values
    registers[0] = 0;                     // Set all the bit zero and the control bits 000 to indicate register 0
    registers[0] = INTA << 15;            // Save the INTA in the register by using the shift
    FRAC = FRAC << 3;                     // Shift the FRAC to save 
    registers[0] = registers[0] + FRAC;   // Save the FRAC in the register

    // Register 1 is to save the MOD values, prescaler and phase value
    registers[1] = 0;                     // Reset the register for calculation, phase adjust DB28 remains set to zero
    registers[1] = MOD << 3;              // Store the MOD values
    registers[1] = registers[1] + 1 ;     // Add +1 to make the control bits 001 and indicate the register 1
    bitWrite (registers[1], 27, 1);       // Set prescaler of 8/9 on DB27
    
    // Set mux-out on digital-lock detect mode [110] for B28, B27, B26 bits (no need if it is already in default)
    bitWrite (registers[2], 28, 1);    
    bitWrite (registers[2], 27, 1);    
    bitWrite (registers[2], 26, 0);  
   
    SetADF4351();
    }   
RFint=RFout;
                            }
                        
