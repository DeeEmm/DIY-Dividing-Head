/********************* 
 * Digital dividing head by DeeEmm AKA Michael Percy.
 * deeemm@deeemm.com
 * 31.05.2016
 * www.fabribles.com
 * 
 * Credits: Encoder interrupt routine by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt and Steve Spence, and code from Nick Gammon
 */


/*********************
 * Includes
 */
 
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 
// Using version 1.2.1 
// The LCD constructor - address shown is 0x27 - may or may not be correct for yours 
// Also based on YWRobot LCM1602 IIC V1 
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

 
/*********************
 * General declarations 
 */
const int TRUE = 1;
const int FALSE = 0;
const int OFF = 0;
const int ON = 1;

/*********************
 * Rotary encoder declarations 
 */
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent 
volatile int encoderPos = 0; //this variable stores our current value of encoder position. 
volatile int oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed 
volatile int reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

/*********************
 * LCD declarations 
 */
char LCDline1[16] = "               ";
char LCDline2[16] = "               ";

/*********************
 * Button reading, including debounce without delay function declarations
 */
const byte buttonPin = 8; // this is the Arduino pin we are connecting the push button to
const unsigned long debounceTime = 10;  // milliseconds
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
unsigned long buttonPressTime;  // when the switch last changed state
boolean buttonPressed = 0; // a flag variable
boolean debounce = 0;

/*********************
 * Menu and submenu/setting declarations
 */
int topMenu = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)
int subMenu = 0;
int maxSubMenu = 1;
int maxTopMenu = 1;
boolean inSubMenu = 0;
boolean inSubMenuEdit = 0;
boolean inMenuItem = 0;
boolean editDivs = 0;
boolean editDegs = 0;
boolean editSpeed = 0;
boolean editSteps = 0;
boolean jog = 0;

/*********************
 * Dividing head declarations
 */
const int DIVS = 1;
const int DEGS = 2;
const int FWD = 1;
const int REV = 2;
int jogMode = 0;
int jogPos = 0;
int jogUnits = DIVS; //operating mode - 1=divs 2=degs
int jogSpeed = 1; //speed at which to jog - 1x / 5x / 10x
int jogDirection = FWD; //default direction for jog
int degreesPerInc = 45; //number of degrees to move for each increment
int divisionsPerRot = 6; // number of increments per 360 degrees.

/*********************
 * Stepper motor declarations
 */
int stepsPerDegree = 50; //<- THIS IS PROBABLY THE ONLY SETTING YOU WILL NEED TO CHANGE !!!

int stepsPerInc = 0;
byte stepEnable = 10;
byte stepDir = 11; //Digital Pin 2 outputs Step Direction
byte stepGo = 12; //Digital Pin 3 outputs Step Execution
int stepDelay = 5; // default delay value in milliseconds
int stepWidth = 20; // default delay value in milliseconds
int divs = 0; // Holds the number of divisions
int pos = 0; // Holds the current dividing head position (0 (zero) based)
int lastpos = 0; // Last absolute stepper position
int nextstep = 0; // Used for calculation of next stepper position
int laststep = 0; // Holds the last stepper position
int nsteps = 0; // Count of steps to advance stepper
int intsteps = 0;
float stepsDiv = 0;   // Steps per Division
float numSteps = 1000.0; // Holds either Rotary Table or Dividing Head steps per rev




/*********************
 * Initialisation
 */
void setup() {
  
  //Assign I/O
  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP); 
  attachInterrupt(0,PinA,RISING); 
  attachInterrupt(1,PinB,RISING); 
  pinMode (buttonPin, INPUT); 
  pinMode (stepDir, OUTPUT);
  pinMode (stepGo, OUTPUT);
  pinMode (stepEnable, OUTPUT);

  Serial.begin(115200);     

  lcd.begin(16,2);  
  lcd.backlight();     
  writeDisplay(1,"Dividing Head V3");
}



/*********************
 * write to the (16x2) display
 */
void writeDisplay(const int LCDrow, const char *LCDtext) {
  lcd.setCursor(0,LCDrow-1);    
  lcd.print(LCDtext);
  lcd.print("                      ");
}



/*********************
 * Rotary encoder interrupt service routines
 */
void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; 
    aFlag = 0; 
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; 
    aFlag = 0; 
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

/*********************
 * Step routine
 */
void step (int dir)
{

  if (jogDirection != 1) {
    dir = dir * -1;
  }
  
  if(dir > 0) {
    nsteps = dir;
    digitalWrite(stepDir,LOW);
  } else {
    nsteps = dir * -1;
    Serial.println("neg step:");
    Serial.println(nsteps);    
    digitalWrite(stepDir,HIGH);
  }
  
  while(nsteps > 0) 
  {
    Serial.println(nsteps);
    digitalWrite(stepGo,HIGH);
    delayMicroseconds (stepWidth);
    digitalWrite(stepGo,LOW);
    delay(stepDelay);
    nsteps--;
  }
}


/*********************
 * Main loop
 * 
 * Two-Tier menu system operated from rotary encoder
 * Click to enter menu
 * Rotate to select menu item
 * Click to change setting
 * To exit sub menu scroll down to 'Exit'
 * 
 */
void loop() {

  //update encoder position
  if(oldEncPos != encoderPos) { 
    oldEncPos = encoderPos;
  } 
  
  //Top menu items (add additional cases for each top menu item you need)
  if (inSubMenu == FALSE) {
    subMenu = 1;
    maxTopMenu = 2;
    switch (topMenu) {
      case 1:
        if (buttonPressed) changeMenuMode();
        writeDisplay(2, "[Index]     Set   ");
        subMenu = 1;
        break;
      case 2:
        if (buttonPressed) changeMenuMode();
        writeDisplay(2, " Index     [Set] ");
        subMenu = 2;
        break;
      default:    
      
      break;
    }
  }


  //Submenu items (replicate for each submenu you need)

  //submenu 1 - Index / jog
  if (inSubMenu == TRUE && topMenu == 1){
    maxSubMenu = 5;
    switch (subMenu) {
      
      case 1:
        if (buttonPressed && debounce == OFF) {
          jogPos ++; 
          debounce = ON; 
          if (jogUnits == DIVS){
            stepsPerInc = (360 / divisionsPerRot) * stepsPerDegree;
            step(stepsPerInc);                                        
          } else {
            stepsPerInc = degreesPerInc * stepsPerDegree;
            step(stepsPerInc);                                        
          }
        }
        if (jogUnits == DIVS){
          if (encoderPos > divisionsPerRot) encoderPos = divisionsPerRot;
          lcd.setCursor(0,0); 
          lcd.print( "Mode: Division  ");    
          lcd.setCursor(0,1); 
          lcd.print( "Divison         ");
          lcd.setCursor(8,1); 
          lcd.print(jogPos);
          lcd.setCursor(11,1); 
          lcd.print( "/");
          lcd.setCursor(13,1); 
          lcd.print(divisionsPerRot);
        } else {
          if (encoderPos > 360 / degreesPerInc) encoderPos = 360 / degreesPerInc;
          lcd.setCursor(0,0); 
          lcd.print( "Mode: Degrees  ");    
          lcd.setCursor(0,1); 
          lcd.print( "Degrees:         ");
          lcd.setCursor(10,1); 
          lcd.print(jogPos);
          lcd.setCursor(11,1); 
          lcd.print( "x");
          lcd.setCursor(13,1); 
          lcd.print(degreesPerInc);
        }
      break;
     
      case 2:
          lcd.setCursor(0,1); 
          if (buttonPressed && debounce == OFF) {
            if (jogUnits == DIVS){
              stepsPerInc = (360 / divisionsPerRot) * stepsPerDegree;
              step(stepsPerInc);                                        
            } else {
              stepsPerInc = degreesPerInc * stepsPerDegree;
              step(stepsPerInc);                                        
            }
            jogPos ++; 
            debounce = ON; 
          }
          if (jogUnits == DIVS){
            lcd.print( "Jog 1 Div CW    ");     
          } else {
            lcd.print( "Jog 1 Unit CW   ");   
          }
      break;

      case 3:
          lcd.setCursor(0,1); 
          if (buttonPressed && debounce == OFF) {
            if (jogUnits == DIVS){
              lcd.print( "Jog 1 Div CCW   ");     
              stepsPerInc = (360 / divisionsPerRot) * stepsPerDegree;
              step(stepsPerInc * -1);                                        
            } else {
              lcd.print( "Jog 1 Unit CCW  ");   
              stepsPerInc = (360 / divisionsPerRot) * stepsPerDegree;
              step(stepsPerInc * -1);                                        
            }
            jogPos --; 
            debounce = ON; 
          }
          if (jogUnits == DIVS){
            lcd.print( "Jog 1 Div CCW   ");     
          } else {
            lcd.print( "Jog 1 Unit CCW  ");   
          }
      break;
   
      
      case 4: //Free jog
        lcd.setCursor(0,0); 
        lcd.print( "Free-Jog:Degrees");    
        if (jogMode == FALSE && buttonPressed == TRUE && debounce == OFF ) {
          jogMode = TRUE;
          inSubMenuEdit = TRUE;
          debounce = ON;
        }
        if (jogMode == TRUE) {
          if (buttonPressed == TRUE && debounce == OFF) {
            jogMode = FALSE;
            inSubMenuEdit = FALSE;
            inSubMenu = TRUE;
            encoderPos = 2;
            subMenu = 2;
            debounce = ON;
            oldEncPos = encoderPos;
          }
          if (encoderPos != oldEncPos) {
            step(jogSpeed * oldEncPos - encoderPos);   
            jogPos = oldEncPos - encoderPos;
            oldEncPos = encoderPos;
          }
          debounce = ON;      
          lcd.setCursor(0,1); 
          lcd.print( "Degrees:        ");
          lcd.setCursor(10,1); 
          lcd.print(jogPos);
        } else {
          lcd.setCursor(0,1); 
          lcd.print( "Press to Enable ");    
        }      
      break;
  
      
      case 5:
        lcd.setCursor(0,0); 
        lcd.print( "                ");    
        writeDisplay(2, "Go to Main Menu ");       
        if (buttonPressed && debounce == OFF) {
          changeMenuMode();
          debounce = ON;
        }
      break;
    }
  }

  //Submenu 2 - Setup
  if (inSubMenu == TRUE && topMenu == 2){
    maxSubMenu = 8;
    lcd.setCursor(0,0); 
    lcd.print( "Settings        ");    

    switch (subMenu) {
      
      case 1:
        if (jogUnits == DIVS) {
          if (buttonPressed == TRUE && debounce == OFF ) {
            jogUnits = DEGS;
            jogPos = 0;
          }
          writeDisplay(2, "Mode : Division ");
          debounce = ON;
        }
        if (jogUnits == DEGS) {
          if (buttonPressed == TRUE && debounce == OFF) {
            jogUnits = DIVS;
            jogPos = 0;
          }
          writeDisplay(2, "Mode : Degrees  ");
          debounce = ON;
        }      
      break;
      
      case 2: //Set degrees per segment
        if (editDegs == FALSE && buttonPressed == TRUE && debounce == OFF) {
          editDegs = 1;
          inSubMenuEdit = TRUE;
          encoderPos = degreesPerInc;
          debounce = ON;
        }
        if (editDegs == TRUE) {
          if (encoderPos > 360) encoderPos = 360; 
          if (encoderPos < 0) encoderPos = 0;
          degreesPerInc = encoderPos;
          if (buttonPressed == TRUE && debounce == OFF) {
            editDegs = 0;
            inSubMenuEdit = 0;
            inSubMenu = TRUE;
            encoderPos = 2;
          }
          debounce = ON;          
          lcd.setCursor(0,1); lcd.print("Degs / Unit->");
        } else {
          lcd.setCursor(0,1); lcd.print("Degs / Unit: ");         
        }
        lcd.setCursor(12,1); lcd.print(degreesPerInc);
        if (degreesPerInc < 10) lcd.setCursor(13,1); lcd.print("  ");
        if (degreesPerInc > 9 && degreesPerInc < 100) lcd.setCursor(14,1); lcd.print("   ");
        if (degreesPerInc > 99) lcd.setCursor(15,1); lcd.print("    ");
      break;
      
      case 3: //Set Divisions per segment
        if (editDivs == FALSE && buttonPressed == TRUE && debounce == OFF ) {
          editDivs = TRUE;
          inSubMenuEdit = TRUE;
          encoderPos = divisionsPerRot;
          debounce = ON;
        }
        if (editDivs == TRUE) {
          if (encoderPos > 3600) encoderPos = 3600; 
          if (encoderPos < 0) encoderPos = 0;
          divisionsPerRot = encoderPos;
          if (buttonPressed == TRUE && debounce == OFF) {
            editDivs = FALSE;
            inSubMenuEdit = FALSE;
            inSubMenu = TRUE;
            encoderPos = 3;
          }
          debounce = ON;          
          lcd.setCursor(0,1); lcd.print("Divisions->");
        } else {
          lcd.setCursor(0,1); lcd.print("Divisions: ");         
        }
        lcd.setCursor(11,1); lcd.print(divisionsPerRot);
        if (divisionsPerRot < 10) lcd.setCursor(12,1); lcd.print("        ");
        if (divisionsPerRot > 9 && divisionsPerRot < 100) lcd.setCursor(13,1); lcd.print("       ");
        if (divisionsPerRot > 99) lcd.setCursor(14,1); lcd.print("      ");    
      break;
      
      case 4: //Set steps per degree
        if (editSteps == FALSE && buttonPressed == TRUE && debounce == OFF ) {
          editSteps = TRUE;
          inSubMenuEdit = TRUE;
          encoderPos = stepsPerDegree;
          debounce = ON;
        }
        if (editSteps == TRUE) {
          if (encoderPos < 0) encoderPos = 0;
          stepsPerDegree = encoderPos;
          if (buttonPressed == TRUE && debounce == OFF) {
            editDivs = FALSE;
            inSubMenuEdit = FALSE;
            inSubMenu = TRUE;
            encoderPos = 3;
          }
          debounce = ON;          
          lcd.setCursor(0,1); lcd.print("Steps / Deg->");
        } else {
          lcd.setCursor(0,1); lcd.print("Steps / Deg: ");
        }
        lcd.setCursor(13,1); lcd.print(stepsPerDegree);
        if (stepsPerDegree < 10) lcd.setCursor(14,1); lcd.print("        ");
        if (stepsPerDegree > 9 && stepsPerDegree < 100) lcd.setCursor(15,1); lcd.print("       ");
        if (stepsPerDegree > 99) lcd.setCursor(16,1); lcd.print("      ");    
      break;
      
      case 5:
        if (jogDirection == FWD) {
          if (buttonPressed == TRUE && debounce == OFF ) jogDirection = REV;
          writeDisplay(2, "Direction: CW   ");          
          debounce = 1;
        }
        if (jogDirection == REV) {
          if (buttonPressed == TRUE && debounce == OFF) jogDirection = FWD;
          writeDisplay(2, "Direction: CCW  ");          
          debounce = ON;
        }
      
      break;

       case 6: //Set jog speed
        lcd.setCursor(0,0); 
        lcd.print( "Set Jog Speed   ");    
        if (editSpeed == FALSE && buttonPressed == TRUE && debounce == OFF ) {
          editSpeed = TRUE;
          inSubMenuEdit = TRUE;
          encoderPos = jogSpeed;
          debounce = ON;
        }
        if (editSpeed == TRUE) {
          if (encoderPos > 5) encoderPos = 5; 
          if (encoderPos < 1) encoderPos = 1;
          jogSpeed = encoderPos;
          if (buttonPressed == TRUE && debounce == OFF) {
            editSpeed = FALSE;
            inSubMenuEdit = FALSE;
            inSubMenu = TRUE;
            encoderPos = 1;
            debounce = ON;
          }
          debounce = ON;          
          if (jogSpeed == 1) writeDisplay(2, "Jog Speed-> 1x");    
          if (jogSpeed == 2) writeDisplay(2, "Jog Speed-> 2x");    
          if (jogSpeed == 3) writeDisplay(2, "Jog Speed-> 5x");    
          if (jogSpeed == 4) writeDisplay(2, "Jog Speed-> 10x");               
          if (jogSpeed == 5) writeDisplay(2, "Jog Speed-> 20x");               
        } else {
          if (jogSpeed == 1) writeDisplay(2, "Jog Speed:  1x");    
          if (jogSpeed == 2) writeDisplay(2, "Jog Speed:  2x");    
          if (jogSpeed == 3) writeDisplay(2, "Jog Speed:  5x");    
          if (jogSpeed == 4) writeDisplay(2, "Jog Speed:  10x");               
          if (jogSpeed == 5) writeDisplay(2, "Jog Speed:  20x");               
        }    
      break;     

      case 7:
        lcd.setCursor(0,0); 
        lcd.print( "                ");    
        writeDisplay(2, "RESET SETTINGS  ");       
        if (buttonPressed == TRUE && debounce == OFF) {
          int jogUnits = DIVS; //operating mode - 1=divs 2=degs
          int jogSpeed = 1; //speed at which to jog - 1x / 5x / 10x
          int jogDirection = FWD; //default direction for jog
          int degreesPerInc = 45; //number of degrees to move for each increment
          int divisionsPerRot = 6; // number of increments per 360 degrees.
          int stepsPerDegree = 1; // number of steps per degree.
        }
      break;
      
      case 8:
        lcd.setCursor(0,0); 
        lcd.print( "                ");    
        writeDisplay(2, "Go to Main Menu ");       
        if (buttonPressed == TRUE && debounce == OFF) {
          changeMenuMode();
          debounce = ON;
        }
      break;
    }
  }

    
 
  if (buttonPressed == FALSE) debounce = OFF;

  if (inSubMenu == FALSE && inMenuItem == FALSE) {
    if (encoderPos > maxTopMenu) encoderPos = maxTopMenu; 
    if (encoderPos <1) encoderPos = 1; 
    topMenu = encoderPos;
  } 
  
  if (inSubMenu == TRUE && inMenuItem == FALSE &! inSubMenuEdit) {
    if (encoderPos > maxSubMenu) encoderPos = maxSubMenu; 
    if (encoderPos <1) encoderPos = 1; 
    subMenu = encoderPos;
  } 

  // Button reading with non-delay() debounce - thank you Nick Gammon!
  byte buttonState = digitalRead (buttonPin); 
  
  if (buttonState != oldButtonState){
    if (millis () - buttonPressTime >= debounceTime){ // debounce
      buttonPressTime = millis ();  // when we closed the switch 
      oldButtonState =  buttonState;  // remember for next time 
      if (buttonState == LOW){ 
        buttonPressed = FALSE;
      } else {
        buttonPressed = TRUE;  
      }  
    }  // end if debounce time up
  } // end of state change

} 

/*********************
 * toggle between main and sub menus
 */
void changeMenuMode(){
  if (buttonPressed == TRUE && inSubMenu == FALSE && debounce == OFF) {
    inSubMenu = TRUE;
    inMenuItem = FALSE;
    debounce = ON;
    subMenu = 1;
    lcd.clear();
  } else {
    inSubMenu = FALSE;
    inMenuItem = FALSE;
    topMenu = topMenu;
    encoderPos = topMenu;
    debounce = ON;
  }
}


