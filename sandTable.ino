/*
 * Code to control the Asheville Makers Makerfaire sand table.
 * 
 * This code is designed to run on an ATMEGA328P.
 * 
 * Serial Commands:
 *   NOTES - All commands are encapsulated within curly brackets {}. 
 *   
 * - Move = M[THETA],[RHO]
 *     where, [THETA] is a float with a resolution of 0.01degrees, and represents movement angle
 *            [RHO] is a float with a resolution of 0.01mm, and represents the radial distance from the center (note: table maximum = 457mm)
 * - Enable movement = START
 * - Disable movement = STOP
 * - Home the machine = HOME
 * - Clear the movement buffer = RESET (note: this will additionally stop movement)
 */

/****************************************************************************************************************/
/* DEFINE MACROS */
/****************************************************************************************************************/
#define MOTOR_GEAR_RATIO_T    20     //Gear ratio for Theta
#define MOTOR_GEAR_RATIO_R    1      //Gear ratio for Rho
#define MOTOR_STEPS_PER_REV   240UL  //Number of steps per motor revolution (this is assuming both motors have the same step ratio
#define MOTOR_MAX_RPM         400UL  //Maximum RPM of the motor
#define MOTOR_MAX_SPS         ((MOTOR_MAX_RPM * MOTOR_STEPS_PER_REV) / 60) //Maximum number of motor steps per second
#define GANTRY_STEPS_PER_REV  (MOTOR_STEPS_PER_REV * MOTOR_GEAR_RATIO_T)   //Number of steps to rotate the gantry one revolution (2pi radians)
#define GANTRY_STEPS_PER_RAD  (GANTRY_STEPS_PER_REV / (2.0 * PI))            //Number of steps to rotate 1 radian
#define GANTRY_RADIAL_LENGTH  1      //Gantry radial length, in mm
#define GANTRY_RADIAL_TPI     17     //Radial arm threads per inch
#define GANTRY_RADIAL_TPMM    (GANTRY_RADIAL_TPI / 25.4f)  //Convert inches to mm
#define GANTRY_STEPS_PER_MM   (MOTOR_STEPS_PER_REV * MOTOR_GEAR_RATIO_R * GANTRY_RADIAL_TPMM) //Number of steps per mm of radial arm length (rho)

#define PIN_MOTOR_THETA       1 //Pin to step the motor controlling the gantry rotation
#define PIN_MOTOR_THETA_DIR   2 //Pin to control the direction of gantry rotation
#define PIN_MOTOR_RHO         3 //Pin to step the motor controlling the shuttle radius (range)
#define PIN_MOTOR_RHO_DIR     4 //Pin to control the direction of the shuttle
#define PIN_END_STOP          5 //Pin to detect if an endstop has been hit
#define PIN_POS_0_THETA       6 //Pin to detect position 0 along the theta axis
#define PIN_POS_0_RHO         7 //Pin to detect position 0 along the rho axis

/* Note: TIMER1_PRESCALAR is used to set the timer speed. We use /8, which equates to (16Mhz/8)^-1 = 0.5us, because this divides evenly into any uSec integer value. 
 * This limits us to a range between approx 31 and 2,000,000 interrupt calls per second.
 */
#define TIMER1_CNTR_CTRL      (1 << WGM12) | (1 << CS11)  //WGM12 bit = Counter Timer Compare mode (reset counter once compare value is reached) 
                                                          //CS11 bit  = Prescalar bit value that coorelates to clk/8.
#define TIMER1_PRESCALAR_VAL  8                           //Prescalar divide-by value. This must match the bit value defined in TIMER1_CNTR_CTRL.
#define TIMER1_COMPARE        (F_CPU / (MOTOR_MAX_SPS * TIMER1_PRESCALAR_VAL)) //Calculates the timer compare value based on the CPU Frequency, prescalar, & max steps/second.
#define TIMER1_COMPARE_REG    (1 << OCIE1A)//0x6  //Value to set the TIMSK1 register to use the value from OCR1A

/****************************************************************************************************************/
/* STATIC VARIABLES */
/****************************************************************************************************************/
static int16_t  stepsRemainingTheta  = 0; //Number of steps remaining to move in the theta direction
static int16_t  stepsRemainingRho    = 0; //Number of steps remaining to move in the rho direction
static uint16_t isrPerTickTheta      = 0; //Number of ISR periods between theta motor steps
static uint16_t isrPerTickRho        = 0; //Number of ISR periods between rho motor steps
static bool enableMotors    = false;      //Enable motors
static bool endStopDetected = false;      //End stop detected
static bool bufferFull      = false;      //This gets set to true when the movement buffer is full

//Setup / initialize hardware
void setup()
{
  //Initialize GPIO
  pinMode(PIN_MOTOR_THETA, OUTPUT);
  pinMode(PIN_MOTOR_THETA_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RHO, OUTPUT);
  pinMode(PIN_MOTOR_RHO_DIR, OUTPUT);
  pinMode(PIN_END_STOP, INPUT);
  pinMode(PIN_POS_0_THETA, INPUT);
  pinMode(PIN_POS_0_RHO, INPUT);
  
  //Initialize / setup TIMER1
  initTimer1();

  //Initialize Serial connection
  Serial.begin(19200);
  Serial.setTimeout(10);
}

//Main program loop - handles serial commands/parsing, and setting motor movement parameters
void loop()
{
  uint16_t   command;
  //uint16_t movementList[10];
  char       readByte;
  String     cmdStr;

  //Check to ensure that we haven't hit an endstop
  if(digitalRead(PIN_END_STOP) == HIGH)
  {
    endStopDetected = true;
  }
  
  //Check serial for new commands
  while (Serial.available()) {
    readByte = Serial.read();

    if(readByte == '{')
    {
      //Clear the command string buffer - start of new command
      cmdStr = "";
    }
    else if(readByte == '}')
    {
      parseCommand(cmdStr);        
    }
    else
    {
      //Add byte to command string buffer
      cmdStr += readByte;
    }
  }

  //Temporary delay for debugging purposes - will want this to be smaller
  delay(1000);
}

void parseCommand(String cmdStr)
{
  bool cmdValid = true; //Note: Ideally, this will be set to false here, and true upon a valid command

  //Serial.println(cmdStr);
  
  //End of command - do stuff
  if(cmdStr.charAt(0) == 'M')
  {
    //Movement command is in the form of Mx,y where x and y are theta and rho respectively
    float  movementTheta = 0;
    float  movementRho   = 0;    
    
    //!!! Add some error checking in here to make sure there are 2 values present, else set cmdValid to false.
    
    //Remove 'M' from movement command
    cmdStr.remove(0,1);

    //Parse out the theta component
    movementTheta = cmdStr.toFloat();

    //Remove first number and delimeter
    cmdStr = cmdStr.substring(cmdStr.indexOf(",")+1);

    //Parse out the rho component
    movementRho   = cmdStr.toFloat();

    //Convert to motor steps
    stepsRemainingTheta = movementTheta * GANTRY_STEPS_PER_RAD;
    stepsRemainingRho = movementRho * GANTRY_STEPS_PER_MM;
  }
  else if(cmdStr.equals("HOME"))
  {
    homeRoutine();
  }
  else if(cmdStr.equals("START"))
  {
    enableMotors = true;
  }
  else if(cmdStr.equals("STOP"))
  {
    enableMotors = false;
  }
  else if(cmdStr.equals("RESET"))
  {
    //Clear movement list
  }
  else
  {
    cmdValid = false;
  }

  //Send acknowledgement
  if(cmdValid == false)
  {
    Serial.println("{ERR}");
  }
  else
  {
    if(bufferFull == false)
    {
      Serial.println("{RDY}");
    }
    else
    {
      Serial.println("{WAIT}");
    }
  }
}

void homeRoutine()
{
  
}

//Initialize Timer1 to be used in conjunction with an ISR for motor timing
void initTimer1()
{
  //Disable interrupts until timer setup is complete
  cli();

  //Clear the timer & count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
 
  //Set the timer mode & prescalar value.
  TCCR1B |= TIMER1_CNTR_CTRL;
  
  //Enable & set compare match to register OCR1A
  TIMSK1 |= TIMER1_COMPARE_REG;
  
  //Set the compare value
  OCR1A = TIMER1_COMPARE;
  
  //Reenable interrupts
  sei();
}

/* This ISR is called periodically based on the settings for TIMER1, and is where motor stepping is handled.
 * !Note: Since this an ISR, this function needs to execute and exit quickly. Do not add any delays or prints.
 *        If "digitalWrite()"s are too slow, a better alternative would be to toggle the GPIO pin register directly.
 */
ISR(TIMER1_COMPA_vect)
{
  static int triggerStepTheta = 0;
  static int triggerStepRho   = 0;
  
  if(motorsEnabled == true)
  {
    if(endStopDetected == false) //temp
    {
       if(abs(stepsRemainingTheta) > 0)
       {
         if(stepRemainingTheta > 0)
         {
           digitalWrite(PIN_MOTOR_THETA_DIR, HIGH);
           stepsRemainingTheta--;
         }
         else
         {
           digitalWrite(PIN_MOTOR_THETA_DIR, LOW);
           stepsRemainingTheta++;
         }
         //Toggle the motor pins
         digitalWrite(PIN_MOTOR_THETA, HIGH);
         digitalWrite(PIN_MOTOR_THETA, LOW);
      }
    }
  }
}
