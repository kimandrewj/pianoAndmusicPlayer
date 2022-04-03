#include <Arduino_FreeRTOS.h>
#include <arduinoFFT.h>
#include <queue.h>
#include <LiquidCrystal.h>


////////////////////////////////////////////////
// APPROVED FOR ECE 474   Spring 2021
//
//  NOTE: modify analogRead() on line 113 according
//   to your setup.
////////////////////////////////////////////////

arduinoFFT FFT = arduinoFFT();


#define melodyPin 3

// define two tasks for Blink & AnalogRead
void BlinkTask( void *pvParameters );
void PlayTheme( void *pvParameters );
void SelectionScreen(void *pvParameters);
void selectMode(void *pvParameters);
//void TaskAnalogRead( void *pvParameters );
//void SpeakerTask( void *pvParameters );

const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
const int cNote =  30; 
const int dNote =  31; 
const int eNote =  32; 
const int fNote =  33; 
const int gNote =  34; 
const int aNote =  35; 
const int bNote =  36; 
const int playButton = 37;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// 2-6, 11, 12, 30-36

// Global timer
unsigned long int timer = 0;
int frequency;
int pot_pos = analogRead(A0);

int n = 32000;
double *inp;
double x[samples];
unsigned long int start;
unsigned long int measure;
#define DEF_TONE    OCR4A = ((16000000 / (frequency * 2 * 1)) - 1);
#define SPEAKER_OFF OCR4A = 0


//Mario main theme melody notes from https://www.hackster.io/jrance/super-mario-theme-song-w-piezo-buzzer-and-arduino-1cc2e4
int mario_melody[] = {
  330, 330, 0, 330,
  0, 262, 330, 0,
  392, 0, 0,  0,
  196, 0, 0, 0,

  262, 0, 0, 196,
  0, 0, 165, 0,
  0, 220, 0, 247,
  0, 233, 220, 0,

  196, 330, 392,
  440, 0, 350, 392,
  0, 330, 0, 262,
  294, 247, 0, 0,

  262, 0, 0, 196,
  0, 0, 165, 0,
  0, 220, 0, 247,
  0, 233, 220, 0,

  196, 330, 392,
  440, 0, 350, 392,
  0, 330, 0, 262,
  294, 247, 0, 0
};

//Mario main theme tempo
int mario_tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};


//Underworld melody
int underworld_melody[] = {
  262, 523, 220, 440, 
  233, 466, 0,
  0,
  262, 523, 220, 440, 
  233, 466, 0,
  0,
  175, 349, 147, 294,
  156, 311, 0,
  0,
  175, 349, 147, 294,
  156, 311, 0,
  0, 311, 277, 294,
  277, 311, 
  311, 208,
  196, 277,
  262, 370,349, 165, 466, 440,
  415, 311, 247,
  233, 220, 208,
  0, 0, 0
};
//Underwolrd tempo
int underworld_tempo[] = {
  12, 12, 12, 12, 
  12, 12, 6,
  3,
  12, 12, 12, 12, 
  12, 12, 6,
  3,
  12, 12, 12, 12, 
  12, 12, 6,
  3,
  12, 12, 12, 12, 
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18,18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988

int pirate_melody[] = {
    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
    NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
    NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,

    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_D5, NOTE_E5, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
    NOTE_C5, NOTE_A4, NOTE_B4, 0,

    NOTE_A4, NOTE_A4,
    //Repeat of first part
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
    NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
    NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,

    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_D5, NOTE_E5, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
    NOTE_C5, NOTE_A4, NOTE_B4, 0,
    //End of Repeat

    NOTE_E5, 0, 0, NOTE_F5, 0, 0,
    NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
    NOTE_D5, 0, 0, NOTE_C5, 0, 0,
    NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

    NOTE_E5, 0, 0, NOTE_F5, 0, 0,
    NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
    NOTE_D5, 0, 0, NOTE_C5, 0, 0,
    NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4};

// Durations (in ms) of each music note of the song
// Quarter Note is 250 ms when songSpeed = 1.0
int pirate_tempo[] = {
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 125, 250, 125,

    125, 125, 250, 125, 125,
    250, 125, 250, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 375,

    250, 125,
    //Rpeat of First Part
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 125, 250, 125,

    125, 125, 250, 125, 125,
    250, 125, 250, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 375,
    //End of Repeat

    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 125, 125, 125, 375,
    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 500,

    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 125, 125, 125, 375,
    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 500};


int song_completed = 1;
int selected = 0;

/*
  These are the input and output vectors
  Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

// declare the QUEUE handle
static QueueHandle_t queueOne; 
static QueueHandle_t queueTwo;

TaskHandle_t Rt3_0Handle, Rt3_1Handle, Rt4Handle; // address to the functions
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  //Serial.begin(19200);
  //Setting up the Timer 0 for timer compare
  TCCR3A = (1 << WGM31);    //Set the CTC mode
  OCR3A = 0xF9; //Value for ORC0A for 1ms
  TIMSK3 |= (1 << OCIE3A);   //Set the interrupt request
  sei(); //Start interrupt


  //Setting up the Timer 4 for waveform generation output
  DDRH |= (1 << DDH3); //OC4A output
  TCCR4A = 0; // Clear register bits
  TCCR4B = 0;
  TCCR4A |= (1 << COM4A0); // Setting toggle mode
  TCCR4B |= (1 << WGM42) | (1 << CS40); //Setting to CTC mode


  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  //  }

  //  xTaskCreate(
  //    SpeakerTask
  //    ,  "SpeakerTask"
  //    ,  128  // Stack size
  //    ,  NULL
  //    ,  1  // Priority
  //    ,  NULL );
  //
  //    xTaskCreate(
  //    RT3p0
  //    ,  "task3_0"
  //    ,  128  // Stack size
  //    ,  NULL
  //    ,  2  // Priority
  //    ,  &Rt3_0Handle );
  //
  //    xTaskCreate(
  //    RT4
  //    ,  "task4"
  //    ,  1024  // Stack size
  //    ,  NULL
  //    ,  2  // Priority
  //    ,  &Rt4Handle );

    xTaskCreate(
      BlinkTask
      ,  "Blink"   // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

    xTaskCreate(
      selectMode
      ,  "LCDControl"   // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

    xTaskCreate(
      SelectionScreen
      ,  "SpeakerControl"   // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );





      
  // Detect hand with Ultrasonic sensor task
  // If hand then piano mode otherwise in song mode
  // Everything is reset when switched -> Displays on LCD screen

  // Piano mode plays tone when button pressed
  // Shows note being played

  // Song mode is array of funcitons
  // Joystick will toggle between functions
  // LCD displays current selected song
  // Press button to start playing song


  delay(500);


  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  //  (note how the above comment is WRONG!!!)
  vTaskStartScheduler();


}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

/*
   Task RT-1
   Blink Task

*/
void BlinkTask(void *pvParameters)  // This is a task.
{
  // (void) pvParameters;  // allocate stack space for params

  /*
    Blink
    Turns on an LED on for one second, then off for one second, repeatedly.

    Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO
    it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care
    of use the correct LED pin whatever is the board used.

    The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
    the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
    e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.

    If you want to know what pin the on-board LED is connected to on your Arduino model, check
    the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products

    This example code is in the public domain.

    modified 8 May 2014
    by Scott Fitzgerald

    modified 2 Sep 2016
    by Arturo Guadalupi
  */

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void SongSelection(){
  if(selected == 1)
  {
    xTaskCreate(
      pianoTask
      ,  "Piano"
      ,  128  // Stack size
      ,  NULL
      ,  3  // Priority
      ,  NULL );      

      song_completed = 0;
  } else if(selected > 1) {
    xTaskCreate(
      PlayTheme
      ,  "Play"
      ,  128  // Stack size
      ,  NULL
      ,  3  // Priority
      ,  NULL );
     
     song_completed = 0;
  }
}

void selectMode(void *pvParameters) {
  for (;;) // A Task shall never return or exit.
  {
    //Serial.println(song_completed);
    if(song_completed == 1){
      int selection = analogRead(A0);
//      
//      Serial.println(selected);
//      Serial.println(selection);
        if (selection >= 0 && selection < 204) {
          // Which song would you to play (base state)
          lcd.begin(16, 2);
          lcd.print("What do you");
          lcd.setCursor(0,1);
          lcd.print("want to play?");
          selected = 0;
        } else if(selection >= 204 && selection < 408) {
          // Piano state
          lcd.begin(16, 2);
          lcd.print("Piano mode");
          selected = 1;
        } else if(selection >= 408 && selection < 610) {
          // song 1 state
          lcd.begin(16, 2);
          lcd.print("Play the");
          lcd.setCursor(0,1);
          lcd.print("Mario Theme");
          selected = 2;
        } else if(selection >= 610 && selection < 812) {
          // song 2 state
          lcd.begin(16, 2);
          lcd.print("Play the");
          lcd.setCursor(0,1);
          lcd.print("Underground Theme");
          selected = 3;
        } else if(selection >= 812 && selection < 1026){
          // song 3 state
          lcd.begin(16, 2);
          lcd.print("Play the");
          lcd.setCursor(0,1);
          lcd.print("Pirate Theme");
          selected = 4;
        }
    }
    vTaskDelay(100);
  }
}

void pianoTask(void *pvParameters) {
  for (;;) {
//    int playButtonState = digitalRead(playButton);
//    if (playButtonState == HIGH){
//      SPEAKER_OFF;
//      song_completed = 1;
//      selected = 0;
//      vTaskDelete( NULL );
//    }
    if (digitalRead(cNote) == 1) {
      // play C note 
      lim(262); 
      // Serial.println("C");
      vTaskDelay( 300 / portTICK_PERIOD_MS ); 
    } else if (digitalRead(dNote) == 1) {
      // play D note   
      lim(294); 
      // Serial.println("D");
      vTaskDelay( 300 / portTICK_PERIOD_MS );
    } else if (digitalRead(eNote) == 1) {
      // play E note   
      lim(330); 
      // Serial.println("E");
      vTaskDelay( 300 / portTICK_PERIOD_MS );
    } else if (digitalRead(fNote) == 1) {
      // play F note
      lim(350); 
      // Serial.println("F");
      vTaskDelay( 300 / portTICK_PERIOD_MS );   
    } else if (digitalRead(gNote) == 1) {
      // play G note  
      lim(392); 
      // Serial.println("G");
      vTaskDelay( 300 / portTICK_PERIOD_MS ); 
    } else if (digitalRead(aNote) == 1) {
      // play A note   
      lim(440); 
      // Serial.println("A");
      vTaskDelay( 300 / portTICK_PERIOD_MS );
    } else if (digitalRead(bNote) == 1) {
      // play B note 
      lim(494); 
      // Serial.println("B");
      vTaskDelay( 300 / portTICK_PERIOD_MS );  
    }
    SPEAKER_OFF;
  }
}

void SelectionScreen(void *pvParameters)  // This is a task.
{
  //Print to display
  for (;;) // A Task shall never return or exit.
  {
    int playButtonState = digitalRead(playButton);
    if(song_completed == 1)
    {
      if (playButtonState == HIGH) 
      {
        song_completed = 0; 
        SongSelection();
        vTaskDelay(2000);
        //selected = selected + 1; //remove this later
      }
    }

  }
}


/*
   Mario Theme Task
*/
void PlayTheme(void *pvParameters) {
  //lcd.begin(16, 2);
  //lcd.print("Playing the Mario Theme");
  
   // Add display output here
  for (;;) // A Task shall never return or exit.
  {
    if(selected == 2){
      for ( int i = 0; i < 78; i++) {
        lim(mario_melody[i]);
        vTaskDelay( (mario_tempo[i] * 10) / portTICK_PERIOD_MS );
      }
    } else if (selected == 3) {
      for ( int i = 0; i < 56; i++) {
        lim(underworld_melody[i]);
        vTaskDelay( (underworld_tempo[i] * 10) / portTICK_PERIOD_MS );
      }
    } else if (selected == 4) {
      for ( int i = 0; i < 230; i++) {
        lim(pirate_melody[i]);
        vTaskDelay( (pirate_tempo[i]) / portTICK_PERIOD_MS );
      }    
    }
    vTaskDelay(100);
    SPEAKER_OFF;
    song_completed = 1;
    //selected = 0;
    vTaskDelete( NULL );
    
  }
}



/*
   Speaker Task
*/
void SpeakerTask(void *pvParameters)
{


  for (;;) // A Task shall never return or exit.
  {

    lim(293);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(329);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(261);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(130);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(196);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    SPEAKER_OFF;
    vTaskDelay( 1500 / portTICK_PERIOD_MS );
    lim(293);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(329);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(261);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(130);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(196);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    SPEAKER_OFF;
    vTaskDelay( 1500 / portTICK_PERIOD_MS );
    lim(293);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(329);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(261);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(130);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    lim(196);
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    SPEAKER_OFF;
    vTaskDelete( NULL );


  }
}

/*
   RT3p0 Task
*/
void RT3p0(void *pvParameters) {
  for (int i = 0; i < samples; i++) {
    x[i] = randN(n);
  }
  inp = x;
  queueOne = xQueueCreate(2, sizeof(double));
  queueTwo = xQueueCreate(2, sizeof(double)); // one will send stuff task 3 to task 4
  vTaskDelete(Rt3_0Handle);
  xTaskCreate(
    RT3p1
    ,  "task3_1"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &Rt3_1Handle );
}

/*
   RT3p1 Task
*/
void RT3p1(void *pvParameters) {
  for (;;) {
    for (int i = 0; i < 5; i++) {
      xQueueSendToBack(queueOne, inp, 100); //
      xQueueReceive(queueTwo, inp, portMAX_DELAY); // block
    }
  }
}

/*
   RT4 Task
*/
void RT4(void *pvParameters) {
  static unsigned long int start;
  static unsigned long int measure;
  for (;;) {
    double buff *buff;
    xQueueReceive(queueOne, buff, 100);
    double cycles = (((samples-1) * signalFrequency) / samplingFrequency);
    for (uint16_t i = 0; i < samples; i++) {
      vReal[i] = int8_t((amplitude * (sin((i * (twoPi * cycles)) / samples))) / 2.0);/* Build data with positive and negative values*/
      //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
      vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    }
    start = millis();
    for (int i = 0; i < 5; i++) {
      FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
    }
    measure = millis();
    Serial.println(measure - start);
    xQueueSendToBack(queueTwo, inp, 100);
  }
}

void lcdControl(int inp) {
  if (inp == 0) {
    lcd.begin(16, 2);
    lcd.print("Piano Mode");
  }
  if (inp == 1) {
    // which song would you like to play?
    lcd.begin(16, 2);
    lcd.print("What song would");
    lcd.setCursor(0,1);
    lcd.print("you like to play");
  }
  if (inp == 2) {
    lcd.begin(16, 2);
    lcd.print("Mario");
  }
  if (inp == 3) {
    // Second Song
    lcd.begin(16, 2);
    lcd.print("Harry Potter");
  }
  if (inp == 4) {
    // Third Song
    lcd.begin(16, 2);
    lcd.print("Star Wars");
  }
  if (inp == 5) {
    // Fourth Song
    lcd.begin(16, 2);
    lcd.print("God Father");
  }
  if (inp == 6) {
    // Fourth Song
    lcd.begin(16, 2);
    lcd.print("Indiana Jones");
  }
}

int randN(int n) {
  double x;
  x = 1.0 + (double) n * rand() / RAND_MAX;
  return ((int)x);
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  // (void) pvParameters;

  /*
    AnalogReadSerial
    Reads an analog input on pin 0, prints the result to the serial monitor.
    Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
    Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

    This example code is in the public domain.
  */

  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A7);  /// modify for your input pin!
    // print out the value you read:
    Serial.println(sensorValue);
    vTaskDelay(500 / portTICK_PERIOD_MS); // 0.5 sec in between reads for stability
  }
}

void lim(int freq) { // Sets CTC overflow to value to generate given freq
  if(freq ==0){
    SPEAKER_OFF;
  }else{
    frequency = freq;
    DEF_TONE;
  }
}

ISR(TIMER3_COMPA_vect) { //This is the timer interrupt request
  timer++; //Increment timers
}
