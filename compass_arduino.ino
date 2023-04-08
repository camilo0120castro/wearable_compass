/*In order from Top X to Bottom
  Pin      ESP8266   Arduino Uno
  VCC       3V3       5V 				//White
  GND       GND       GND 				//Black
  SCL       D1        A5 				//Purple - Brown
  SDA       D4        A4 */ 				//Green - Red

/* FIXME:Make the initial offset values and manual offset anglue value in a function
Save the offset and manual offset values in the EEPROM memory
Check if the calibration_time is too long or just enough
*/

// Libraries
#include <Wire.h>
#include <HMC5883L.h> //Two (2) new functions were included in the .h and .cpp files

HMC5883L compass;

//The Arduino receives power VIN from a Orange wire with black tape

/* Pinout for calibration
Perform calibration when a button is pushed, FIXME:LED indicates status*/
const int button_pin = 2; // The pin number of the button 	Yellow
const int buzzer_pin = 3; // The pin number of the Buzzer 	Orange Jumper
const int calibration_time = 10000; //in milliseconds
int button_state = LOW;  	// 3.3V Orange wire to CLBR button

/* Pinout for motors
Motor 1 is connected to digital pin 4*/
const int motor_1_pin = 4; // N - motor_pins[0] 	Blue No case jumper
const int motor_2_pin = 5; // E - motor_pins[1] 	White jumper
const int motor_3_pin = 6; // S - motor_pins[2] 	Purple jumper
const int motor_4_pin = 7; // W - motor_pins[3] 	Blue CASE jumper
const int motor_pins[] = {motor_1_pin, motor_2_pin, motor_3_pin, motor_4_pin};

/* Set declination angle for your location
Visit: http://magnetic-declination.com/ or https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
(+) Positive for E and (-) negative for W
For Ottawa, ON declination angle is 12° 50'W (negative)
Formula: (+/-) (deg + (min/60) + (sec/3600)) * (PI/180); in radians*/
const float declination_angle = -12.833; //in degrees
float global_heading = 0;



/* Compass initialization
Initial offset values FIXME: from my last test
Offset Values from tests
    Xoff  Yoff  Diff °
    -101, 66 = 30
    -82, 22 = 20
    -79, 36 = 20

    -232, 80
    -76, -7
    -40, 64
    -111, 96
    -125, 88
    -137, 62
    -117, 54
    -117, 58
    -10, 99
    -149, 37
    -120, 116
    -347, 79
    -318, 61
    -136, 115
	-33, 79 */
const int x_off_initial = -13;
const int y_off_initial = 129;

void initialize_compass_in_setup()
{
    Wire.begin();

    // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_15HZ);

    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_4);

    // Set initial calibration offset in X and Y. See HMC5883L_calibration.ino by jarzebski
    compass.setOffset(x_off_initial, y_off_initial);
}

//Read Heading
const int manual_off_angle_heading = 30; //FIXME: From a test in Ottawa, ON.
float read_heading_compass()
{
    Vector norm = compass.readNormalize();

    // Calculate heading in degrees (°)
    float heading = (atan2f(norm.YAxis, norm.XAxis) * 180) / PI;
    
    /*Debugging
	Serial.print("Y: ");
    Serial.print(norm.YAxis);
	Serial.print(" X: ");
    Serial.println(norm.XAxis);

    Serial.print("H bef: ");
    Serial.print(heading);*/

    //Correct the value of the heading
    heading = heading + declination_angle + manual_off_angle_heading;

    // Correct for heading < 0° and heading > 360°
    if (heading < 0)
    {
      heading += 360;
    }

    if (heading > 360)
    {
      heading -= 360;
    }

	/* Debugging*/
    Serial.print("\taf: ");
    Serial.println(heading);

    return heading;
}



//Calibration
void calibrate_compass_setup()
{
    // Set the button and Buzzer pins as inputs and outputs
    pinMode(button_pin, INPUT);
    pinMode(buzzer_pin, OUTPUT);
}

int x_off = x_off_initial; //Calibration offset values
int y_off = y_off_initial;
void calibrate_compass()
{
    /* The user needs to complete a full rotation (360°) of the compass
    The minimum and maximum value in X and Y are calculated, and the offset is also calculated.
    After the full rotation is completed, the new offset values are set outside of this function,
    below in the "is_calibration_button_pushed()" function. */

    Serial.println("Please complete ONLY one (1) full rotation (360°)");

    // While the calibration is performed, all the motors are off
    for(int i=0; i<4; i++)
    {
        motor_n_deactivate(i);
    }
    
    /*FIXME:What about this?
    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);*/
    
    int minX = 0;
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    compass.setOffset(0, 0); //reset offset

    Vector mag = compass.readRaw(); //reads raw values from the sensor

    // Determine Min / Max values
    if (mag.XAxis < minX) minX = mag.XAxis;
    if (mag.XAxis > maxX) maxX = mag.XAxis;
    if (mag.YAxis < minY) minY = mag.YAxis;
    if (mag.YAxis > maxY) maxY = mag.YAxis;

    /* Calculate offsets
    the new values are set below in the "is_calibration_button_pushed()" function*/
    x_off = (maxX + minX)/2;
    y_off = (maxY + minY)/2;

    /* Debugging
    Serial.print(mag.XAxis);
    Serial.print(":");
    Serial.print(mag.YAxis);
    Serial.print(":");
    Serial.print(minX);
    Serial.print(":");
    Serial.print(maxX);
    Serial.print(":");
    Serial.print(minY);
    Serial.print(":");
    Serial.print(maxY);
    Serial.print(":");
    
    //To check whether the offset values change when the button is released before completing the calibration add the next code in "void loop()"
    Serial.print("Main: ");
    Serial.print(compass.getOffsetValue_X());
    Serial.print(":");
    Serial.println(compass.getOffsetValue_Y());*/
    // Serial.print(x_off);
    // Serial.print(":");
    // Serial.println(y_off);
}


//Play a song to indicate ON/OFF and End of calibration
void play_melody(String melody_name)
{
	//----------------- All Notes-----------------
	const int NOTE_B0 = 31;
	const int NOTE_C1 = 33;
	const int NOTE_CS1 = 35;
	const int NOTE_D1 = 37;
	const int NOTE_DS1 = 39;
	const int NOTE_E1 = 41;
	const int NOTE_F1 = 44;
	const int NOTE_FS1 = 46;
	const int NOTE_G1 = 49;
	const int NOTE_GS1 = 52;
	const int NOTE_A1 = 55;
	const int NOTE_AS1 = 58;
	const int NOTE_B1 = 62;
	const int NOTE_C2 = 65;
	const int NOTE_CS2 = 69;
	const int NOTE_D2 = 73;
	const int NOTE_DS2 = 78;
	const int NOTE_E2 = 82;
	const int NOTE_F2 = 87;
	const int NOTE_FS2 = 93;
	const int NOTE_G2 = 98;
	const int NOTE_GS2 = 104;
	const int NOTE_A2 = 110;
	const int NOTE_AS2 = 117;
	const int NOTE_B2 = 123;
	const int NOTE_C3 = 131;
	const int NOTE_CS3 = 139;
	const int NOTE_D3 = 147;
	const int NOTE_DS3 = 156;
	const int NOTE_E3 = 165;
	const int NOTE_F3 = 175;
	const int NOTE_FS3 = 185;
	const int NOTE_G3 = 196;
	const int NOTE_GS3 = 208;
	const int NOTE_A3 = 220;
	const int NOTE_AS3 = 233;
	const int NOTE_B3 = 247;
	const int NOTE_C4 = 262;
	const int NOTE_CS4 = 277;
	const int NOTE_D4 = 294;
	const int NOTE_DS4 = 311;
	const int NOTE_E4 = 330;
	const int NOTE_F4 = 349;
	const int NOTE_FS4 = 370;
	const int NOTE_G4 = 392;
	const int NOTE_GS4 = 415;
	const int NOTE_A4 = 440;
	const int NOTE_AS4 = 466;
	const int NOTE_B4 = 494;
	const int NOTE_C5 = 523;
	const int NOTE_CS5 = 554;
	const int NOTE_D5 = 587;
	const int NOTE_DS5 = 622;
	const int NOTE_E5 = 659;
	const int NOTE_F5 = 698;
	const int NOTE_FS5 = 740;
	const int NOTE_G5 = 784;
	const int NOTE_GS5 = 831;
	const int NOTE_A5 = 880;
	const int NOTE_AS5 = 932;
	const int NOTE_B5 = 988;
	const int NOTE_C6 = 1047;
	const int NOTE_CS6 = 1109;
	const int NOTE_D6 = 1175;
	const int NOTE_DS6 = 1245;
	const int NOTE_E6 = 1319;
	const int NOTE_F6 = 1397;
	const int NOTE_FS6 = 1480;
	const int NOTE_G6 = 1568;
	const int NOTE_GS6 = 1661;
	const int NOTE_A6 = 1760;
	const int NOTE_AS6 = 1865;
	const int NOTE_B6 = 1976;
	const int NOTE_C7 = 2093;
	const int NOTE_CS7 = 2217;
	const int NOTE_D7 = 2349;
	const int NOTE_DS7 = 2489;
	const int NOTE_E7 = 2637;
	const int NOTE_F7 = 2794;
	const int NOTE_FS7 = 2960;
	const int NOTE_G7 = 3136;
	const int NOTE_GS7 = 3322;
	const int NOTE_A7 = 3520;
	const int NOTE_AS7 = 3729;
	const int NOTE_B7 = 3951;
	const int NOTE_C8 = 4186;
	const int NOTE_CS8 = 4435;
	const int NOTE_D8 = 4699;
	const int NOTE_DS8 = 4978;
	const int REST = 0;
	
	/*----------------- Songs -----------------
	
	More songs available at https://github.com/robsoncouto/arduino-songs                                                                                          
												Robson Couto, 2019

	notes of the melody followed by the duration.
	a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
	!!negative numbers are used to represent dotted notes,
	so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!! */

	const int initialization_song[] = { /* Take on me, by A-ha
		Score available at https://musescore.com/user/27103612/scores/4834399
		Arranged by Edward Truong and modified by Camilo Castro */
		//Tempo = 140

		//REST,8, NOTE_B4,8, REST,8, NOTE_E5,8, REST,8, NOTE_E5,8,
		REST,8, REST,8, NOTE_E5,8, NOTE_GS5,8, NOTE_GS5,8, NOTE_A5,8, NOTE_B5,8,
		NOTE_A5,8, NOTE_A4,8, NOTE_A5,8, NOTE_E5,8, REST,8, NOTE_D5,8, REST,8, NOTE_A5,8, REST,8
	};

	const int already_calibrated_song[] = { /* Mii Channel theme 
		Score available at https://musescore.com/user/16403456/scores/4984153
    	Uploaded by Catalina Andrade and modified by Camilo Castro */
    	//Tempo = 114;
    
	    //NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, 
	    REST,8, NOTE_A4,8, REST,8, NOTE_FS4,8, //1
	    NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8
	};

	/* const int furelise[] = { // Fur Elise - Ludwig van Beethovem
		// Score available at https://musescore.com/user/28149610/scores/5281944
		//Tempo = 80;

		//starts from 1 ending on 9
		NOTE_E5, 16, NOTE_DS5, 16, //1
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, -8, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16,
		NOTE_C5, 8,  REST, 16, NOTE_E4, 16, NOTE_E5, 16,  NOTE_DS5, 16,
		
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,//6
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16, 
		NOTE_B4, -8, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16, 
		NOTE_A4 , 4, REST, 8, //9 - 1st ending

		//repaets from 1 ending on 10
		NOTE_E5, 16, NOTE_DS5, 16, //1
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, -8, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16,
		NOTE_C5, 8,  REST, 16, NOTE_E4, 16, NOTE_E5, 16,  NOTE_DS5, 16,
		
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,//6
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16, 
		NOTE_B4, -8, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16, 
		NOTE_A4, 8, REST, 16, NOTE_B4, 16, NOTE_C5, 16, NOTE_D5, 16, //10 - 2nd ending
		//continues from 11
		NOTE_E5, -8, NOTE_G4, 16, NOTE_F5, 16, NOTE_E5, 16, 
		NOTE_D5, -8, NOTE_F4, 16, NOTE_E5, 16, NOTE_D5, 16, //12
		
		NOTE_C5, -8, NOTE_E4, 16, NOTE_D5, 16, NOTE_C5, 16, //13
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16, REST, 16,
		REST, 16, NOTE_E5, 16, NOTE_E6, 16, REST, 16, REST, 16, NOTE_DS5, 16,
		NOTE_E5, 16, REST, 16, REST, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_DS5, 16,
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16, //19
		NOTE_C5, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16,  NOTE_DS5, 16,
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16,
		NOTE_A4, 8, REST, 16, NOTE_B4, 16, NOTE_C5, 16, NOTE_D5, 16, //24 (1st ending)
		
		//repeats from 11
		NOTE_E5, -8, NOTE_G4, 16, NOTE_F5, 16, NOTE_E5, 16, 
		NOTE_D5, -8, NOTE_F4, 16, NOTE_E5, 16, NOTE_D5, 16, //12
		
		NOTE_C5, -8, NOTE_E4, 16, NOTE_D5, 16, NOTE_C5, 16, //13
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16, REST, 16,
		REST, 16, NOTE_E5, 16, NOTE_E6, 16, REST, 16, REST, 16, NOTE_DS5, 16,
		NOTE_E5, 16, REST, 16, REST, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_DS5, 16,
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16, //19
		NOTE_C5, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16,  NOTE_DS5, 16,
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16,
		NOTE_A4, 8, REST, 16, NOTE_C5, 16, NOTE_C5, 16, NOTE_C5, 16, //25 - 2nd ending

		//continues from 26
		NOTE_C5 , 4, NOTE_F5, -16, NOTE_E5, 32, //26
		NOTE_E5, 8, NOTE_D5, 8, NOTE_AS5, -16, NOTE_A5, 32,
		NOTE_A5, 16, NOTE_G5, 16, NOTE_F5, 16, NOTE_E5, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_AS4, 8, NOTE_A4, 8, NOTE_A4, 32, NOTE_G4, 32, NOTE_A4, 32, NOTE_B4, 32,
		NOTE_C5 , 4, NOTE_D5, 16, NOTE_DS5, 16,
		NOTE_E5, -8, NOTE_E5, 16, NOTE_F5, 16, NOTE_A4, 16,
		NOTE_C5 , 4,  NOTE_D5, -16, NOTE_B4, 32,
		
		
		NOTE_C5, 32, NOTE_G5, 32, NOTE_G4, 32, NOTE_G5, 32, NOTE_A4, 32, NOTE_G5, 32, NOTE_B4, 32, NOTE_G5, 32, NOTE_C5, 32, NOTE_G5, 32, NOTE_D5, 32, NOTE_G5, 32, //33
		NOTE_E5, 32, NOTE_G5, 32, NOTE_C6, 32, NOTE_B5, 32, NOTE_A5, 32, NOTE_G5, 32, NOTE_F5, 32, NOTE_E5, 32, NOTE_D5, 32, NOTE_G5, 32, NOTE_F5, 32, NOTE_D5, 32,
		NOTE_C5, 32, NOTE_G5, 32, NOTE_G4, 32, NOTE_G5, 32, NOTE_A4, 32, NOTE_G5, 32, NOTE_B4, 32, NOTE_G5, 32, NOTE_C5, 32, NOTE_G5, 32, NOTE_D5, 32, NOTE_G5, 32,

		NOTE_E5, 32, NOTE_G5, 32, NOTE_C6, 32, NOTE_B5, 32, NOTE_A5, 32, NOTE_G5, 32, NOTE_F5, 32, NOTE_E5, 32, NOTE_D5, 32, NOTE_G5, 32, NOTE_F5, 32, NOTE_D5, 32, //36
		NOTE_E5, 32, NOTE_F5, 32, NOTE_E5, 32, NOTE_DS5, 32, NOTE_E5, 32, NOTE_B4, 32, NOTE_E5, 32, NOTE_DS5, 32, NOTE_E5, 32, NOTE_B4, 32, NOTE_E5, 32, NOTE_DS5, 32,
		NOTE_E5, -8, NOTE_B4, 16, NOTE_E5, 16, NOTE_DS5, 16,
		NOTE_E5, -8, NOTE_B4, 16, NOTE_E5, 16, REST, 16,

		REST, 16, NOTE_DS5, 16, NOTE_E5, 16, REST, 16, REST, 16, NOTE_DS5, 16, //40
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16,
		NOTE_C5, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16, NOTE_DS5, 16,
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,

		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16, //46
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16,
		NOTE_A4, 8, REST, 16, NOTE_B4, 16, NOTE_C5, 16, NOTE_D5, 16,
		NOTE_E5, -8, NOTE_G4, 16, NOTE_F5, 16, NOTE_E5, 16,
		NOTE_D5, -8, NOTE_F4, 16, NOTE_E5, 16, NOTE_D5, 16,
		NOTE_C5, -8, NOTE_E4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16, REST, 16,
		REST, 16, NOTE_E5, 16, NOTE_E6, 16, REST, 16, REST, 16, NOTE_DS5, 16,

		NOTE_E5, 16, REST, 16, REST, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_D5, 16, //54
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16,
		NOTE_C5, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16, NOTE_DS5, 16,
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		
		NOTE_A4, 8, REST, 16, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16, //60
		NOTE_B4, 8, REST, 16, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16,
		NOTE_A4, 8, REST, 16, REST, 16, REST, 8, 
		NOTE_CS5 , -4, 
		NOTE_D5 , 4, NOTE_E5, 16, NOTE_F5, 16,
		NOTE_F5 , 4, NOTE_F5, 8, 
		NOTE_E5 , -4,
		NOTE_D5 , 4, NOTE_C5, 16, NOTE_B4, 16,
		NOTE_A4 , 4, NOTE_A4, 8,
		NOTE_A4, 8, NOTE_C5, 8, NOTE_B4, 8,
		NOTE_A4 , -4,
		NOTE_CS5 , -4,

		NOTE_D5 , 4, NOTE_E5, 16, NOTE_F5, 16, //72
		NOTE_F5 , 4, NOTE_F5, 8,
		NOTE_F5 , -4,
		NOTE_DS5 , 4, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_AS4 , 4, NOTE_A4, 8,
		NOTE_GS4 , 4, NOTE_G4, 8,
		NOTE_A4 , -4,
		NOTE_B4 , 4, REST, 8,
		NOTE_A3, -32, NOTE_C4, -32, NOTE_E4, -32, NOTE_A4, -32, NOTE_C5, -32, NOTE_E5, -32, NOTE_D5, -32, NOTE_C5, -32, NOTE_B4, -32,

		NOTE_A4, -32, NOTE_C5, -32, NOTE_E5, -32, NOTE_A5, -32, NOTE_C6, -32, NOTE_E6, -32, NOTE_D6, -32, NOTE_C6, -32, NOTE_B5, -32, //80
		NOTE_A4, -32, NOTE_C5, -32, NOTE_E5, -32, NOTE_A5, -32, NOTE_C6, -32, NOTE_E6, -32, NOTE_D6, -32, NOTE_C6, -32, NOTE_B5, -32,
		NOTE_AS5, -32, NOTE_A5, -32, NOTE_GS5, -32, NOTE_G5, -32, NOTE_FS5, -32, NOTE_F5, -32, NOTE_E5, -32, NOTE_DS5, -32, NOTE_D5, -32,

		NOTE_CS5, -32, NOTE_C5, -32, NOTE_B4, -32, NOTE_AS4, -32, NOTE_A4, -32, NOTE_GS4, -32, NOTE_G4, -32, NOTE_FS4, -32, NOTE_F4, -32, //84
		NOTE_E4, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, -8, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16,

		NOTE_C5, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16, NOTE_DS5, 16, //88
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16, 
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16, 
		NOTE_B4, -8, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16, 
		NOTE_A4, -8, REST, -8,
		REST, -8, NOTE_G4, 16, NOTE_F5, 16, NOTE_E5, 16,
		NOTE_D5 , 4, REST, 8,
		REST, -8, NOTE_E4, 16, NOTE_D5, 16, NOTE_C5, 16,
		
		NOTE_B4, -8, NOTE_E4, 16, NOTE_E5, 8, //96
		NOTE_E5, 8, NOTE_E6, -8, NOTE_DS5, 16,
		NOTE_E5, 16, REST, 16, REST, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_DS5, 16,
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, -8, NOTE_E4, 16, NOTE_GS4, 16, NOTE_B4, 16,

		NOTE_C5, 8, REST, 16, NOTE_E4, 16, NOTE_E5, 16, NOTE_DS5, 16, //102
		NOTE_E5, 16, NOTE_DS5, 16, NOTE_E5, 16, NOTE_B4, 16, NOTE_D5, 16, NOTE_C5, 16,
		NOTE_A4, -8, NOTE_C4, 16, NOTE_E4, 16, NOTE_A4, 16,
		NOTE_B4, -8, NOTE_E4, 16, NOTE_C5, 16, NOTE_B4, 16,
		NOTE_A4 , -4,
	};
	*/
	

	//----------------- Variable Assignations -----------------

	int tempo = 0; //Change this to make the song slower or faster
	int song_length = 0; //Size of each individual songs array
	int song_size_bits = 0; //The size in bits of each individual songs array

	/* Assign the tempo of each individual song to the variable tempo
	Determine the size of each individual song to create a melody array with the same size */  
	if (melody_name == "initialization_song")
	{
		int initialization_song_tempo = 140;
    	tempo = initialization_song_tempo;
    	song_length = ( sizeof(initialization_song) / sizeof(initialization_song[0]) );
    	song_size_bits = sizeof(initialization_song);

		/*Debugging
		Serial.print(song_length);
    	Serial.print(" ");
    	Serial.print(song_size_bits);
    	Serial.print(" .\n");*/
	}

	else if (melody_name == "already_calibrated_song")
	{
		int already_calibrated_song_tempo = 114;
    	tempo = already_calibrated_song_tempo;
    	song_length = ( sizeof(already_calibrated_song) / sizeof(already_calibrated_song[0]) );
    	song_size_bits = sizeof(already_calibrated_song);

		/*Debugging
		Serial.print(song_length);
    	Serial.print(" ");
    	Serial.print(song_size_bits);
    	Serial.print(" .\n");*/
	}

	// Copy the elements of each individual song to the general melody array
	int melody[song_length]; // General melody array
	
	if (melody_name == "initialization_song")
	{
		memcpy(melody, initialization_song, song_size_bits);
	}
	
	else if (melody_name == "already_calibrated_song")
	{
		memcpy(melody, already_calibrated_song, song_size_bits);
	}
	

	/* sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
	there are two values per note (pitch and duration), so for each note there are four bytes*/
	int notes = sizeof(melody) / sizeof(melody[0]) / 2;

	// this calculates the duration of a whole note in ms
	int wholenote = (60000 * 4) / tempo;

	int divider = 0, noteDuration = 0;


	//----------------- Playing Operation -----------------

	/* iterate over the notes of the melody.
	Remember, the array is twice the number of notes (notes + durations)*/
	for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2)
	{
		// calculates the duration of each note
		divider = melody[thisNote + 1];

		if (divider > 0)
		{
			noteDuration = (wholenote) / divider; // regular note, just proceed
		}
		else if (divider < 0)
		{
			// dotted notes are represented with negative durations!!
			noteDuration = (wholenote) / abs(divider);
			noteDuration *= 1.5; // increases the duration in half for dotted notes
		}

		// we only play the note for 90% of the duration, leaving 10% as a pause
		tone(buzzer_pin, melody[thisNote], noteDuration * 0.9);

		// Wait for the specief duration before playing the next note.
		delay(noteDuration);

		// stop the waveform generation before the next note.
		noTone(buzzer_pin);
	}
}


void is_calibration_button_pushed()
{
    // Read the state of the button
    button_state = digitalRead(button_pin);

    // If the button is not pushed, the Buzzer is off
    // if (button_state == LOW) // FIXME:
    // {
    //     // noTone(buzzer_pin); FIXME:
    // }

    // If the button is pushed, the calibration will start
    if (button_state == HIGH)
    {
        long startTime = millis(); // get the current time in milliseconds

        //get the current offset values. If the button is released, the values return to their previous values and do not change 
        int temp_x_off = x_off;
        int temp_y_off = y_off;

        while (millis() - startTime < calibration_time) // run the loop for 10 seconds (10 000 milliseconds)
        {
            if (digitalRead(button_pin) == LOW) // check if the button has been released
            {
                // keep the previous offset values
                x_off = temp_x_off;
                y_off = temp_y_off;
                break; // exit the loop
            }
            // FIXME: digitalWrite(led_pin, HIGH); //The LED indicates that the calibration is being performed
            calibrate_compass(); //TODO: Check if the fixme is needed or not inside there
        }

        // Set the new calibration offset in X and Y to the sensor
        compass.setOffset(x_off, y_off);

        button_state = digitalRead(button_pin);
        if (button_state == HIGH)// if the button is still being pressed, the calibration was completed
        {
            play_melody("already_calibrated_song"); // FIXME:
			delay(250);
            Serial.println("Calibration completed!");
        }
    }
}



//Motors
void motors_setup()
{
    // Set the motors as outputs
    for(int i=0; i<4; i++)
    {
        pinMode(motor_pins[i], OUTPUT);
    }
}

void motor_n_activate(int motor_number)
{
    if(motor_number < 0 || motor_number > 3)
    {
        Serial.println("motor number is wrong");
    }
    else
    {
        // Turn on motor
        digitalWrite(motor_pins[motor_number], HIGH);
        /* Debugging
        Serial.print("MOTOR ");
        Serial.print(motor_number);
        Serial.println(" ON");*/
    }
}

void motor_n_deactivate(int motor_number)
{
    if(motor_number < 0 || motor_number > 3)
    {
        Serial.println("motor number is wrong");
    }
    else
    {
        // Turn off motor
        digitalWrite(motor_pins[motor_number], LOW);
        /* Debugging
        Serial.print("MOTOR ");
        Serial.print(motor_number);
        Serial.println(" OFF");*/
    }
}

void heading_toggle_motor (float heading_angle)
{
    // Turns off all the motors
    for(int i=0; i<4; i++)
    {
        motor_n_deactivate(i);
    }

    // Turn on a motor(s) based on the direction of the heading
    if ( (heading_angle >= 330 && heading_angle < 360) || (heading_angle >= 0 && heading_angle < 30) )
    {
        // N
		motor_n_activate(0);
	}
    else if (heading_angle >= 30 && heading_angle < 60)
    {
        // N-E
		motor_n_activate(0);
		motor_n_activate(1);
	}
    else if (heading_angle >= 60 && heading_angle < 120)
    {
        // E
		motor_n_activate(1);
	}
    else if (heading_angle >= 120 && heading_angle < 150)
    {
        // E-S
		motor_n_activate(1);
		motor_n_activate(2);
	}
    else if (heading_angle >= 150 && heading_angle < 210)
    {
        // S
		motor_n_activate(2);
	}
    else if (heading_angle >= 210 && heading_angle < 240)
    {
        // S-W
		motor_n_activate(2);
		motor_n_activate(3);
	}
    else if (heading_angle >= 240 && heading_angle < 300)
    {
        // W
		motor_n_activate(3);
	}
    else if (heading_angle >= 300 && heading_angle < 330)
    {
        // W-N
		motor_n_activate(3);
		motor_n_activate(0);
	}
}



// -------------------------------  MAIN PROGRAM   ---------------------------------------

void setup()
{
    Serial.begin(9600); //for debugging
    calibrate_compass_setup();
    motors_setup();
    pinMode(LED_BUILTIN, OUTPUT); //To know if Arduino is working
	play_melody("initialization_song"); //To know if Arduino is working
    initialize_compass_in_setup();
}

// the loop function runs over and over again forever
void loop()
{
    digitalWrite(LED_BUILTIN,HIGH); //To know if Arduino is working
    is_calibration_button_pushed();
    delay(100);
    global_heading = read_heading_compass();
    heading_toggle_motor(global_heading);
    delay(100);
}