/*In order from Top X to Bottom
  Pin      ESP8266   Arduino Uno
  VCC       3V3       5V
  GND       GND       GND
  SCL       D5        A5
  SDA       D6        A4 */

/* FIXME:Make the initial offset values and manual offset anglue value in a function
Save the offset and manual offset values in the EEPROM memory
Check if the calibration_time is too long or just enough
Change the LED for a buzzer, but it may need more current, maybe include a transistor (?)
*/

// Libraries
#include <Wire.h>
#include <HMC5883L.h> //Two (2) new functions were included in the .h and .cpp files

HMC5883L compass;

/* Pinout for calibration
Perform calibration when a button is pushed, LED indicates status*/
const int button_pin = 2; // The pin number of the button
const int led_pin = 3; // The pin number of the LED
const int calibration_time = 10000; //in milliseconds
int button_state = LOW;

/* Pinout for motors
Motor 1 is connected to digital pin 4*/
const int motor_1_pin = 4; // N - motor_pins[0]
const int motor_2_pin = 5; // E - motor_pins[1]
const int motor_3_pin = 6; // S - motor_pins[2]
const int motor_4_pin = 7; // W - motor_pins[3]
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

	/* Debugging
    Serial.print("\taf: ");
    Serial.println(heading);*/

    return heading;
}



//Calibration
void calibrate_compass_setup()
{
    // Set the button and LED pins as inputs and outputs
    pinMode(button_pin, INPUT);
    pinMode(led_pin, OUTPUT);
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

void is_calibration_button_pushed()
{
    // Read the state of the button
    button_state = digitalRead(button_pin);

    // If the button is not pushed, the LED is off
    if (button_state == LOW)
    {
        digitalWrite(led_pin, LOW);
    }

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
            digitalWrite(led_pin, HIGH); //The LED indicates that the calibration is being performed
            calibrate_compass(); //TODO: Check if the fixme is needed or not inside there
        }

        // Set the new calibration offset in X and Y to the sensor
        compass.setOffset(x_off, y_off);

        button_state = digitalRead(button_pin);
        if (button_state == HIGH)// if the button is still being pressed, the calibration was completed
        {
            for (int i = 0; i < 5; i++) // the LED blink 5 times -- Calibration completed
            {  
                digitalWrite(led_pin, LOW);
                delay(250); 
                digitalWrite(led_pin, HIGH);
                delay(250); 
            }
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