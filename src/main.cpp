#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BME680_I2C.h"
#include <SparkFunLSM9DS1.h>

TaskHandle_t Task0;
TaskHandle_t Task1;

DFRobot_BME680_I2C bme(0x77); // 0x77 I2C address
LSM9DS1 imu;
#define PRINT_CALCULATED
#define PRINT_SPEED 250             // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time
#define DECLINATION -8.58
// imu adresses
// #define LSM9DS1_M 0x1E  // Would be 0x1C if SDO_M is LOW
// #define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

#define RXD2 16
#define TXD2 17

int updates;
int failedUpdates;
int pos;
int stringplace = 0;

String timeUp;
String nmea[15];
String labels[12]{
    "Time: ", "Status: ", "Latitude: ", "Hemisphere: ", "Longitude: ", "Hemisphere: ",
    "Speed: ", "Track Angle: ", "Date: "};

String ConvertLat();
String ConvertLng();

// Function declarations
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
String ConvertLat();
String ConvertLng();

void goToDeepSleep()
{
    Serial.println("Going to Sleep");
    esp_sleep_enable_timer_wakeup(1 * 10 * 1000000); //sleep 10 seconds
    esp_deep_sleep_start();
}

void loop0(void * parameter)
{
    Serial.println("Task 0 running on core 0\n");
    for(;;)
    {
        Serial.println("Starting task 0 loop");
        if (imu.gyroAvailable())
        {
            imu.readGyro();
        }
        if (imu.accelAvailable())
        {
            imu.readAccel();
        }
        if (imu.magAvailable())
        {
            imu.readMag();
        }
        if ((lastPrint + PRINT_SPEED) < millis())
        {
            printGyro();  // Print "G: gx, gy, gz"
            printAccel(); // Print "A: ax, ay, az"
            printMag();   // Print "M: mx, my, mz"
            printAttitude(imu.ax, imu.ay, imu.az,
                            -imu.my, -imu.mx, imu.mz);
            Serial.println();
            lastPrint = millis(); // Update lastPrint time
        }
        delay(1000);
        bme.startConvert();
        bme.update();
        Serial.print("temperature(C) :");
        Serial.println(bme.readTemperature() / 100, 2);
        Serial.print("humidity(%rh) :");
        Serial.println(bme.readHumidity() / 1000, 2);
        Serial.print("gas resistance(ohm) :");
        Serial.println(bme.readGasResistance());
        Serial.println();
        Serial.println("Ending task 0 loop");
    }
}
 
void loop1(void * parameter)
{
    Serial.println("Task 1 running on core 1\n");
    for(;;)
    {
        Serial.println("Starting task 1 loop");
        while (Serial2.available() > 0)
        {
            char(Serial2.read());
        }
        if (char(Serial2.find("$GPRMC,")))
        {
            String tempMsg = String(Serial2.readStringUntil('\n'));
            for (int i = 0; i < tempMsg.length(); i++)
            {
                if (tempMsg.substring(i, i + 1) == ",")
                {
                    nmea[pos] = tempMsg.substring(stringplace, i);
                    stringplace = i + 1;
                    pos++;
                }
                if (i == tempMsg.length() - 1)
                {
                    nmea[pos] = tempMsg.substring(stringplace, i);
                }
            }
            updates++;
            nmea[2] = ConvertLat();
            nmea[4] = ConvertLng();
            Serial.println(labels[1] + nmea[1]);
            Serial.println("Latitude, Longtitude: " + nmea[2] + "," + " " + nmea[4]);
            Serial.println();
        }
        else
        {
            failedUpdates++;
            Serial.println("Serial 2 gprmc not avalible");
        }
        stringplace = 0;
        pos = 0;
        Serial.println("Ending task 1 loop");
        if (nmea[1] == "A")
        {
            goToDeepSleep();
            Serial.println("Waking up!");
        }
    }
}

void setup()
{ 
    Serial.begin(9600); // this creates the Serial Monitor
    Wire.begin();       // this creates a Wire object
    Serial2.begin(9600, SERIAL_8N2, RXD2, TXD2);
    uint8_t rslt = 1;
    while (!Serial)
        ;
    delay(1000);
    Serial.println();
    while (rslt != 0)
    {
        rslt = bme.begin();
        if (rslt != 0)
        {
            Serial.println("bme begin failure");
            delay(2000);
        }
    }
    Serial.println("bme begin successful");

    if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
    {
        Serial.println("Failed to communicate with LSM9DS1.");
        Serial.println("Double-check wiring.");
        Serial.println("Default settings in this sketch will "
                       "work for an out of the box LSM9DS1 "
                       "Breakout, but may need to be modified "
                       "if the board jumpers are.");
        while (1)
            ;
    } 
    else 
    {
        Serial.println("imu ready");
    }

    xTaskCreatePinnedToCore(
        loop0, /* Function to implement the task */
        "Task0", /* Name of the task */
        2048 , /* Stack size in words */
        NULL, /* Task input parameter */
        1, /* Priority of the task */
        &Task0, /* Task handle. */
        0); /* Core where the task should run */

    xTaskCreatePinnedToCore(
        loop1, /* Function to implement the task */
        "Task1", /* Name of the task */
        2048 , /* Stack size in words */
        NULL, /* Task input parameter */
        1, /* Priority of the task */
        &Task1, /* Task handle. */
        1); /* Core where the task should run */
}

void loop()
{
}

void printGyro()
{
    Serial.print("G: ");
#ifdef PRINT_CALCULATED
    Serial.print(imu.calcGyro(imu.gx), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gy), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gz), 2);
    Serial.println(" deg/s");
#elif defined PRINT_RAW
    Serial.print(imu.gx);
    Serial.print(", ");
    Serial.print(imu.gy);
    Serial.print(", ");
    Serial.println(imu.gz);
#endif
}

void printAccel()
{
    Serial.print("A: ");
#ifdef PRINT_CALCULATED
    Serial.print(imu.calcAccel(imu.ax), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.ay), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.az), 2);
    Serial.println(" g");
#elif defined PRINT_RAW
    Serial.print(imu.ax);
    Serial.print(", ");
    Serial.print(imu.ay);
    Serial.print(", ");
    Serial.println(imu.az);
#endif
}

void printMag()
{
    Serial.print("M: ");
#ifdef PRINT_CALCULATED
    Serial.print(imu.calcMag(imu.mx), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.my), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.mz), 2);
    Serial.println(" gauss");
#elif defined PRINT_RAW
    Serial.print(imu.mx);
    Serial.print(", ");
    Serial.print(imu.my);
    Serial.print(", ");
    Serial.println(imu.mz);
#endif
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float heading;
    if (my == 0)
        heading = (mx < 0) ? PI : 0;
    else
        heading = atan2(mx, my);

    heading -= DECLINATION * PI / 180;

    if (heading > PI)
        heading -= (2 * PI);
    else if (heading < -PI)
        heading += (2 * PI);

    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    Serial.print("Pitch, Roll: ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    Serial.print("Heading: ");
    Serial.println(heading, 2);
}

String ConvertLat()
{
    String posneg = "";
    if (nmea[3] == "S")
    {
        posneg = "-";
    }
    String latfirst;
    float latsecond;
    for (int i = 0; i < nmea[2].length(); i++)
    {
        if (nmea[2].substring(i, i + 1) == ".")
        {
            latfirst = nmea[2].substring(0, i - 2);
            latsecond = nmea[2].substring(i - 2).toFloat();
        }
    }
    latsecond = latsecond / 60;
    String CalcLat = "";

    char charVal[9];
    dtostrf(latsecond, 4, 6, charVal);
    for (int i = 0; i < sizeof(charVal); i++)
    {
        CalcLat += charVal[i];
    }
    latfirst += CalcLat.substring(1);
    latfirst = posneg += latfirst;
    return latfirst;
}

String ConvertLng()
{
    String posneg = "";
    if (nmea[5] == "W")
    {
        posneg = "-";
    }

    String lngfirst;
    float lngsecond;
    for (int i = 0; i < nmea[4].length(); i++)
    {
        if (nmea[4].substring(i, i + 1) == ".")
        {
            lngfirst = nmea[4].substring(0, i - 2);
            // Serial.println(lngfirst);
            lngsecond = nmea[4].substring(i - 2).toFloat();
            // Serial.println(lngsecond);
        }
    }
    lngsecond = lngsecond / 60;
    String CalcLng = "";
    char charVal[9];
    dtostrf(lngsecond, 4, 6, charVal);
    for (int i = 0; i < sizeof(charVal); i++)
    {
        CalcLng += charVal[i];
    }
    lngfirst += CalcLng.substring(1);
    lngfirst = posneg += lngfirst;
    return lngfirst;
}
