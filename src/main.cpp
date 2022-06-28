#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BME680_I2C.h>
#include <SparkFunLSM9DS1.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
//.....................................firebase..........................................
#define WIFI_SSID "tel"
#define WIFI_PASSWORD "huds2717"
#define API_KEY "AIzaSyCvHwT0Ng76xLgDAK7ZRXI4WD1h8UnSaZ4"
#define DATABASE_URL "earthquake-a2802-default-rtdb.europe-west1.firebasedatabase.app"
#define USER_EMAIL "cagatayparlar99@gmail.com"
#define USER_PASSWORD "123456789"
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
unsigned long count = 0;
boolean  connectionStatus;
const String randomSerialNumber = String(random(1000,2000));
//....................................................end firebase........................

DFRobot_BME680_I2C bme(0x77);
LSM9DS1 imu;
#define PRINT_SPEED 1000
static unsigned long lastPrint = 0;
#define DECLINATION -8.58

void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
bool areTheySame(float arr1[], float arr2[], int n);
bool checkMovement();

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

void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N2, RXD2, TXD2);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
        connectionStatus = false;
    }

    connectionStatus = true;

    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the api key (required) */
    config.api_key = API_KEY;

    /* Assign the user sign in credentials */
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    /* Assign the RTDB URL (required) */
    config.database_url = DATABASE_URL;

    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    Firebase.setDoubleDigits(5);
    Serial.println(randomSerialNumber);

    Wire.begin();

    uint8_t rslt = 1;
    while (!Serial)
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

    if (imu.begin() == false) // (AG:0x6B, M:0x1E) and i2c port (Wire).
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
        imu.begin();
        Serial.println("imu ready");
    }
}

void loop()
{
    if (imu.gyroAvailable())
    {
        imu.readGyro();
        printGyro();
    }
    if (imu.accelAvailable())
    {
        imu.readAccel();
        printAccel();
    }
    if (imu.magAvailable())
    {
        imu.readMag();
        printMag();
    }
    if ((lastPrint + PRINT_SPEED) < millis())
    {
        printGyro();  // Print "G: gx, gy, gz"
        printAccel(); // Print "A: ax, ay, az"
        printMag();   // Print "M: mx, my, mz"
        printAttitude(imu.ax, imu.ay, imu.az,
                      -imu.my, -imu.mx, imu.mz);
        Serial.println();
        Serial.println(checkMovement());
        Firebase.setBool(fbdo, "User1/Movement", checkMovement());
        Serial.println();
        lastPrint = millis(); // Update lastPrint time
    }

    bme.startConvert();
    delay(1000);
    bme.update();
    Serial.print("temperature(C) :");
    Serial.println(bme.readTemperature() / 100, 2);
    Firebase.setFloat(fbdo, "User1/Temperature" , bme.readTemperature()/100);
    Serial.print("humidity(%rh) :");
    Serial.println(bme.readHumidity() / 1000, 2);
    Firebase.setFloat(fbdo, "User1/Humudity" , bme.readHumidity()/1000);
    Serial.print("gas resistance(ohm) :");
    Serial.println(bme.readGasResistance());
    Firebase.setFloat(fbdo, "User1/GRes" , bme.readGasResistance()/1000);
    Serial.println();

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
        Firebase.setString(fbdo, "User1/Connection", nmea[1]);
        Serial.println("Latitude, Longtitude: " + nmea[2] + "," + " " + nmea[4]);
        Firebase.setString(fbdo, "User1/Location", nmea[2] + "," + " " + nmea[4]);
        Serial.println();
    }
    else
    {
        failedUpdates++;
    }
    stringplace = 0;
    pos = 0;
    delay(1000);
}

void printGyro()
{
    Serial.print("G: ");
    Serial.print(imu.calcGyro(imu.gx), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gy), 2);
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gz), 2);
    Serial.println(" deg/s");
}

void printAccel()
{
    Serial.print("A: ");
    Serial.print(imu.calcAccel(imu.ax), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.ay), 2);
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.az), 2);
    Serial.println(" g");
}

void printMag()
{
    Serial.print("M: ");
    Serial.print(imu.calcMag(imu.mx), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.my), 2);
    Serial.print(", ");
    Serial.print(imu.calcMag(imu.mz), 2);
    Serial.println(" gauss");
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

bool areTheySame(float arr1[], float arr2[], int n)
{
    for (int i = 0; i < n; i++)
        if (arr1[i] - arr2[i] > 0.5)
        {
            return false;
        }
    return true;
}

bool checkMovement()
{
    float gx = imu.calcGyro(imu.gx);
    float gy = imu.calcGyro(imu.gy);
    float gz = imu.calcGyro(imu.gz);
    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);
    float mx = imu.calcMag(imu.mx);
    float my = imu.calcMag(imu.my);
    float mz = imu.calcMag(imu.mz);

    float gyroArr[] = {gx, gy, gz};
    float accelArr[] = {ax, ay, az};
    float magArr[] = {mx, my, mz};

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

    float hprArr[] = {heading, pitch, roll};

    //................................delay part............................................//
    delay(1000);
    //................................delay part............................................//

    float gx2 = imu.calcGyro(imu.ax);
    float gy2 = imu.calcGyro(imu.gy);
    float gz2 = imu.calcGyro(imu.gz);
    float ax2 = imu.calcAccel(imu.ax);
    float ay2 = imu.calcAccel(imu.ay);
    float az2 = imu.calcAccel(imu.az);
    float mx2 = imu.calcMag(imu.mx);
    float my2 = imu.calcMag(imu.my);
    float mz2 = imu.calcMag(imu.mz);

    float gyroArr2[] = {gx2, gy2, gz2};
    float accelArr2[] = {ax2, ay2, az2};
    float magArr2[] = {mx2, my2, mz2};

    float roll2 = atan2(ay, az);
    float pitch2 = atan2(-ax, sqrt(ay * ay + az * az));

    float heading2;
    if (my == 0)
        heading2 = (mx < 0) ? PI : 0;
    else
        heading2 = atan2(mx, my);

    heading2 -= DECLINATION * PI / 180;

    if (heading2 > PI)
        heading2 -= (2 * PI);
    else if (heading2 < -PI)
        heading2 += (2 * PI);

    // Convert everything from radians to degrees:
    heading2 *= 180.0 / PI;
    pitch2 *= 180.0 / PI;
    roll2 *= 180.0 / PI;

    float hprArr2[] = {heading2, pitch2, roll2};

    if (areTheySame(gyroArr, gyroArr2, 3) == false || areTheySame(accelArr, accelArr2, 3) == false || areTheySame(magArr, magArr2, 3) == false || areTheySame(hprArr, hprArr2, 3) == false)
    {
        return true;
    }
    return false;
}
