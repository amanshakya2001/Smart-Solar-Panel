#include <Wire.h>
#include <ds3231.h>
#include<Servo.h>

#define SUNDIAMETER 0.53  // Sun diameter in degrees
#define AIRREFRACTION (34.0/60.0) // athmospheric refraction in degrees

Servo MyServo1, MyServo2;

struct ts t; 


double myLat = 27.603482;  //south latitude negative
double myLon = 77.595686; //west longitude negative
float TZ = 5.50 ; //west negative
double FNday (int y, int m, int d, float h) 
{
  //   Get the days to J2000
  //   h is UT in decimal hours
  //   FNday only works between 1901 to 2099 - see Meeus chapter 7
  long int count = - 7 * (y + (m + 9)/12)/4 + 275*m/9 + d;
  // type casting necessary on PC DOS and TClite to avoid overflow
  count+= (long int)y*367;
  return (double)count - 730531.5 + h/24.0;
}

double FNrange (double x) 
{
  //  the function returns an angle in the range 0 to 2*PI
  double b = x / TWO_PI;
  double a = TWO_PI * (b - (long)(b));
  if (a < 0) a = TWO_PI + a;
  return a;
}

double f0(double lat, double declin) 
{
  // Calculating the hourangle
  double fo,dfo;
  // Correction: different sign at S HS
  dfo = DEG_TO_RAD*(0.5*SUNDIAMETER + AIRREFRACTION); if (lat < 0.0) dfo = -dfo;
  fo = tan(declin + dfo) * tan(lat*DEG_TO_RAD);
  if (fo>0.99999) fo=1.0; // to avoid overflow //
  fo = asin(fo) + PI/2.0;
  return fo;
}

double f1(double lat, double declin) 
{
  // Calculating the hourangle for twilight times
  double fi,df1;
  // Correction: different sign at S HS
  df1 = DEG_TO_RAD * 6.0; if (lat < 0.0) df1 = -df1;
  fi = tan(declin + df1) * tan(lat*DEG_TO_RAD);
  if (fi>0.99999) fi=1.0; // to avoid overflow //
  fi = asin(fi) + PI/2.0;
  return fi;
}

double FNsun (double d, double &L) 
{
  //   Find the ecliptic longitude of the Sun
  double g;
  //   mean longitude of the Sun
  L = FNrange(280.461 * DEG_TO_RAD + .9856474 * DEG_TO_RAD * d);

  //   mean anomaly of the Sun
  g = FNrange(357.528 * DEG_TO_RAD + .9856003 * DEG_TO_RAD * d);

  //   Ecliptic longitude of the Sun
  return FNrange(L + 1.915 * DEG_TO_RAD * sin(g) + .02 * DEG_TO_RAD * sin(2 * g));
};


void sunCalc(int year, int month, int day, float timezone, double latitude, double longitude, int &sunrise, int &sunset, int &dawn, int &dusk)
// calculates times of morning dawn, sunrise, sunset, evening dusk
{
  double L, daylen;
  double h = 12; // assume sun position at high noon
  double d = FNday(year, month, day, h);

  //   Use FNsun to find the ecliptic longitude of the Sun
  double lambda = FNsun(d, L);

  //   Obliquity of the ecliptic
  double obliq = 23.439 * DEG_TO_RAD - .0000004 * DEG_TO_RAD * d;

  //   Find the RA and DEC of the Sun
  double alpha = atan2(cos(obliq) * sin(lambda), cos(lambda));
  double delta = asin(sin(obliq) * sin(lambda));

  // Find the Equation of Time
  // in minutes
  // Correction suggested by David Smith
  double LL = L - alpha;
  if (L < PI) LL += TWO_PI;
  double equation = 1440.0 * (1.0 - LL / TWO_PI);
  double ha = f0(latitude,delta);
  double hb = f1(latitude,delta);
  double twx = hb - ha; // length of twilight in radians
  twx = 12.0*twx/PI;    // length of twilight in hours
  // Conversion of angle to hours and minutes //
  daylen = RAD_TO_DEG*ha/7.5;
  if (daylen<0.0001) {daylen = 0.0;}
  // arctic winter //

  double riset = 12.0 - 12.0 * ha/PI + timezone - longitude/15.0 + equation/60.0;
  double settm = 12.0 + 12.0 * ha/PI + timezone - longitude/15.0 + equation/60.0;
 // double noont = riset + 12.0 * ha/PI;
  double altmax = 90.0 + delta * RAD_TO_DEG - latitude; 
  // Correction for S HS suggested by David Smith
  // to express altitude as degrees from the N horizon
  if (latitude < delta * RAD_TO_DEG) altmax = 180.0 - altmax;

  double twam = riset - twx;      // morning twilight begin
  double twpm = settm + twx;      // evening twilight end

  if (riset > 24.0) riset-= 24.0;
  if (settm > 24.0) settm-= 24.0;

  sunrise=round(riset*60); //minutes past midnight
  sunset=round(settm*60);
  dawn=round(twam*60);
  dusk=round(twpm*60);
}

void setup() {

  MyServo1.attach(9);
  MyServo2.attach(10);
  Serial.begin(9600);
  Wire.begin();
  DS3231_init(DS3231_CONTROL_INTCN);
 }

void loop() 
{
  DS3231_get(&t);
  Serial.print("Date : ");
  Serial.print(t.mday);
  Serial.print("/");
  Serial.print(t.mon);
  Serial.print("/");
  Serial.print(t.year);
  Serial.print("\t Hour : ");
  Serial.print(t.hour);
  Serial.print(":");
  Serial.print(t.min);
  Serial.print(".");
  Serial.println(t.sec);

  
  int sunrise,sunset,dawn,dusk;
  sunCalc(t.year, t.mon,t.mday, TZ, myLat, myLon, sunrise, sunset, dawn, dusk);
  char buf[30];
  
  snprintf(buf,sizeof(buf),"Sunrise %02d:%02d, Sunset %02d:%02d",sunrise/60,sunrise%60,sunset/60,sunset%60);
  Serial.println(buf); 
    int difference = ((t.hour-(sunrise/60))*60)+ (t.min- (sunrise%60));
    float angle = difference * 0.25;
    
    if(angle>=0 && angle <=180){
      MyServo1.write(angle);
    if(angle<=90)
    {
      MyServo2.write(45);
    }
    else
    {
      MyServo2.write(45);
    }
    }
    else if(angle==181)
    {
       for(int angle=180;angle>=0;angle--)
      {
        MyServo1.write(angle);
        delay(50);
      }
    }
    else
    {
      MyServo1.write(0);
    }
    delay(5000);
}
