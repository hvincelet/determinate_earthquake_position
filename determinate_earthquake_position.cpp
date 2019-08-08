/*
  Nécessite la bibliothèque STL pour Arduino.
  Pour l'installer, aller dans Outils>Gérer les bibliothèques
  Rechercher par le mot-clé "ArduinoSTL", puis installer la première proposée (développée par Mike Matera)
*/

#include <SoftwareSerial.h>
#include <ArduinoSTL.h>

SoftwareSerial BTserial(10, 11); // Rx | Tx

#define SensorLED   13
#define SensorINPUT  12
#define MASTER 0
#define SLAVE 1
#define SPEED_CONSTANT 1.94

#define C1_X -3
#define C1_Y 3
#define C2_X 9
#define C2_Y 10
#define C3_X 0
#define C3_Y 0

unsigned char sensor_state = 0;

struct wave
{
  bool state;
  unsigned long timestamp;
  void save_timestamp()
  {
    this->timestamp = micros();
    this->state = 1;
    sensor_state = 0;
    digitalWrite(SensorLED, HIGH);
    delay(1);
    digitalWrite(SensorLED, LOW);
  }
};

struct station
{
  const unsigned char address;
  const double distance;
  station(const unsigned char a, const double d) : address(a), distance(d) {}
  station(const String& data) :
    address(data[0]),
    distance(data.substring(1, 9).toDouble()) // sizeof(double) = 8
  {}
};

struct coordinates
{
  double x;
  double y;
};

wave p{0, 0}, s{0, 0};
const unsigned char site_status = MASTER;
const unsigned char ADDRESS = 3; // C1, C2, C3
std::vector<station> station_results;
const std::vector<coordinates> station_coordinates(
  {
    coordinates{C1_X, C1_Y},
    coordinates{C2_X, C2_Y},
    coordinates{C3_X, C3_Y}
  }
);

int calculate_earthquake_coordinates(coordinates& result)
{
  const auto delta = [](const double& a, const double& b, const double& c) -> double
  {
    return sqrt((b*b) - (4*a*c));
  };
  const station& c0 = station_results[0];
  const station& c1 = station_results[1];
  const station& c2 = station_results[2];
  const double& r0 = c0.distance;
  const double& r1 = c1.distance;
  const double& r2 = c2.distance;
  const double& x0 = station_coordinates[c0.address].x;
  const double& y0 = station_coordinates[c0.address].y;
  const double& x1 = station_coordinates[c1.address].x;
  const double& y1 = station_coordinates[c1.address].y;
  const double& x2 = station_coordinates[c2.address].x;
  const double& y2 = station_coordinates[c2.address].y;
  const auto find_real_case_with_last_station = [&delta, &result, &r2, &x2, &y2](const double& xa, const double& ya, const double& xb, const double& yb) -> void
  {
    const double a = (yb-ya) / (xb-xa);
    const double b = ya - (((yb - ya) / (xb - xa)) * xa);
    const double A = (a*a)+1;
    const double B = 2 * a * (b - y2) - 2 * x2;
    const double C = ((b - y2)*(b - y2)) + (x2*x2);
    const double d = delta(A,B,C);
    const double x_case_1 = (-B-d)/(2*A);
    const double x_case_2 = (d-B)/(2*A);
    
  };
  if(y0 == y0)
  {
    if(x0 == x1) return 1;
    const double A = 1.0;
    const double B = -(2*y1);
    const double x = ((r1*r1) - (r0*r0) - (x1*x1) + (x0*x0)) / (2*(x0 - x1));
    const double C = (x1*x1) + (x*x) - (2*x1*x) + (y1*y1) - (r1*r1);
    const double d = delta(A,B,C);
    const double y_case_1 = (-B-d)/(2*A);
    const double y_case_2 = (d-B)/(2*A);
    find_real_case_with_last_station(x, y_case_1, x, y_case_2);
  }
  else
  {
    const double A = (((x0 - x1)/(y0 - y1)) * ((x0 - x1)/(y0 - y1))) + 1;
    const double N = ((r1*r1) - (r0*r0) - (x1*x1) + (x0*x0) - (y1*y1) + (y0*y0)) / (2 * (y0 - y1));
    const double B = 2 * y0 * ((x0 - x1)/(y0 - y1)) - (2 * N * ((x0 - x1)/(y0 - y1))) - (2 * x0);
    const double C = (x0*x0) + (y0*y0) + (N*N) - (r0*r0) - (2 * y0 * N);
    const double d = delta(A,B,C);
    const double x_case_1 = (-B-d)/(2*A);
    const double x_case_2 = (d-B)/(2*A);
    const auto calculate_y = [&N, &x0, &x1, &y0, &y1](const double& x) -> double
    {
      return N - (x * ((x0 - x1)/(y0 - y1)));
    };
    const double y_case_1 = calculate_y(x_case_1);
    const double y_case_2 = calculate_y(x_case_2);
    find_real_case_with_last_station(x_case_1, y_case_1, x_case_2, y_case_2);
  }
   return 0;
}

void blink()
{
  sensor_state++;
}

void send(const station source_station)
{
  
}

void receive_data()
{
  if (BTserial.available())
  {
    char c;
    String data = "";
    while(c = BTserial.read())
    {
      data + c;
    }
    if(data.length() != sizeof(station)) return;
    const station source_station(data);
    if(site_status == MASTER)
    {
      station_results.push_back(source_station);
      if(station_results.size() == 3)
      {
        coordinates result;
        const int status = calculate_earthquake_coordinates(result);
        station_results.clear();
      }
    }
    else
    {
      send(source_station); // Slave transfer to its neighbour
    }
  }
}

void setup() {
  BTserial.begin(9600);
  pinMode(SensorLED, OUTPUT);
  pinMode(SensorINPUT, INPUT);
  attachInterrupt(1, blink, FALLING); 
}


void loop() {
  receive_data();
  if(sensor_state != 0)
  {
    if(p.state == 0)
    {
      p.save_timestamp();
    }
    else if(s.state == 0)
    {
      s.save_timestamp();
    }
    if(p.state != 0 && s.state != 0)
    {
      if(site_status == SLAVE)
      {
        send(station{ADDRESS, (s.timestamp - p.timestamp) * SPEED_CONSTANT});
        p.state = 0;
        s.state = 0;
      }
      else
      {
        station_results.push_back(station{ADDRESS, (s.timestamp - p.timestamp) * SPEED_CONSTANT});
        p.state = 0;
        s.state = 0;
        if(station_results.size() == 3)
        {
          coordinates result;
          const int status = calculate_earthquake_coordinates(result);
          station_results.clear();
        }
      }
    }
  }
}
