////////////////////////////////////////////////////////////////////////////////
/////// Author: Andres Zibula                                           ////////
/////// Source: https://github.com/andres-zibula/parallel-scara-stylus  ////////
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <Servo.h>

//Arduino pins for servos
#define S1_PIN 3
#define S2_PIN 4
#define S3_PIN 5

//first servo position
#define S1_POS_X 0
#define S1_POS_Y 0

//second servo position
#define S2_POS_X 50
#define S2_POS_Y 0

#define S1_OFFSET_ANGLE -13
#define S2_OFFSET_ANGLE -5

#define STICK1_LEN 75
#define STICK2_LEN 100
#define STICK3_LEN 36

#define LIFT_HEIGHT 32
#define DESCEND_HEIGHT 10

#define STEPS_PER_MM 1

#define REST_POS_X 32
#define REST_POS_Y 130
#define CENTER_POS_X 15
#define CENTER_POS_Y 150

#define SLIDE_RIGHT '0'
#define SLIDE_DOWN '1'
#define SLIDE_LEFT '2'
#define SLIDE_UP '3'
#define RESPONSE_OK '4'

#define SLIDE_LEN 15

Servo servo1;
Servo servo2;
Servo servo3;

double actual_x = 0;
double actual_y = 0;
bool is_lifted = false;

void go_to(double, double);
void lift_stylus();
void descend_stylus();

void slide(char direction)
{
  switch(direction)
  {
    case SLIDE_RIGHT:
      go_to(CENTER_POS_X, CENTER_POS_Y);
      delay(200);
      descend_stylus();
      delay(200);
      go_straight_line(actual_x, actual_y, actual_x, actual_y + SLIDE_LEN);
      delay(200);
      lift_stylus();
      delay(300);
      go_to(REST_POS_X, REST_POS_Y);
    break;

    case SLIDE_DOWN:
      go_to(CENTER_POS_X, CENTER_POS_Y);
      delay(200);
      descend_stylus();
      delay(200);
      go_straight_line(actual_x, actual_y, actual_x + SLIDE_LEN, actual_y);
      delay(200);
      lift_stylus();
      delay(300);
      go_to(REST_POS_X, REST_POS_Y);
    break;

    case SLIDE_LEFT:
      go_to(CENTER_POS_X, CENTER_POS_Y);
      delay(200);
      descend_stylus();
      delay(200);
      go_straight_line(actual_x, actual_y, actual_x, actual_y - SLIDE_LEN);
      delay(200);
      lift_stylus();
      delay(300);
      go_to(REST_POS_X, REST_POS_Y);
    break;

    case SLIDE_UP:
      go_to(CENTER_POS_X, CENTER_POS_Y);
      delay(200);
      descend_stylus();
      delay(200);
      go_straight_line(actual_x, actual_y, actual_x - SLIDE_LEN, actual_y);
      delay(200);
      lift_stylus();
      delay(300);
      go_to(REST_POS_X, REST_POS_Y);
    break;
  }
}

void lift_stylus()
{
  if(is_lifted)
    return;

  servo3.write(LIFT_HEIGHT);

  is_lifted = true;
}

void descend_stylus()
{
  if(!is_lifted)
    return;

  servo3.write(DESCEND_HEIGHT);

  is_lifted = false;
}

void go_straight_line(double x1, double y1, double x2, double y2)
{  
  go_to(x1, y1);

  double dx = x2 - x1;
  double dy = y2 - y1;  
  double c = round(STEPS_PER_MM * sqrt(dx*dx + dy*dy));
  
  for(int i = 1; i <= c; i++)
  {
    go_to(x1 + i*dx/c, y1 + i*dy/c);
  }

}

inline double cosine_angle_rule(double a, double b, double c)
{
  return acos((a*a + c*c - b*b) / (2*a*c));
}

inline double cosine_side_rule(double A, double b, double c)
{
  return sqrt(b*b + c*c - 2*b*c*cos(A));
}

inline double rad_to_deg(double rad)
{
  return rad * (180.0 / M_PI);
}

inline double pitagoras(double b, double c)
{
  return sqrt(b*b + c*c);
}

void go_to(double x, double y)
{
  //for a better understanding look at the following image:
  //https://github.com/andres-zibula/project-images/blob/master/parallel_scara_stylus/parallel_scara_stylus.jpeg

  double a = pitagoras(S2_POS_X - x, y - S2_POS_Y);
  double b = cosine_side_rule(M_PI - M_PI/4.0, STICK2_LEN, STICK3_LEN);
  double beta = atan2(y, (S2_POS_X - x)) + cosine_angle_rule(a, b, STICK1_LEN);
  
  double x1 = S2_POS_X + STICK1_LEN*cos(M_PI - beta);
  double y1 = S2_POS_Y + STICK1_LEN*sin(M_PI - beta);
  
  double delta = atan2((x1-x), (y-y1));

  double theta = cosine_angle_rule(b, STICK2_LEN, STICK3_LEN);
  
  double x2 = x + STICK3_LEN*sin(delta-theta);
  double y2 = y - STICK3_LEN*cos(delta-theta);
  
  double c = pitagoras(x2 - S1_POS_X, y2 - S1_POS_Y);

  double alpha = atan2((y2 - S1_POS_Y), (x2 - S1_POS_X)) + cosine_angle_rule(c, STICK2_LEN, STICK1_LEN);

  servo2.write((rad_to_deg(M_PI - beta)) + S2_OFFSET_ANGLE);
  servo1.write((rad_to_deg(alpha)) + S1_OFFSET_ANGLE);
  
  actual_x = x;
  actual_y = y;
}

void setup()
{
  Serial.begin(9600);
  
  servo1.attach(S1_PIN);
  servo2.attach(S2_PIN);
  servo3.attach(S3_PIN);

  go_to(REST_POS_X, REST_POS_Y);
  lift_stylus();
  delay(200);
}

void loop()
{
  delay(10);
}

void serialEvent()
{
  while (Serial.available())
  {
    char command = (char)Serial.read();

    slide(command);

    Serial.print(RESPONSE_OK);
    Serial.flush();
  }
}
