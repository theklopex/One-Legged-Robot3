#include <Servo.h>

Servo hip;
Servo knee;

#define SERVOMIN  0
#define SERVOMAX  180

typedef struct Position
{
    uint16_t hip;
    uint16_t knee;
} Position;

Position curPos;
double femurLen;
double tibiaLen;
uint16_t lapCounter;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("\n\n\nThe one legged robot wants to walk!");

  femurLen = 5.8; //(cm)
  tibiaLen = 8.0; //(cm)

  curPos.hip = 90;
  curPos.knee = 90;

  hip.attach(8);
  knee.attach(9);

  lapCounter = 0;
}

void
printPosition(
  Position pos)
{
    Serial.print("<<hip = ");
    Serial.print(pos.hip);
    Serial.print(", knee = ");
    Serial.print(pos.knee);
    Serial.print(">>");
}

uint16_t
fixBounds(
  uint16_t pulselen)
{
  if (pulselen > SERVOMAX)
  {
    return SERVOMAX;
  }
  if (pulselen < SERVOMIN)
  {
    return SERVOMIN;
  }
  return pulselen;
}

void
stepTo(
  Position pos)
{
  uint16_t sensorValue;

  pos.hip = fixBounds(pos.hip);
  hip.write(pos.hip);              // tell servo to go to position in variable 'pos' 

  pos.knee = fixBounds(pos.knee);
  knee.write(pos.knee);              // tell servo to go to position in variable 'pos' 
  delay(5);                       // waits 15ms for the servos to reach the position 
}

uint8_t stepAmount = 5;

Position
advancePosition(
  Position oldPos,
  Position newPos)
{
  Position nextPos;
  bool debug = false;

  if (debug)
  {
    Serial.print("oldPos = ");
    printPosition(oldPos);
    Serial.print(" newPos = ");
    printPosition(newPos);
  }
  
  nextPos.hip = oldPos.hip;
  if (newPos.hip > oldPos.hip)
  {
    nextPos.hip += stepAmount;
    if (nextPos.hip > newPos.hip)
    {
      // Fix it if we overstep.
      nextPos.hip = newPos.hip;
    }
  }
  if (newPos.hip < oldPos.hip)
  {
    // This avoids going "negative" with unsigned integers
    if ((oldPos.hip - newPos.hip) < stepAmount)
    {
      // Fix it if we overstep.
      nextPos.hip = newPos.hip;
    }
    else
    {
      nextPos.hip -= stepAmount;
    }
  }

  nextPos.knee = oldPos.knee;
  if (newPos.knee > oldPos.knee)
  {
    nextPos.knee += stepAmount;
    if (nextPos.knee > newPos.knee)
    {
      // Fix it if we overstep.
      nextPos.knee = newPos.knee;
    }
  }
  if (newPos.knee < oldPos.knee)
  {
    // This avoids going "negative" with unsigned integers
    if ((oldPos.knee - newPos.knee) < stepAmount)
    {
      // Fix it if we overstep.
      nextPos.knee = newPos.knee;
    }
    else
    {
      nextPos.knee -= stepAmount;
    }
  }

  if (debug)
  {
    Serial.print("nextPos = ");
    printPosition(nextPos);
    Serial.print("\n");
  }
  return nextPos;
}

bool
operator==(
  Position pLeft,
  Position pRight)
{
  if (pLeft.hip != pRight.hip)
  {
    return false;
  }
  if (pLeft.knee != pRight.knee)
  {
    return false;
  }

  return true;
}

bool
operator!=(Position pLeft, Position pRight)
{
  return !(pLeft == pRight);
}

Position
glideTo(
  Position oldPos,
  Position newPos)
{
  bool debug = false;
  Position curPos;

  if (debug)
  {
    Serial.print("glideTo(");
    printPosition(oldPos);
    Serial.print("->");
    printPosition(newPos);
    Serial.print(")\n");
  }
  if (oldPos.hip == newPos.hip)
  {
    // Apparently, they are equal.  Make them unequal to guarantee that we set the servo.
    if (debug)
    {
      Serial.println("They were equal");
    }
    oldPos.hip += stepAmount;
  }

  curPos = oldPos;
  do
  {
    if (debug)
    {
      Serial.print("cur = ");
      printPosition(curPos);
      Serial.print("  new = ");
      printPosition(newPos);
    }

    curPos = advancePosition(curPos, newPos);
    if (debug)
    {
      Serial.print("new cur = ");
      printPosition(curPos);
      Serial.print("\n");
    }
    stepTo(curPos);
  } while (!(newPos == curPos));

  if (debug)
  {
    Serial.print("New Position = ");
    printPosition(curPos);
    Serial.print("\n");
  }
  return curPos;
}

Position
anglesForLocation(
  double x,
  double y)
{
  double distFromHipToFoot;
  double hipAngleFromFootToKnee;
  double hipAngleFromLevelToFoot;
  double kneeAngleFromHipToFoot;
  Position newPos;

  //  Angles are capitals
  //  side lengths are lower-case
  //
  // |   (c,d)     f
  // |     *--------------* (a,b)
  // |    / G      H __-- |
  // | h /       __--     |
  // |  /    __--g        | b
  // | /F__--             |
  // |/-- B               |
  // +--------------------------
  //             a
  // (a==x), (b==y)
  
  Serial.print("x = ");
  Serial.println(x);

  Serial.print("y = ");
  Serial.println(y);

  // g = (a^2 + b^2)^(1/2)
  distFromHipToFoot = sqrt(pow(x,2) + pow(y,2));
  Serial.print("distFromHipToFoot = ");
  Serial.println(distFromHipToFoot);

  // B = arcsin(b/g) (for 0<=B<=90)
  hipAngleFromLevelToFoot = asin(y / distFromHipToFoot);
  Serial.print("hipAngleFromLevelToFoot = ");
  Serial.println(hipAngleFromLevelToFoot);

  // F = arccos((f^2 - h^2 - g^2)/(-2hg)) (for 0<=F<=180)
  hipAngleFromFootToKnee = acos( (pow(tibiaLen,2) - pow(femurLen,2) - pow(distFromHipToFoot,2)) / (-2 * femurLen * distFromHipToFoot));
  Serial.print("hipAngleFromFootToKnee = ");
  Serial.println(hipAngleFromFootToKnee);

  // G = arcsin((g * sin(F))/f) (for 0<=G<=90) no good.
  // kneeAngleFromHipToFoot = asin((distFromHipToFoot * sin(hipAngleFromFootToKnee))/tibiaLen);
  // G = arccos((g^2 - h^2 - f^2) / (-2hf)) (for 0<=G<=180)
  kneeAngleFromHipToFoot = acos( (pow(distFromHipToFoot,2) - pow(femurLen,2) - pow(tibiaLen,2)) / (-2 * femurLen * tibiaLen));
  Serial.print("kneeAngleFromHipToFoot = ");
  Serial.println(kneeAngleFromHipToFoot);

  // Add .5 so that the truncation will work like rounding
  // (this self-corrects because of truncation, so we do not have to subtract .5 later.)
  newPos.hip = (uint16_t) (0.5 + ((hipAngleFromLevelToFoot + hipAngleFromFootToKnee) * 180.0/(M_PI)));
  newPos.knee = (uint16_t) (0.5 + (kneeAngleFromHipToFoot * 180.0/(M_PI)));
  // The knee angle is actually not G, but the external angle 180-G
  newPos.knee = 180 - newPos.knee;
  // Adjust for orientation of the servos.
  // The servos point the straight leg (level) when: hip=90 degrees, knee=0 degrees.
  Serial.print("newPosa = ");
  printPosition(newPos);
  Serial.print("\n");
  newPos.hip += 90;
  Serial.print("newPosb = ");
  printPosition(newPos);
  Serial.print("\n");

  return newPos;
}


typedef enum TestBehavior
{
    LEG_POSITIONS = 0,
    TEST_CIRCLE = 1,
    ROBOT_WALK_FORWARD = 2,
    ROBOT_WALK_BACKWARD = 3,
} TestBehavior;
TestBehavior doPart = TEST_CIRCLE;

void loop() {
  // put your main code here, to run repeatedly:
  Position nextPos;
  uint16_t i;
  double x;
  double y;
  double j;

  Serial.println("----------------------------------------");

  if (doPart == LEG_POSITIONS)
  {
    // straight leg
    nextPos = curPos;
    nextPos.hip = 90;
    nextPos.knee = 0;
    curPos = glideTo(curPos, nextPos);
    delay(1000);
  
    // raise hip and bend knee
    nextPos = curPos;
    nextPos.hip = 120;
    nextPos.knee = 140;
    curPos = glideTo(curPos, nextPos);
    delay(1000);
  
    //raise hip to top.
    nextPos = curPos;
    nextPos.hip = 180;
    // do not change the knee position
    curPos = glideTo(curPos, nextPos);
    delay(1000);
  
    //straighten knee
    // do not change the hip position
    nextPos = curPos;
    nextPos.knee = 0;
    curPos = glideTo(curPos, nextPos);
    delay(1000);
  }

  if (doPart == TEST_CIRCLE)
  {
    double radius = 2.0;
    // This is PI/2 * 1000.  round(3.141592/2 * 1000) = round(1.570796 * 1000) = round(1570.796) = 1571
    // So, it is 1/4 of a circle in radians.
    // PI/2 = 1571, PI = 3142, 3PI/2 = 4713, 2PI = 6284
    // 2 laps around a circle == 4PI == 8PI/2 = 12568
    uint16_t kiloHalfPi = 1571;
    for(i=0; i<6284; i+=392)
    {
      j = i/1000.0;
      Serial.print("i = ");
      Serial.print(i);
      Serial.print(",  j = ");
      Serial.println(j);
      x = 11.0 + (radius * sin(j));
      y = 2.0 + (radius * cos(j));
      nextPos = anglesForLocation(x, y);
      curPos = glideTo(curPos, nextPos);
    }
    if (lapCounter > 2)
    {
      lapCounter = 0;
      doPart = ROBOT_WALK_FORWARD;
    }
  }

  if (doPart == ROBOT_WALK_FORWARD)
  {
      nextPos = anglesForLocation(12, 2);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(12, -6);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(8, -6);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(4, -5);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(3, -5);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(4, 4);
      curPos = glideTo(curPos, nextPos);   
      delay(50);

      if (lapCounter > 10)
      {
        lapCounter = 0;
        doPart = ROBOT_WALK_BACKWARD;
      }
   }

  if (doPart == ROBOT_WALK_BACKWARD)
  {
      nextPos = anglesForLocation(6, 4);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(6, -4);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(12, -4);
      curPos = glideTo(curPos, nextPos);   
      delay(50);
      nextPos = anglesForLocation(12, 4);
      curPos = glideTo(curPos, nextPos);   
      delay(50);

      if (lapCounter > 5)
      {
        lapCounter = 0;
        doPart = ROBOT_WALK_FORWARD;
      }
  }

  lapCounter++;
  Serial.print("lapCounter = ");
  Serial.println(lapCounter);
}
