#define enA 13
#define enB 8
#define in1 12
#define in2 11
#define in3 10
#define in4 9

void MotorSetup()
{
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void RobotStop()
{
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void RobotMove(int leftRotation, int rightRotation)
{
  if (leftRotation < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    leftRotation = -1 * leftRotation;
  }
  else // forward
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  analogWrite(enA, leftRotation);

  if (rightRotation < 0)
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    rightRotation = -1 * rightRotation;
  }
  else // forward
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  analogWrite(enB, rightRotation);
}

void setup()
{
  MotorSetup();
}

void loop()
{
  RobotMove(60, 60);
}
