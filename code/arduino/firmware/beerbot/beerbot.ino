#include <Servo.h>

#define GRIPPER_ID 'A'
#define LEFT_WHEEL_ID 'B'
#define RIGHT_WHEEL_ID 'C'

#define LEFT_ZERO 1415
#define RIGHT_ZERO 1385
#define OPEN_GRIPPER 86
#define CLOSE_GRIPPER 120

Servo pinza;
Servo derecho;
Servo izquierdo;




void setup()
{
  Serial.begin(19200);
  pinza.attach(8);
  derecho.attach(9);
  izquierdo.attach(10);
  //pinMode(12, OUTPUT);

  izquierdo.write(LEFT_ZERO);
  derecho.write(RIGHT_ZERO);


}

char command;
char value;

void loop()
{
  if (Serial.available()) {
    Serial.print("available ");

    command = Serial.read();

    Serial.println(command);
    if (command == GRIPPER_ID) {
      value = Serial.read();
      Serial.print("value ");
      Serial.println((int)value);

      if (value == 'a') pinza.write(OPEN_GRIPPER);
      else if (value == 'b') pinza.write(CLOSE_GRIPPER);
      else Serial.flush();
    }
    else if (command == LEFT_WHEEL_ID) {
      value = Serial.read();
      Serial.print("value ");
      Serial.println((int)value);
      izquierdo.write(LEFT_ZERO + (int)value);
    }
    else if (command == RIGHT_WHEEL_ID) {
      value = Serial.read();
      Serial.print("value ");
      Serial.println((int)value);
      derecho.write(RIGHT_ZERO - (int)value);
    }

    Serial.flush();
  }
  else {
    Serial.println("MAL");
  }
  delay(1000);
}



