const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;
int val=0;
unsigned long time;
double rpmDeltaT=1000;
unsigned long lastTime;
double rpm;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), test, CHANGE);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(ledPin, state);
  //Serial.println(val/2);
  time = millis();
  //Serial.println(time);
  if(time-lastTime>=rpmDeltaT){
    rpm=60000*(val/2)/rpmDeltaT;
    Serial.println(rpm);
    val=0;
    lastTime=time;
  }
}

void test() {
  state = !state;
  val++;
}
