int randomNum;

void setup() {
  // put your setup code here, to run once:
  randomSeed(20);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  randomNum = random(2);
  Serial.print("random :");
  Serial.println(randomNum);
}
