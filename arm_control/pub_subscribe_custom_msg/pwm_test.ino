int pwm_pin = 6;

void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22)| _BV(CS21)| _BV(CS20);           //
  OCR2A = 156;
  OCR2B = 4;
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(3, 13);
}
