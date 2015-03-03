#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>
#include <RobotOpen.h>



/* I/O Setup */
ROJoystick usb1(1);         // Joystick #1
ROPWM pwm0(0);
ROPWM pwm1(1);
ROPWM pwm2(2);
ROPWM pwm3(3);
ROPWM pwm4(4);
ROPWM pwm5(5);
int restrict;
RODigitalIO dig0Out(0,OUTPUT);
RODigitalIO dig1Out(1,OUTPUT);
RODigitalIO dig2Out(2,OUTPUT);
RODigitalIO PressureSensor(19,INPUT);
ROSolenoid sol0(0);
ROSolenoid sol1(1);

void setup()
{
  /* initiate comms */
  RobotOpen.begin(&enabled,&disabled, &timedtasks);
}
void enabled() {
  // get desired translation and rotation, scaled to [-127..128] (0 neutral)
  int x = usb1.leftX() - 127;
  int y = (255 - usb1.leftY()) - 127;
  int rotate = usb1.rightX() - 127;
  restrict = 3 - (min(usb1.rTrigger(),usb1.lTrigger())/127);
  
  
  // calculate wheel throttles
  int lf = (x + y + rotate)/restrict;
  int rf = (x - y + rotate)/restrict;
  int lr = (-x + y + rotate)/restrict;
  int rr = (-x - y + rotate)/restrict;
  
  // normalize wheel throttles
  int maximum = max(max(abs(lf), abs(rf)), max(abs(lr), abs(rr)));
  if (maximum > 127) {
    lf = (lf / maximum) * 127;
    rf = (rf / maximum) * 127;
    lr = (lr / maximum) * 127;
    rr = (rr / maximum) * 127;
  }

  // Set PWMs, shifted back to [0..255]
  pwm0.write(lf + 127);
  pwm1.write(rf + 127);
  pwm2.write(lr + 127);
  pwm3.write(rr + 127);
  if (usb1.btnLShoulder())
  {
    pwm4.write(127-127/2);
    pwm5.write(127+127/2);
  }
  else if (usb1.btnRShoulder())
  {
    pwm4.write(127+127/2);
    pwm5.write(127-127/2);
  }
  else
  {
    pwm4.write(127);
    pwm5.write(127);
  }
  if (usb1.btnY()){
    dig0Out.off();
    dig1Out.off();
    dig2Out.on();
    RODashboard.debug("sensor high");}
    else{
    dig0Out.off();
    dig1Out.off();
    dig2Out.off();
    RODashboard.debug("sensor low");}
  if(usb1.btnA())
     sol0.on();
  else
    sol0.off();
  if(usb1.btnB())
    sol1.on();
  else
    sol1.off();
    
  RODashboard.publish("sensorinput", PressureSensor.read());
 
}



/* This is called while the robot is disabled
 * All outputs are automatically disabled (PWM, Solenoid, Digital Outs)
 */
void disabled() {
  // safety code
}


/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code
 */
void timedtasks() {
  RODashboard.publish("Uptime Seconds", ROStatus.uptimeSeconds());
  RODashboard.publish("restrict", restrict);
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
