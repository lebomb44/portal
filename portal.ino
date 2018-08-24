#include <Fifo_U16.h>
#include <HomeEasy.h>
//#include <HT12E.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define LED_pin 13
#define RELAY_LEFT_pin 7
#define RELAY_RIGHT_pin 8
#define IR_POWER_pin 10
/* LIMITER used at start of CLOSE and end of OPEN */
#define LIMITER_LEFT_pin 5
/* LIMITER used at start of OPEN and end of CLOSE */
#define LIMITER_RIGHT_pin 3
#define RF_IN_pin 2
#define MOTOR_SENSE_pin A0
#define MOTOR_PWM_pin 11

#define PORTAL_FULL_SLOT ((PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT) * 4)
#define PORTAL_SLOW_SLOT 2000
#define PORTAL_CRUISE_SLOT 6000

#define PORTAL_CMD_CLOSE 0
#define PORTAL_CMD_OPEN  1

#define MOTOR_MAX_CURRENT 190
#define MOTOR_MAX_SLOW_CURRENT 190

HomeEasy homeEasy;
int portal_last_cmd = PORTAL_CMD_CLOSE;
int portal_cmd = PORTAL_CMD_CLOSE;
boolean portal_cmd_accepted = false;
uint16_t portal_start_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
uint16_t portal_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
uint16_t portal_force = 0;
int motor_max_current = MOTOR_MAX_CURRENT;
uint16_t motor_max_current_nb = 0;

uint16_t get_force(uint16_t _begin_position, uint16_t _position) {
  uint16_t _begin_force = 0;
  uint16_t _end_force = 0;
  uint16_t _force = 0;

  if(PORTAL_SLOW_SLOT > (_position - _begin_position)) {
    _begin_force = _position - _begin_position;
  }
  else {
    _begin_force = PORTAL_SLOW_SLOT;
  }

  if(PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT > _position) {
    _end_force = PORTAL_SLOW_SLOT;
  }
  else {
    if((PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT) < _position) {
      _position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
    }
    if((PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT) <= _position) {
      _end_force = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT - _position;
      if(PORTAL_SLOW_SLOT/2 > _end_force) {
        _end_force = PORTAL_SLOW_SLOT/4;
      }
    }
  }

  /* Set to MAX force to unstruck the portal */
  if(50 > _position) { _force = PORTAL_SLOW_SLOT; }
  /* Else normal work */
  else { _force = min(_begin_force, _end_force); }

  if((50 == _position - _begin_position) || (PORTAL_SLOW_SLOT == _position - _begin_position) || (PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT == _position) || (PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT - 1 == _position)) {
    Serial.print("FORCE at "); Serial.print(_position, DEC); Serial.print(" (begin at "); Serial.print(_begin_position, DEC); Serial.print(") = "); Serial.println(_force, DEC);
  }
  return _force;
}

void setup()
{
  homeEasy.init();
  // initialize serial communications and wait for port to open:
  Serial.begin(115200);
  pinMode(13, OUTPUT); 
  pinMode(RELAY_LEFT_pin, OUTPUT);
  digitalWrite(RELAY_LEFT_pin, LOW);
  pinMode(RELAY_RIGHT_pin, OUTPUT);
  digitalWrite(RELAY_RIGHT_pin, LOW);
  pinMode(IR_POWER_pin, OUTPUT);
  digitalWrite(IR_POWER_pin, LOW);
  pinMode(LIMITER_LEFT_pin, INPUT);
  pinMode(LIMITER_RIGHT_pin, INPUT);
  pinMode(RF_IN_pin, INPUT);
  pinMode(MOTOR_SENSE_pin, INPUT);
  pinMode(MOTOR_PWM_pin, OUTPUT);
  digitalWrite(MOTOR_PWM_pin, LOW);

  /* Disable Timer 0 */
  TCCR0B = TCCR0B & 0xF8;
}

void loop()
{
  homeEasy.run();
  if(true == homeEasy.rxCodeIsReady()) {
    Serial.print(homeEasy.rxGetCode(), HEX);Serial.print(" : ");
    Serial.print(homeEasy.rxGetManufacturer(), HEX);Serial.print("-");
    Serial.print(homeEasy.rxGetGroup(), HEX);Serial.print("-");
    Serial.print(homeEasy.rxGetDevice(), HEX);Serial.print("-");
    Serial.print(homeEasy.rxGetStatus(), HEX);Serial.println();
    _delay_ms(500);
    homeEasy.purge();
    /* Check the authorized codes */
    if(((0xFCE1CE == homeEasy.rxGetManufacturer()) && (0x0 == homeEasy.rxGetGroup()) && (0x2 == homeEasy.rxGetDevice())) \
    || ((0xFCBDD6 == homeEasy.rxGetManufacturer()) && (0x0 == homeEasy.rxGetGroup()) && (0x2 == homeEasy.rxGetDevice())) \
    || ((0xFCDAD2 == homeEasy.rxGetManufacturer()) && (0x0 == homeEasy.rxGetGroup()) && (0x2 == homeEasy.rxGetDevice())) \
    || ((0xFCC302 == homeEasy.rxGetManufacturer()) && (0x0 == homeEasy.rxGetGroup()) && (0x2 == homeEasy.rxGetDevice()))) {
      digitalWrite(IR_POWER_pin, HIGH);
      if(PORTAL_CMD_CLOSE == homeEasy.rxGetStatus()) {
        if(LOW == digitalRead(LIMITER_RIGHT_pin)) {
          digitalWrite(RELAY_LEFT_pin, HIGH);
          digitalWrite(RELAY_RIGHT_pin, LOW);
          portal_last_cmd = portal_cmd;
          portal_cmd = PORTAL_CMD_CLOSE;
          if((PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT) < portal_position) {
            portal_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
          }
          if(PORTAL_CMD_CLOSE == portal_last_cmd) {
            portal_start_position = portal_position;
          }
          if(PORTAL_CMD_OPEN == portal_last_cmd) {
            portal_start_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT - portal_position;
            portal_position = portal_start_position;
          }
          if(HIGH == digitalRead(LIMITER_LEFT_pin)) {
            portal_start_position = 0;
            portal_position = portal_start_position;
            Serial.println("CLOSE from LIMITER");
          }
          portal_cmd_accepted = true;
          Serial.print("CLOSE at: "); Serial.println(portal_position, DEC);
        }
        else {
          /* Use this new command to reset to known position */
          digitalWrite(MOTOR_PWM_pin, LOW);
          digitalWrite(RELAY_LEFT_pin, LOW);
          digitalWrite(RELAY_RIGHT_pin, LOW);
          digitalWrite(IR_POWER_pin, LOW);
          portal_start_position = PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
          portal_position = portal_start_position;
          portal_cmd_accepted = false;
          portal_force = 0;
          Serial.println("Already CLOSE");
        }
      }
      if(PORTAL_CMD_OPEN == homeEasy.rxGetStatus()) {
        if(LOW == digitalRead(LIMITER_LEFT_pin)) {
          digitalWrite(RELAY_LEFT_pin, LOW);
          digitalWrite(RELAY_RIGHT_pin, HIGH);
          portal_last_cmd = portal_cmd;
          portal_cmd = PORTAL_CMD_OPEN;
          if((PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT) < portal_position) {
            portal_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
          }
          if(PORTAL_CMD_OPEN == portal_last_cmd) {
            portal_start_position = portal_position;
          }
          if(PORTAL_CMD_CLOSE == portal_last_cmd) {
            portal_start_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT - portal_position;
            portal_position = portal_start_position;
          }
          if(HIGH == digitalRead(LIMITER_RIGHT_pin)) {
            portal_start_position = 0;
            portal_position = portal_start_position;
            Serial.println("OPEN from LIMITER");
          }
          portal_cmd_accepted = true;
          Serial.print("OPEN at: "); Serial.println(portal_position, DEC);
        }
        else {
          /* Use this new command to reset to known position */
          digitalWrite(MOTOR_PWM_pin, LOW);
          digitalWrite(RELAY_LEFT_pin, LOW);
          digitalWrite(RELAY_RIGHT_pin, LOW);
          digitalWrite(IR_POWER_pin, LOW);
          portal_start_position = PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
          portal_position = portal_start_position;
          portal_cmd_accepted = false;
          portal_force = 0;
          Serial.println("Already OPEN");
        }
      }
    }
    homeEasy.rxRelease();
  }

  if(true == portal_cmd_accepted) {
    _delay_ms(1);
    portal_position++;

    /* Compute force related to the position */
    portal_force = get_force(portal_start_position, portal_position);

    /* Compute maximum current authorized related to the position */
    if((PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT) > portal_position) {
      if(100 > portal_position) {
        motor_max_current = 1024;
      }
      else {
        motor_max_current = MOTOR_MAX_CURRENT;
      }
    }
    else {
      motor_max_current = MOTOR_MAX_SLOW_CURRENT;
    }
  }

  /* Check over current filtered on several acquisitions */
  int motor_currentRead = analogRead(MOTOR_SENSE_pin);
  if(motor_max_current < motor_currentRead) {
    motor_max_current_nb++;
  }
  else {
    motor_max_current_nb = 0;
  }

  /* Check over current */
  if(1000 < motor_max_current_nb) {
    motor_max_current_nb = 0;
    digitalWrite(MOTOR_PWM_pin, LOW);
    digitalWrite(RELAY_LEFT_pin, LOW);
    digitalWrite(RELAY_RIGHT_pin, LOW);
    digitalWrite(IR_POWER_pin, LOW);
    Serial.print("CURRENT MAX detected: "); Serial.println(motor_currentRead, DEC);
    portal_cmd_accepted = false;
    portal_force = 0;
  }
  /* Check Limiter at end of CLOSE */
  if(PORTAL_CMD_CLOSE == portal_cmd) {
    if(HIGH == digitalRead(LIMITER_RIGHT_pin)) {
      /* As the limiter triggered we are at the end of the rail */
      portal_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
      digitalWrite(MOTOR_PWM_pin, LOW);
      digitalWrite(RELAY_LEFT_pin, LOW);
      digitalWrite(RELAY_RIGHT_pin, LOW);
      digitalWrite(IR_POWER_pin, LOW);
      if(true == portal_cmd_accepted) { Serial.println("LIMITER detected at end of CLOSE"); }
      portal_cmd_accepted = false;
      portal_force = 0;
    }
  }
  /* Check Limiter at end of OPEN */
  if(PORTAL_CMD_OPEN == portal_cmd) {
    if(HIGH == digitalRead(LIMITER_LEFT_pin)) {
      /* As the limiter triggered we are at the end of the rail */
      portal_position = PORTAL_SLOW_SLOT + PORTAL_CRUISE_SLOT + PORTAL_SLOW_SLOT;
      digitalWrite(MOTOR_PWM_pin, LOW);
      digitalWrite(RELAY_LEFT_pin, LOW);
      digitalWrite(RELAY_RIGHT_pin, LOW);
      digitalWrite(IR_POWER_pin, LOW);
      if(true == portal_cmd_accepted) { Serial.println("LIMITER detected at end of OPEN"); }
      portal_cmd_accepted = false;
      portal_force = 0;
    }
  }
  /* Timeout */
  if(PORTAL_FULL_SLOT < portal_position) {
    portal_position = PORTAL_FULL_SLOT;
    digitalWrite(MOTOR_PWM_pin, LOW);
    digitalWrite(RELAY_LEFT_pin, LOW);
    digitalWrite(RELAY_RIGHT_pin, LOW);
    digitalWrite(IR_POWER_pin, LOW);
    if(true == portal_cmd_accepted) { Serial.print("TIMEOUT detected"); }
    portal_cmd_accepted = false;
    portal_force = 0;
  }

  /* Clone LED status on RF signal */
  //digitalWrite(MOTOR_PWM_pin, HIGH);
  analogWrite(MOTOR_PWM_pin, map(portal_force, 0, PORTAL_SLOW_SLOT, 0, 255));
  //Serial.println(portal_force, DEC);
  digitalWrite(LED_pin, digitalRead(RF_IN_pin));
  //Serial.println(analogRead(MOTOR_SENSE_pin), DEC);
}

