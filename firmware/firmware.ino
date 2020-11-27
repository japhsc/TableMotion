#include <Arduino.h>
#include "myserial.h"

#include "utilities.h"
#include "definitions.h"

/** ############# STATE ############# **/

uint8_t led_state=0;

bool update_required=false, update_ready=false;
unsigned long update_ts=0;

// state variables
long steps[2]={0,0};

long positions[4]={POS0,POS1,POS2,POS3};
uint8_t index=0;

long* target = positions+index;

// Save current state (steps)
void save_state() {
  if (update_required && update_ready && !go_running() ) {
    if (!update_ts) {
      update_ts = millis();
    } else if (millis()-update_ts > 500) {
      save_steps();
      
      print("Update steps \t");
      print(steps[0]);
      print("/");
      println(steps[1]);
      
      update_required = false;
      update_ready = false;
      update_ts = 0;
    }
  }
}

// Load last steps from persisten eeprom
void load_steps() {
  bool ret;
  for (int j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
    ret = load_eeprom(ADDR_STEPS[j], &steps[j]);
    
    print("Setting: steps M");
    print(j);
    print(" (");
    print(ret);
    print(")\t");
    println(steps[j]);
  }
}

void save_steps() {
  save_eeprom(ADDR_STEPS[0], &steps[0]);
  save_eeprom(ADDR_STEPS[1], &steps[1]);
}

void load_targets() {
  bool ret;
  for (byte i=0; i<4; ++i) {
    ret = load_eeprom(ADDR_TARGETS+i*sizeof(long), positions+i);
    
    if (i==0) println("Setting: positions");
    print("\t\tP");
    print(i);
    print(" (");
    print(ret);
    print(")\t");
    println(*(positions+i));
  }
}

void reset_targets() {
  for (byte i=0; i<4; ++i) {
    reset_eeprom(ADDR_TARGETS+i*sizeof(long), (long) 0);
  }
  // Blink Reset
  blinkn(PIN_IND[1], 4, 100);
}

void reset_steps() {
  reset_eeprom(ADDR_STEPS[0], (long) 0);
  reset_eeprom(ADDR_STEPS[1], (long) 0);
  // Blink Reset
  blinkn(PIN_IND[0], 4, 100);
}

void save_target(const uint8_t idx) {
  save_eeprom(ADDR_TARGETS+idx*sizeof(long), positions+idx);
}

void load_index() {
  bool ret = load_eeprom(ADDR_INDEX, &index);
  
  print("Setting: index (");
  print(ret);
  print(") ");
  println(index);
}

void save_index() {
  save_eeprom(ADDR_INDEX, &index);
}

void next_target() {
  index = (index+1)%4;
  target = positions + index;
  save_index();

  // show selection
  led_state = 3;

  print("Select target storage P");
  print(index);
  print(": ");
  println(*target);
}

void set_target() {
  *target = steps[0]; //use motor A as reference

  save_target(index);

  print("Save current position ");
  print(steps[0]);
  print(" to target storage P");
  println(index);
}

/** ############# MOTOR ############# **/

bool val_hall[2][2] = {{0,0},{0,0}};

void sense_hall() {
  uint8_t ret;
  long temp_step;
  for (int j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
    temp_step = steps[j];
    ret = sample_hall(HALL_SENSOR[0][j], 
                      HALL_SENSOR[1][j], 
                      val_hall[0][j], 
                      val_hall[1][j], 
                      steps[j]);
    if (ret == 2) {
      // motor full step
      step_action(j, steps[j]-temp_step);
    } else if (ret == 1) {
      // motor half step
    }
  }
}

void step_action(const uint8_t M, int diff){
  print("Step M");
  print(M);
  print("\t");
  print(diff);
  print("\t");
  println(steps[M]);

  if ( (steps[M]<POS_LOWER && diff<0) || (steps[M]>POS_UPPER && diff>0) ) {
    print("Stop! Limit reached! ");
    print(POS_LOWER);
    print("/");
    println(POS_UPPER);

    go_stop(M);
  } 
}

void motor_speed(const uint8_t M, const bool fwd, const int pwm){
  analogWrite(MOTOR_PWM[0][M], (!fwd)*pwm);
  analogWrite(MOTOR_PWM[1][M], fwd*pwm);
}

void motor_state( const uint8_t M, const bool state, 
                  const bool fwd=true, const uint8_t pwm=0){
  motor_speed(M, fwd, state*pwm);
  digitalWrite(MOTOR_EN[M], state);
  //delay(10);
  update_ready = !state;
}


long last_steps[2];
bool next_step[2]={false, false};

bool new_steps() {
  bool upd;
  for (uint8_t j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
    if (abs(last_steps[j]-steps[j])>0) {
      next_step[j] = true;
      last_steps[j] = steps[j];
      
      upd=true;
      for (uint8_t jj=0; jj<num_motor; ++jj) {
        upd &= next_step[jj];
      }
      if (upd) {
        next_step[0] = false;
        next_step[1] = false;
        return true;
      }
    }
  }
  return false;
}

void scale_speed(unsigned long d, byte& speed) {
  if (new_steps()) {
    if (d>20) {
      if (speed < PWM_UPPER) speed += PWM_INC;
    } else {
      if (speed >  PWM_LOWER) speed -= PWM_DEC;
    }
    /*
    print("PWM speed ");
    println(speed);
    */
  }
}

unsigned long go_ts[2];
bool go[2] = {false, false};
byte go_speed;

void go_start(){
  print("Go: move to target P");
  print(index);
  print(": ");
  println(*target);

  go_speed = PWM_LOWER;
  for (uint8_t j=0; j<num_motor; ++j)
    go[j] = true;

  // show motor steps
  led_state = 4;
}

void go_abort(){
  for (uint8_t j=0; j<num_motor; ++j)
    go_stop(j);

  println("Go: abort!");
}

bool go_running() {
  return (go[0] || go[1]);
}

void go_stop(const uint8_t M) {
  motor_state(M,false);
  go[M] = false;
  update_required = true;

  // show selection
  led_state = 3;
}


long diff, dist;

// integrate second motor
void go_motor() {
  for (uint8_t j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
    if (go[j]) {
      diff = *target - steps[j];
      dist = abs(diff);
      if (dist>0) {
        if ((millis()-go_ts[j])>=10) {
          if (diff>0)
            motor_state(j, true, false, go_speed);
          else if (diff<0)
            motor_state(j, true, true, go_speed);
          go_ts[j] = millis();
        }
        scale_speed(dist, go_speed);
      } else {
        go_stop(j);

        print("Go: M");
        print(j);
        println(" reached target!");
      }
    }
  }
}

void cruise_motor(bool state, bool forward) {
  for (int j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
    motor_state(j, state, forward, PWM_CRUISE);
  }
}

int val_IS[2][2];

void sense_current(){
  for (int i=0; i<2; ++i) { // Direction  0:left, 1:right
    for (int j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
      /* digitalRead causes problems with pwm
         5: OC0A/TIMER0B, 6: OC0B/TIMER0A
         OC0A/OC0B set pwm duty
      */
      if (read_pwm_state(MOTOR_PWM[i][j])) {
        val_IS[i][j] = analogRead(MOTOR_IS[i][j]);

        // Warning: out of specs
        if (val_IS[i][j]>IS_OC_WARN) {

          #if SERIAL_DEBUG
          println("Over current!");
          double U = double(val_IS[i][j])/1023. * 1.1;
          double I = U*8.5;
          print("IS (PWM) M");
          print(j);
          print(" D");
          print(i);
          print(":\t I=");
          print(I);
          print("A\t(");
          print(val_IS[i][j]);
          print(", ");
          print(U);
          println("V)");
          #endif

          // Error: overcurrent
          if (val_IS[i][j]>IS_OC) {
            go_abort();
          }
          return;
        }
      }
    }
  }
}


/** ############# HID ############# **/

/*               LED                 */
unsigned long led_ts;

unsigned long b_ts;
bool bbs=false;

void update_led() {
  if ((millis()-led_ts)>=10) {
    if (!led_state) {
      // do nothing
    } else if (led_state==1) { // off
      for (byte i=0; i<4; ++i)
        digitalWrite(PIN_IND[i], LOW);
      led_state = 0;
    } else if (led_state==2) { // blink
      if ((millis()-b_ts)>=500) {
        bbs = !bbs;
        digitalWrite(PIN_IND_1, bbs);
        b_ts = millis();
      }
    } else if (led_state==3) { // indicate selection
      for (byte i=0; i<4; ++i)
        digitalWrite(PIN_IND[i], i==index);
      led_state = 0;
    } else if (led_state==4) { // indicate motor moving
      digitalWrite(PIN_IND_1, val_hall[0][0]);
      digitalWrite(PIN_IND_2, val_hall[1][0]);
      digitalWrite(PIN_IND_3, val_hall[0][1]);
      digitalWrite(PIN_IND_4, val_hall[1][1]);
      //led_state = 0;
    }
    led_ts = millis();
  }
}

/*               BUTTON              */

bool var_btn[4]={true, true, true, true};
unsigned long ts_btn[4];

uint8_t action[4]={0,0,0,0};

void sample_button_1() {
  sample_action(PIN_BUTTON[0], var_btn[0], ts_btn[0], action[0]);
  if (action[0] == 2) {
    action[0] = 0;
    set_target();

    // blink to indicate action and show selection
    blinkn(PIN_IND_1, 4, 100);
    led_state=3;
  } else if (action[0] == 1) {
    action[0] = 0;

    if (go_running())
      go_abort();
    next_target();
  }
}

void sample_button_2() {
  sample_action(PIN_BUTTON[1], var_btn[1], ts_btn[1], action[1]);
  if (action[1] == 2) {
    action[1] = 0;
    led_state = 2;
  } else if (action[1] == 1) {
    action[1] = 0;
    led_state = 1;
  }
}

// motor off -> level steps min(reverse)/max(forward)
void level_steps(bool do_level, bool forward) {
  if (do_level) {
    *target = steps[0];
    for (uint8_t j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
      if (forward)
	    *target = max(*target, steps[j]);
	  else
	    *target = min(*target, steps[j]);
    }
    go_start();
  }
}

void sample_button_3() {
  if (read_digital(PIN_BUTTON[2], var_btn[2])){
    if (var_btn[3] && !go_running()) {
      cruise_motor(!var_btn[2], true);
      level_steps(var_btn[2], true);
      //update_required = var_btn[2];
    }
  }
}
  
void sample_button_4() {
  if (read_digital(PIN_BUTTON[3], var_btn[3])){
    if (var_btn[2] && !go_running()) {
      cruise_motor(!var_btn[3], false);
      level_steps(var_btn[3], false);
      //update_required = var_btn[3];
    }
  }
}

void sample_button_5() {
  sample_action(PIN_BUTTON[0], var_btn[0], ts_btn[0], action[0]);
  if (action[0]) {
    if (action[0] == 2) {
      *target -= 10;
    } else if (action[0] == 1) {
      *target += 10;
    }
    save_target(index);
    
    action[0] = 0;
    
    print("Target ");
    println(*target);
  }
}
  
void sample_button_6() {
  sample_action(PIN_BUTTON[1], var_btn[1], ts_btn[1], action[1]);
  if (action[1]) {
    action[1] = 0;
    if (var_btn[2] && var_btn[3]) {
      if (go_running()) go_abort();
      else go_start();
    }
  }
}


/** ############# INIT ############# **/

void setup() {
  // Setup serial debug
  init_serial();
  println("Boot");
  
  // Setup GPIO pins
  for (byte i=0; i<4; ++i) {
    pinMode(PIN_IND[i], OUTPUT);
    pinMode(PIN_BUTTON[i], INPUT);
  }
  
  for (int i=0; i<2; ++i) { // Direction  0:left, 1:right
    for (int j=0; j<num_motor; ++j) { // Motor 0:A, 1:B
      pinMode(MOTOR_IS[i][j], INPUT);
      pinMode(MOTOR_PWM[i][j], OUTPUT);
      pinMode(HALL_SENSOR[i][j], INPUT);
    
      digitalWrite(MOTOR_PWM[i][j], LOW);
    }
    pinMode(MOTOR_EN[i], OUTPUT);
    digitalWrite(MOTOR_EN[i], LOW);
  }
  analogReference(INTERNAL); //DEFAULT

  bool reset[2] = { (digitalRead(PIN_BUTTON[0]) == LOW), 
                    (digitalRead(PIN_BUTTON[1]) == LOW)};

  if (reset[0]) reset_steps();
  if (reset[1]) reset_targets();

  // Load last state
  if (!(reset[0] || reset[1])) load_index();
  load_steps();
  load_targets();

  // Blink OK
  if (!(reset[0] || reset[1])) blinkn(PIN_IND_1, 4, 200);

  // Show selection
  led_state=3;
}


/** ############# LOOP ############# **/

void loop() {
  // read hall sensors
  sense_hall();

  // move to position?
  go_motor();

  // overcurrent shutdown
  sense_current();

  // parse HID
  /*
  // blink led
  sample_button_2();
  
  // inc. (short) dec. target
  sample_button_5();
  */
  
  // next target (short) / set target (long)
  sample_button_1();

  // move up
  sample_button_3();

  // move down
  sample_button_4();

  // move: go/stop
  sample_button_6();
  

  // update indicator leds
  update_led();

  // save current state (power loss)?
  save_state();
}
