// Open loop motor control example
#include <SimpleFOC.h>

// BLDC motor & driver 인스턴스
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// 속도 타겟 (rad/s)
float target_velocity = 0;

// 시리얼 명령어 처리기
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd)  { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  Serial.begin(9600);
  SimpleFOCDebug::enable(&Serial);

  // 드라이버 설정
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  // 모터 전압 리밋 (안전용)
  motor.voltage_limit = 3;   // [V]

  // 개방루프 속도 제어
  motor.controller = MotionControlType::velocity_openloop;
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }

  // 명령어 등록
  // T: 속도 설정 (양수 → 시계 방향, 음수 → 반시계 방향)
  command.add('T', doTarget, "target velocity (rad/s, +CW, -CCW)");
  // L: 전압 리밋 설정
  command.add('L', doLimit,  "voltage limit (V)");

  Serial.println("Motor ready!");
  Serial.println("Commands:");
  Serial.println("  T<값>  : 속도 설정 (rad/s, +CW, -CCW)");
  Serial.println("  L<값>  : 전압 리밋 설정");
  delay(500);
}

void loop() {
  // target_velocity 값의 부호에 따라 CW/CCW 자동 전환
  motor.move(target_velocity);
  command.run();
}
