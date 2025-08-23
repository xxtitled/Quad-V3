#include <stdio.h>   // 표준 입출력 (printf) 사용을 위한 헤더
#include <math.h>    // 수학 함수(sin, cos, atan, acos, isnan 등) 사용을 위한 헤더

// 액추에이터 감속비 (Reduction ratio)
// 계산된 라디안 값을 실제 모터 회전량으로 변환할 때 곱한다
const int rr = 9;

// 다리 마디 길이 및 힙 오프셋 (단위: mm)
// knee   : 허벅지(femur) 길이
// tibia  : 정강이(tibia) 길이
// yOffset: 힙 모터 위치와 회전축 사이 수평 거리
const float knee    = 170.0f;
const float tibia   = 170.0f;
const float yOffset =  85.0f;

// 각 다리의 상태를 저장하는 구조체
typedef struct Leg {
    int   id;      // 다리 번호 (0~3)
    float theta;   // hip pitch (앞뒤 기울기) [rad * rr]
    float phi;     // knee flex (무릎 굽힘)   [rad * rr]
    float gamma;   // hip yaw   (좌우 회전)   [rad * rr]
} Leg;

// 제곱 연산 헬퍼 함수: 가독성을 위해 x*x 대신 사용
float sq(float numb) {
    return numb * numb;
}

//----------------------------------------------------------------------
// z(): 무릎 각도(phi) 계산 및 hip pitch(theta) 보정
//  - leg: 결과를 저장할 Leg 구조체 포인터
//  - height: 이전 단계에서 계산된 다리 높이 (mm)
//----------------------------------------------------------------------
void z(Leg *leg, float height) {
    // 코사인 법칙으로 θ 보정값 계산
    //   θZ = (90° - acos((knee² + height² - tibia²)/(2·knee·height))) * rr
    float thetaZ = ((3.1416f/2.0f)
                   - acosf((sq(knee) + sq(height) - sq(tibia))
                           / (2.0f * knee * height)))
                   * rr;

    // 코사인 법칙으로 무릎 굽힘 각 φ 계산
    //   φZ = acos((knee² + tibia² - height²)/(2·knee·tibia)) * rr
    float phiZ = acosf((sq(knee) + sq(tibia) - sq(height))
                       / (2.0f * knee * tibia))
                 * rr;

    // hip pitch 최종값: 이전 theta에서 보정량을 빼 줌
    float *theta = &leg->theta;
    *theta -= thetaZ;

    // 무릎 굽힘 각도 저장 (부호 반전)
    float *phi = &leg->phi;
    *phi = -phiZ;
}

//----------------------------------------------------------------------
// x(): hip pitch(theta) 초기값 계산 및 다리 길이 보정
//  - leg: 결과를 저장할 Leg 구조체 포인터
//  - height: y()에서 리턴된 다리 높이 (mm)
//  - distX: 발끝의 앞뒤 offset 거리 (mm)
//  Returns: 보정된 다리 길이 (mm)
//----------------------------------------------------------------------
float x(Leg *leg, float height, float distX) {
    // 앞뒤 offset이 만드는 추가 각도 (rad)
    float extraTheta = atan2f(distX, height);

    // 감속비(rr)를 곱해 실제 모터 회전량으로 변환
    float thetaX = extraTheta * rr;

    // 삼각형 빗변 길이 = height / cos(extraTheta)
    float newLegLength = height / cosf(extraTheta);

    // hip pitch 초기값 저장
    float *theta = &leg->theta;
    *theta = thetaX;

    return newLegLength;
}

//----------------------------------------------------------------------
// y(): hip yaw(gamma) 계산 및 1단계 다리 길이 보정
//  - leg: 결과를 저장할 Leg 구조체 포인터
//  - height: 초기 높이 pos_z (mm)
//  - posY: 발끝의 좌우 offset 거리 (mm)
//  Returns: hip→knee까지 보정된 다리 길이 (mm)
//----------------------------------------------------------------------
float y(Leg *leg, float height, float posY) {
    // hip 중심선에서 발끝까지 수평 거리 계산
    float distY = yOffset + posY;

    // 직각삼각형에서의 각도 γP = atan(distY / height)
    float gammaP = atan2f(distY, height);
    if (isnan(gammaP)) {
        gammaP = 3.1416f / 2.0f;  // height==0 방어 처리
    }

    // 삼각형 빗변 길이 계산
    float hipHyp = distY / sinf(gammaP);

    // hip 오프셋(yOffset)에 의한 보정 각도 λ = asin(yOffset / hipHyp)
    float lambda = asinf(yOffset / hipHyp);

    // 최종 yaw 각도 γ = (γP - λ) * rr
    float gammaY = (gammaP - lambda) * rr;

    // hip yaw 저장
    float *gamma = &leg->gamma;
    *gamma = gammaY;

    // hip→knee까지 보정된 길이를 리턴
    float newNewLegLength = yOffset / tanf(lambda);
    return newNewLegLength;
}

//----------------------------------------------------------------------
// inverseKinematics(): 3단계 호출로 θ, φ, γ를 한 번에 계산
//  - leg: 결과를 저장할 Leg 구조체 포인터
//  - pos_z: 발끝 높이 (mm), pos_x: 앞뒤 offset (mm), pos_y: 좌우 offset (mm)
//----------------------------------------------------------------------
void inverseKinematics(Leg *leg, float pos_z, float pos_x, float pos_y) {
//int *id = &(*leg).id;
//printf("ID: %d \n", (*leg).id);
//*id = 2;
  z(leg, x(leg, y(leg, pos_z, pos_y), pos_x));
}

int main() {
    // 4개의 다리 구조체 생성 및 ID 초기화
    Leg legs[] = {
        {.id = 0},
        {.id = 1},
        {.id = 2},
        {.id = 3}
    };
    int leg_amount = sizeof(legs) / sizeof(legs[0]);

    // 초기 상태 출력 (아직 각도 계산 전)
    printf("Amount of legs: %d\n", leg_amount);
    for (int i = 0; i < leg_amount; i++) {
        printf("ID: %d, Theta: %f, Phi: %f, Gamma: %f\n",
               legs[i].id,
               legs[i].theta,
               legs[i].phi,
               legs[i].gamma);
    }

    // 서로 다른 목표 좌표로 역기구학 계산 호출
    inverseKinematics(&legs[0], 10.0f, 10.0f,  5.0f);
    inverseKinematics(&legs[1], 10.0f, 20.0f,  5.0f);
    inverseKinematics(&legs[2], 10.0f,  2.0f, 15.0f);
    inverseKinematics(&legs[3], 10.0f,  0.0f,  0.0f);

    // 계산된 각도 결과 출력
    printf("\nIK 계산 결과:\n");
    for (int i = 0; i < leg_amount; i++) {
        printf("ID: %d, Theta: %f, Phi: %f, Gamma: %f\n",
               legs[i].id,
               legs[i].theta,
               legs[i].phi,
               legs[i].gamma);
    }

    return 0;
}

