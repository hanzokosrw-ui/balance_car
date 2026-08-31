#ifndef __KALMAN_H
#define __KALMAN_H

typedef struct
{
    float angle;
    float bias;
    float rate;
    float p00;
    float p01;
    float p10;
    float p11;
} kalman_t;

void kalman_init(kalman_t *kalman, float angle);
float kalman_update(kalman_t *kalman, float new_angle, float new_rate, float dt);

/* ---- complementary filter ---- */
typedef struct
{
    float angle;
    float alpha;
} complementary_t;

void complementary_init(complementary_t *cf, float alpha, float angle);
float complementary_update(complementary_t *cf, float accel_angle, float gyro_rate, float dt);

#endif
