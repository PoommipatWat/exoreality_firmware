// Omni_kinematics.cpp

#include "omni_kinematics.h"
#include <math.h>

Omni_kinematics::Omni_kinematics(float Distance, float Radius, float initial_theta) {
    this->Distance = Distance;
    this->initial_theta = initial_theta;
    this->Radius = Radius;
}

void Omni_kinematics::inverse_omni(float vx, float vy, float w, float max_rpm, float &rpm1, float &rpm2, float &rpm3, bool filter) {
    float instance[3][3] = {
        {-sin(this->initial_theta), cos(this->initial_theta), this->Distance},
        {-sin(M_PI/3 - this->initial_theta), -cos(M_PI/3 - this->initial_theta), this->Distance},
        {sin(M_PI/3 + this->initial_theta), -cos(M_PI/3 + this->initial_theta), this->Distance}
    };
    
    float v[3] = {vx, vy, w};
    float wheel_velocities[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            wheel_velocities[i] += instance[i][j] * v[j];
        }
    }

    float v1 = wheel_velocities[0];
    float v2 = wheel_velocities[1];
    float v3 = wheel_velocities[2];

    float omega1 = v1 / this->Radius;
    float omega2 = v2 / this->Radius;
    float omega3 = v3 / this->Radius;

    rpm1 = omega1 * 60 / (2 * M_PI);
    rpm2 = omega2 * 60 / (2 * M_PI);
    rpm3 = omega3 * 60 / (2 * M_PI);

    if (filter) {
        float max_rpm_abs = fmax(fmax(fabs(rpm1), fabs(rpm2)), fabs(rpm3));
        if (max_rpm_abs > max_rpm) {
            float scale_factor = max_rpm / max_rpm_abs;
            rpm1 *= scale_factor;
            rpm2 *= scale_factor;
            rpm3 *= scale_factor;
        }
    }
}

void Omni_kinematics::forward_omni(float rpm1, float rpm2, float rpm3, float &vx, float &vy, float &w) {
    float omega1 = rpm1 * 2 * M_PI / 60;
    float omega2 = rpm2 * 2 * M_PI / 60;
    float omega3 = rpm3 * 2 * M_PI / 60;

    float v1 = omega1 * this->Radius;
    float v2 = omega2 * this->Radius;
    float v3 = omega3 * this->Radius;

    float instance[3][3] = {
        {-sin(this->initial_theta), cos(this->initial_theta), this->Distance},
        {-sin(M_PI/3 - this->initial_theta), -cos(M_PI/3 - this->initial_theta), this->Distance},
        {sin(M_PI/3 + this->initial_theta), -cos(M_PI/3 + this->initial_theta), this->Distance}
    };

    float inv_instance[3][3];
    // Inverting the matrix manually (assuming the matrix is invertible)
    // This is a simplified method, in practice, use a library for matrix inversion if available
    float det = instance[0][0]*(instance[1][1]*instance[2][2] - instance[2][1]*instance[1][2]) -
                instance[0][1]*(instance[1][0]*instance[2][2] - instance[1][2]*instance[2][0]) +
                instance[0][2]*(instance[1][0]*instance[2][1] - instance[1][1]*instance[2][0]);

    float inv_det = 1.0 / det;

    inv_instance[0][0] = (instance[1][1]*instance[2][2] - instance[2][1]*instance[1][2]) * inv_det;
    inv_instance[0][1] = (instance[0][2]*instance[2][1] - instance[0][1]*instance[2][2]) * inv_det;
    inv_instance[0][2] = (instance[0][1]*instance[1][2] - instance[0][2]*instance[1][1]) * inv_det;
    inv_instance[1][0] = (instance[1][2]*instance[2][0] - instance[1][0]*instance[2][2]) * inv_det;
    inv_instance[1][1] = (instance[0][0]*instance[2][2] - instance[0][2]*instance[2][0]) * inv_det;
    inv_instance[1][2] = (instance[1][0]*instance[0][2] - instance[0][0]*instance[1][2]) * inv_det;
    inv_instance[2][0] = (instance[1][0]*instance[2][1] - instance[2][0]*instance[1][1]) * inv_det;
    inv_instance[2][1] = (instance[2][0]*instance[0][1] - instance[0][0]*instance[2][1]) * inv_det;
    inv_instance[2][2] = (instance[0][0]*instance[1][1] - instance[1][0]*instance[0][1]) * inv_det;

    float wheel_velocities[3] = {v1, v2, v3};
    float result[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i] += inv_instance[i][j] * wheel_velocities[j];
        }
    }

    vx = result[0];
    vy = result[1];
    w = result[2];
}
