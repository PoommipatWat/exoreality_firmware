// Omni_kinematics.h

#ifndef OMNI_KINEMATICS_H
#define OMNI_KINEMATICS_H

class Omni_kinematics {
  public:
    Omni_kinematics(float Distance, float Radius, float initial_theta = 0);
    void inverse_omni(float vx, float vy, float w, float max_rpm, float &rpm1, float &rpm2, float &rpm3, bool filter = true);
    void forward_omni(float rpm1, float rpm2, float rpm3, float &vx, float &vy, float &w);
  
  private:
    float Distance;
    float initial_theta;
    float Radius;
};

#endif
