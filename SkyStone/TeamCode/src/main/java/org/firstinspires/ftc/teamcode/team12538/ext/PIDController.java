package org.firstinspires.ftc.teamcode.team12538.ext;

public interface PIDController {
    double getKP();
    double getKI();
    double getKD();

    void setKP(double m_K);
    void setKI(double m_I);
    void setKD(double m_D);

}
