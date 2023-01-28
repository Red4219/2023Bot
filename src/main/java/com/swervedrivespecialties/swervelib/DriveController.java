package com.swervedrivespecialties.swervelib;

import com.revrobotics.REVLibError;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getPositionConversionFactor();

    int getCountsPerRevolution();

    boolean getInverted();

    double getPosition();

    REVLibError setVelocityConversionFactor(double factor);
}
