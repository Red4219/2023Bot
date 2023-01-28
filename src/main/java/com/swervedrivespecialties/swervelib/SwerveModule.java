package com.swervedrivespecialties.swervelib;

import com.revrobotics.REVLibError;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    double getPositionConversionFactor();

    int getCountsPerRevolution();

    boolean getInverted();

    double getPosition();

    REVLibError setVelocityConversionFactor(double factor);
}
