package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private boolean fieldOriented = true;

    public DriveCommand(
            DrivetrainSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        this.drivetrain.setDriveCommand(this);

        addRequirements(drivetrain);
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    public boolean getFieldOriented() {
        return this.fieldOriented;
    }

    public boolean toggleFieldOriented() {

        if(this.fieldOriented) {
            fieldOriented = false;
        } else {
            fieldOriented = true;
        }

        return this.fieldOriented;
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();

        if(fieldOriented) {
            // We are in field oriented
            drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                drivetrain.getRotation()
            )
        );
        } else {
            // We are not in field oriented
            drivetrain.drive(new ChassisSpeeds(
                translationXPercent,
                translationYPercent,
                rotationPercent
            ));
        }        
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
