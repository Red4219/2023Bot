package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoAutoAimCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private boolean fieldOriented = true;

    public AutoAutoAimCommand(
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

        //addRequirements(drivetrain);
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

        System.out.println("AutoAimCommand called");

        /*if(translationXSupplier.getAsDouble() < 0.0) {
            System.out.println("need to rotate to the left");
        } else if(translationYSupplier.getAsDouble() > 0.0) {
            System.out.println("need to rotate to the right");
        }*/
        
        /*double translationXPercent = translationXSupplier.getAsDouble();
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
        }*/

        

        drivetrain.drive(new ChassisSpeeds(
            0.0,
            0.0,
            -(translationXSupplier.getAsDouble() * .1)
        ));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        System.out.println("done");
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {

        double targetX = translationXSupplier.getAsDouble();
        
        if(targetX >= Constants.VISION_REFLECTIVE_THRESHOLD_MIN && targetX <= Constants.VISION_REFLECTIVE_THRESHOLD_MAX) {
            return true;
        }
        
        return false;
    }
}
