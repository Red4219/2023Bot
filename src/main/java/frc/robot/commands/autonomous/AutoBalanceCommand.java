package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalanceCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    float pitchMoveDirection = 0.0f;
    float rolllMoveDirection = 0.0f;

    public AutoBalanceCommand(
            DrivetrainSubsystem drivetrain) {
                this.drivetrain = drivetrain;
                this.drivetrain.setDriveCommand(this);

                //addRequirements(drivetrain);
            }

    @Override
    public void execute() {

        System.out.println("AutoBalanceCommand::execure() called, pitch: " + drivetrain.getPitch() + ", roll: " + drivetrain.getRoll());

        if(drivetrain.getPitch() > (0 + Constants.PITCH_CHANGE_THRESHOLD)) {
            pitchMoveDirection = Constants.PITCH_CHANGE_AMOUNT;
        } else if(drivetrain.getPitch() < (0 - Constants.PITCH_CHANGE_THRESHOLD)) {
            pitchMoveDirection = -Constants.PITCH_CHANGE_AMOUNT;
        } else {
            pitchMoveDirection = 0;
        }

        if(drivetrain.getPitch() > (0 + Constants.ROLL_CHANGE_THRESHOLD)) {
            rolllMoveDirection = Constants.ROLL_CHANGE_AMOUNT;
        } else if(drivetrain.getPitch() < (0 - Constants.ROLL_CHANGE_THRESHOLD)) {
            rolllMoveDirection = -Constants.ROLL_CHANGE_AMOUNT;
        } else {
            rolllMoveDirection = 0;
        }

        drivetrain.drive(new ChassisSpeeds(
            pitchMoveDirection,
            rolllMoveDirection,
            0.0
        ));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        System.out.println("AutoPitchCommand - done, pitch is: " + drivetrain.getPitch() + ", roll is: " + drivetrain.getRoll());
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {

        /*if(drivetrain.getPitch() > (0 + Constants.PITCH_CHANGE_THRESHOLD) && drivetrain.getPitch() < (0 - Constants.PITCH_CHANGE_THRESHOLD))  {
            System.out.println("Hit threshold isFinished() pitch: " + drivetrain.getPitch());
            return true;
        }*/

        return false;
    }
}
