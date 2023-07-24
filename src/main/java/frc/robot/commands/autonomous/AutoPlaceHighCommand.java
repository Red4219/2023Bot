package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoPlaceHighCommand extends CommandBase{
    
    private ArmSubsystem arm;
    private Timer timer;
    private boolean startedTimer = false;

    public AutoPlaceHighCommand(
            ArmSubsystem arm
    ) {
        this.arm = arm;
        //addRequirements(drivetrain);
        timer = new Timer();
    }
    
    @Override
    public void execute() {
        if(startedTimer == false) {
            this.arm.moveHigh();
            timer.start();
            startedTimer = true;
        }
        this.arm.periodic();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        //System.out.println("AutoAutoAimCommand - done, targetX is: " + translationXSupplier.getAsDouble());
        //drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        /*if(timer.hasElapsed(5)) {
            timer.stop();
            return true;
        }*/

        return false;

        /*double targetX = translationXSupplier.getAsDouble();

        if(this.targetFoundSupplier.getAsBoolean() == true) {

            if(targetX >= Constants.VISION_REFLECTIVE_THRESHOLD_MIN && targetX <= Constants.VISION_REFLECTIVE_THRESHOLD_MAX) {
                System.out.println("Hit threshold isFinished() targetX: " + targetX);
                return true;
            }

            System.out.println("Target is found, threshold not met");
        } 
        
        return false;*/
    }
}
