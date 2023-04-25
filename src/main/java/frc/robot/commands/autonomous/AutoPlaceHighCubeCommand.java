package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoPlaceHighCubeCommand extends CommandBase{
    
    private ArmSubsystem arm;
    private Timer timer;
    private boolean startedTimer = false;

    public AutoPlaceHighCubeCommand(
            ArmSubsystem arm
    ) {
        this.arm = arm;
        //addRequirements(drivetrain);
        timer = new Timer();
    }
    
    @Override
    public void execute() {
        if(startedTimer == false) {
            this.arm.moveHighCube();
            timer.start();
            startedTimer = true;
            System.out.println("AutoPlaceHighCubeCommand called");
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
        if(timer.hasElapsed(1)) {
            timer.stop();
            return true;
        }

        return false;
    }
}
