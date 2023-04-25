package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoEjectCubeCommand extends CommandBase {

    private ArmSubsystem arm;
    private Timer timer;
    private boolean hasSetSpeed = false;

    public AutoEjectCubeCommand(
            ArmSubsystem arm
    ) {
        this.arm = arm;
        //addRequirements(drivetrain);

        timer = new Timer();
    }

    @Override
    public void execute() {
        if(hasSetSpeed == false) {
            timer.start();
            this.arm.setAutoRunning(true);
            this.arm.ejectCube();
            System.out.println("AutoEjectCubeCommand called");
            hasSetSpeed = true;
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
            arm.stopIntake();
            arm.setAutoRunning(false);
            return true;
        }

        return false;
    }
}
