package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private Constants.IntakeDirection direction;

    public IntakeCommand(ArmSubsystem armSubsystem, Constants.IntakeDirection direction) {
        this.armSubsystem = armSubsystem;
        this.direction = direction;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeCommand started");
    }

    @Override
    public void execute() {
        System.out.println("IntakeCommand executed");
        // subsystem.setmotor
        
        if(this.direction == Constants.IntakeDirection.IN) {
            armSubsystem.setBaseMotor(.5);
        } else {
            armSubsystem.setBaseMotor(-.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IntakeCommand ended");
        armSubsystem.setHighMotor(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println("IntakeCommand finished");
        return false;
        //return true;
    }
}
