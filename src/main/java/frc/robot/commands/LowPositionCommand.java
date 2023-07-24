package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class LowPositionCommand extends CommandBase {

    ArmSubsystem armSubsystem;
    PIDController pidController = new PIDController(3, 0, 0.8);

    public LowPositionCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        pidController.setSetpoint(5.0);

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("LowPositionCommand started");
        pidController.reset();
    }

    @Override
    public void execute() {
        System.out.println("LowPositionCommand executed");
        double speed = pidController.calculate(5);
        // subsystem.setmotor
        armSubsystem.setHighMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("LowPositionCommand ended");
        armSubsystem.setHighMotor(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println("LowPositionCommand finished");
        return false;
        //return true;
    }
}
