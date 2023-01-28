package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
    private final RobotContainer container = new RobotContainer();

    @Override
    public void teleopInit() {
        container.getDrivetrain().resetPose();
    }

    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        container.scheduleAutonomous();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void simulationInit() {
        //CANSparkMax frontLeftMotor = new CANSparkMax(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
        //REVPhysicsSim.getInstance().addSparkMax(frontLeftMotor, DCMotor.getNEO(1));
    }

    @Override
    public void simulationPeriodic() {
        
    }

    @Override
    public void disabledPeriodic() {
        
    }
}
