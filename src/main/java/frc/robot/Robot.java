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
    private boolean simulation = false;
    private REVPhysicsSim simulator;

    @Override
    public void teleopInit() {
        //System.out.println("teleopInit() called");
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
        //System.out.println("autonomousInit() called");
        container.scheduleAutonomous();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void simulationInit() {

        System.out.println("simulationInit() called");

        simulation = true;

        simulator = REVPhysicsSim.getInstance();

        container.setRevPhysicsSim(simulator);

        //CANSparkMax frontLeftMotor = new CANSparkMax(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
        //REVPhysicsSim.getInstance().addSparkMax(frontLeftMotor, DCMotor.getNEO(1));

        //RevPhysicsSim.getInstance().addSparkMax(sparkMax, DCMotor.getNEO(1));
        //REVPhysicsSim.getInstance().addSparkMax(
            //new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless),
            //DCMotor.getNEO(1)
        //);
        
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        
    }
}
