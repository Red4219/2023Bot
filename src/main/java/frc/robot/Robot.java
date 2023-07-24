package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.photonvision.PhotonCamera;
import com.revrobotics.REVPhysicsSim;

//
// Now that Robot is running, create an instance of RobotContainer which starts everything
//

public class Robot extends TimedRobot {
    private final RobotContainer container = new RobotContainer();
    private boolean simulation = false;
    private REVPhysicsSim simulator;

    PhotonCamera camera = new PhotonCamera("photonvision");

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
