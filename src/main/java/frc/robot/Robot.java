package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends LoggedRobot {
    private final RobotContainer container = new RobotContainer();
    private boolean simulation = false;
    private REVPhysicsSim simulator;

    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        /*if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the
                                                          // user)
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); 
            // Save outputs to a new log
        }*/

        
        Logger.getInstance().addDataReceiver(new NT4Publisher());

        // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic
        // Timestamps" in the "Understanding Data Flow" page
        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may
                                      // be added.

        //new RobotContainer();
    }

    @Override
    public void teleopInit() {
        // System.out.println("teleopInit() called");
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
        // System.out.println("autonomousInit() called");
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

        // CANSparkMax frontLeftMotor = new
        // CANSparkMax(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, MotorType.kBrushless);
        // REVPhysicsSim.getInstance().addSparkMax(frontLeftMotor, DCMotor.getNEO(1));

        // RevPhysicsSim.getInstance().addSparkMax(sparkMax, DCMotor.getNEO(1));
        // REVPhysicsSim.getInstance().addSparkMax(
        // new CANSparkMax(Constants.ARM_BASE_MOTOR_ID,
        // com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless),
        // DCMotor.getNEO(1)
        // );

    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {

    }
}
