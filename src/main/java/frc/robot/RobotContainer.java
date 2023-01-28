package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubSystem;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
    //private final LimeLightSubSystem limeLight = new LimeLightSubSystem();
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    SendableChooser<Command> chooser = new SendableChooser<>();
    

    private final XboxController controller = new XboxController(0);

    public RobotContainer() {
        drivetrain.register();
        //limeLight.register();

        drivetrain.setDefaultCommand(new DriveCommand(
                drivetrain,
                () -> -modifyAxis(controller.getLeftY()), // Axes are flipped here on purpose
                () -> -modifyAxis(controller.getLeftX()),
                () -> -modifyAxis(controller.getRightX())
        ));

        //new Button(controller::getBackButtonPressed)
                //.whenPressed(drivetrain::zeroGyroscope);

        configureAutoCommands();
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    /*public LimeLightSubSystem getLimeLightSubSystem() {
        return limeLight;
    }*/

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    public void configureAutoCommands() {
        //frc.robot.Constants.AUTO_EVENT_MAP.put("event 1", new PrintCommand("passed marker 1"));
        //frc.robot.Constants.AUTO_EVENT_MAP.put("event 2", new PrintCommand("passed marker 2"));

        //build auth paths
        /*List<PathPlannerTrajectory> auto1Paths = 
                PathPlanner.loadPathGroup("testPaths1",
                frc.robot.Constants.AUTO_MAX_SPEED_METERS_PER_SECOND,
                frc.robot.Constants.AUTO_MAX_SPEED_METERS_PER_SECOND_SQUARED);*/

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "FullAuto", 
            new PathConstraints(2.0, 2.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drivetrain::getPose,
            drivetrain::setPose,
            drivetrain.getKinimatics(), 
            new PIDConstants(5.0, 0.0, 0.0), 
            new PIDConstants(0.5, 0.0, 0.0), 
            drivetrain::setModuleStates, 
            eventMap, 
            false,
            drivetrain
        );
        
        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        /*Command autoTest = 
                new SequentialCommandGroup(
                    
                    //new FollowPathWithEvents(FollowPath, null, null)
                        //new FollowPathWithEvents(
                                //new FollowPath(auto1Paths.get(0), drivetrain, true),
                                    //auto1Paths.get(0).getMarkers(),
                                    //frc.robot.Constants.AUTO_EVENT_MAP),
                                //new InstantCommand(drivetrain::)
                                //new WaitCommand(5.0)
                    new WaitCommand(5.0)
                );*/

        //chooser.addOption("curvy path", loadPathPlannerTrajectoryToRameseteCommand("deploy/pathplanner/generatedJSON/curvy.wpilib.json", true));
        //chooser.addOption("test path",autoTest);
        chooser.addOption("test path",fullAuto);
        chooser.setDefaultOption("test path", fullAuto);
        Shuffleboard.getTab("Autonomous").add(chooser);
    }

    public void scheduleAutonomous() {
        //Command command = chooser.getSelected();
        chooser.getSelected().schedule();
    }
}
