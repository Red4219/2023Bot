package frc.robot;

import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HighPositionCommand;
import frc.robot.commands.autonomous.AutoAutoAimCommand;
import frc.robot.commands.autonomous.AutoBalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
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
import com.revrobotics.REVPhysicsSim;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController controller = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private final LimeLightSubSystem limeLight = new LimeLightSubSystem();
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem(operatorController);
    SendableChooser<Command> chooser = new SendableChooser<>();
    private DriveCommand driveCommand; 
    private AutoBalanceCommand autoPitchCommand;

    // Simulation Stuff
    private REVPhysicsSim revPhysicsSim;

    public RobotContainer() {
        //drivetrain.register();
        //limeLight.register();

        drivetrain.setLimeLight(limeLight);

        driveCommand = new DriveCommand(
            drivetrain,
            () -> -modifyAxis(controller.getLeftY()), // Axes are flipped here on purpose
            () -> -modifyAxis(controller.getLeftX()),
            () -> -modifyAxis(controller.getRightX())
        );

        drivetrain.setDefaultCommand(driveCommand);

        drivetrain.setDriverController(controller);

        autoPitchCommand = new AutoBalanceCommand(drivetrain);

        configureAutoCommands();
        configureButtonBindings();
    } 

    public void configureButtonBindings() {
        //new JoystickButton(operatorController, Constants.OPERATOR_BUTTON_HIGH).whileTrue(new HighPositionCommand(armSubsystem));

        // High Motor
        //new JoystickButton(operatorController, Constants.OPERATOR_BUTTON_HIGH).onTrue(new HighPositionCommand(armSubsystem));
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public LimeLightSubSystem getLimeLightSubSystem() {
        return limeLight;
    }

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
        
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "TestPath1", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("event1", new PrintCommand("Passed marker 1"));
        /*eventMap.put("event1", new AutoAutoAimCommand(
            drivetrain, 
            () -> this.limeLight.getTargetX(), 
            () -> this.limeLight.getTargetY(), 
            () -> this.limeLight.getTargetRotation(),
            () -> this.limeLight.canSeeTarget()
        ));*/

        eventMap.put("event2", new AutoAutoAimCommand(
            drivetrain, 
            () -> this.limeLight.getTargetX(), 
            () -> this.limeLight.getTargetY(), 
            () -> this.limeLight.getTargetRotation(),
            () -> this.limeLight.canSeeTarget()
        ));


        //eventMap.put("event2", new HighPositionCommand(armSubsystem));
        eventMap.put("balance", new AutoBalanceCommand(drivetrain));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drivetrain::getPose,
            drivetrain::setPose,
            //drivetrain.getKinimatics(), 
            new PIDConstants(0.0, 0.0, 0.5), 
            new PIDConstants(0.0, 0.5, 0.0),
            drivetrain::drive, 
            //drivetrain::setModuleStates,
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

    public void setRevPhysicsSim(REVPhysicsSim sim) {
        this.revPhysicsSim = sim;
        this.armSubsystem.setRevPhysicsSim(sim);
    }
}
