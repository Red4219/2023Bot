package frc.robot;

import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.HighPositionCommand;
import frc.robot.commands.autonomous.AutoAutoAimCommand;
import frc.robot.commands.autonomous.AutoBalanceCommand;
import frc.robot.commands.autonomous.AutoEjectConeCommand;
import frc.robot.commands.autonomous.AutoEjectCubeCommand;
import frc.robot.commands.autonomous.AutoFinishedIntakeCubeCommand;
import frc.robot.commands.autonomous.AutoFoldCommand;
import frc.robot.commands.autonomous.AutoIntakeCubeCommand;
import frc.robot.commands.autonomous.AutoPlaceHighCommand;
import frc.robot.commands.autonomous.AutoPlaceHighCubeCommand;
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
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;


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
        
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public LimeLightSubSystem getLimeLightSubSystem() {
        return limeLight;
    }

    //
    // This removes the "noise" that comes from the controllers
    //
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
        Command blueLeftPlaceTaxiCommand = loadBlueLeftPlaceTaxi();
        
        chooser.addOption("BlueCenterPlaceBalance", loadBlueCenterPlaceBalance());
        chooser.addOption("BlueCenterPlaceBalanceCube", loadBlueCenterPlaceBalanceCube());
        chooser.addOption("BlueLeftPlaceTaxi", blueLeftPlaceTaxiCommand);
        chooser.addOption("BlueRightPlaceTaxi", loadBlueRightPlaceTaxi());
        chooser.addOption("RedCenterPlaceBalance", loadRedCenterPlaceBalance());
        chooser.addOption("RedLeftPlaceTaxi", loadRedLeftPlaceTaxi());
        chooser.addOption("RedRightPlaceTaxi", loadRedRightPlaceTaxi());
        chooser.addOption("BlueDoubleCube", loadBlueDoubleCube());
        chooser.addOption("RedDoubleCube", loadRedDoubleCube());


        chooser.setDefaultOption("BlueLeftPlaceTaxi", blueLeftPlaceTaxiCommand);

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

    //
    // Below are just the auto routines 
    //

    //
    // Blue Auto Cone Routines
    //

    public Command loadBlueCenterPlaceBalance() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "BlueCenterPlaceBalance", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();

        
        eventMap.put("placeHigh", new AutoPlaceHighCommand(armSubsystem));
        eventMap.put("eject", new AutoEjectConeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
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

        return fullAuto;
    }

    public Command loadBlueLeftPlaceTaxi() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "BlueLeftPlaceTaxi", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();

        
        eventMap.put("placeHigh", new AutoPlaceHighCommand(armSubsystem));
        eventMap.put("eject", new AutoEjectConeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
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

        return fullAuto;
    }

    public Command loadBlueRightPlaceTaxi() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "BlueRightPlaceTaxi", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();

        
        eventMap.put("placeHigh", new AutoPlaceHighCommand(armSubsystem));
        eventMap.put("eject", new AutoEjectConeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
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

        return fullAuto;
    }

    //
    // Blue Autonomous Cube Routines
    //

    public Command loadBlueCenterPlaceBalanceCube() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "BlueCenterPlaceBalanceCube", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();

        
        eventMap.put("placeHighCube", new AutoPlaceHighCubeCommand(armSubsystem));
        eventMap.put("ejectCube", new AutoEjectCubeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
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

        return fullAuto;
    }

    public Command loadBlueLeftPlaceTaxiCube() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "BlueLeftPlaceTaxiCube", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        
        /*eventMap.put("placeHighCube", new AutoPlaceHighCommand(armSubsystem));
        eventMap.put("ejectCube", new AutoEjectCubeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
        eventMap.put("balance", new AutoBalanceCommand(drivetrain));*/


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

        return fullAuto;
    }

    public Command loadBlueDoubleCube() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "BlueLeftDoubleCube", 
            new PathConstraints(1.5, 1.5)
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        
        eventMap.put("placeHighCube", new AutoPlaceHighCubeCommand(armSubsystem));
        eventMap.put("ejectCube", new AutoEjectCubeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
        eventMap.put("balance", new AutoBalanceCommand(drivetrain));
        eventMap.put("intakeCube", new AutoIntakeCubeCommand(armSubsystem));
        eventMap.put("finishedIntakeCube", new AutoFinishedIntakeCubeCommand(armSubsystem));
        eventMap.put("placeHighCube2", new AutoPlaceHighCubeCommand(armSubsystem));
        eventMap.put("ejectCube2", new AutoEjectCubeCommand(armSubsystem));
        eventMap.put("fold2", new AutoFoldCommand(armSubsystem));


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

        return fullAuto;
    }

    public Command loadRedDoubleCube() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "RedRightDoubleCube", 
            new PathConstraints(1.5, 1.5)
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        
        eventMap.put("placeHighCube", new AutoPlaceHighCubeCommand(armSubsystem));
        eventMap.put("ejectCube", new AutoEjectCubeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
        eventMap.put("balance", new AutoBalanceCommand(drivetrain));
        eventMap.put("intakeCube", new AutoIntakeCubeCommand(armSubsystem));
        eventMap.put("finishedIntakeCube", new AutoFinishedIntakeCubeCommand(armSubsystem));
        eventMap.put("placeHighCube2", new AutoPlaceHighCubeCommand(armSubsystem));
        eventMap.put("ejectCube2", new AutoEjectCubeCommand(armSubsystem));
        eventMap.put("fold2", new AutoFoldCommand(armSubsystem));


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

        return fullAuto;
    }

    //
    // Red Autonomous Routines
    //

    public Command loadRedCenterPlaceBalance() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "RedCenterPlaceBalance", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();

        
        eventMap.put("placeHigh", new AutoPlaceHighCommand(armSubsystem));
        eventMap.put("eject", new AutoEjectConeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
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

        return fullAuto;
    }

    public Command loadRedLeftPlaceTaxi() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "RedLeftPlaceTaxi", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();

        
        eventMap.put("placeHigh", new AutoPlaceHighCommand(armSubsystem));
        eventMap.put("eject", new AutoEjectConeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
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

        return fullAuto;
    }

    public Command loadRedRightPlaceTaxi() {
        // Load the basic blue
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "RedRightPlaceTaxi", 
            new PathConstraints(1.0, 1.0)
        );

        HashMap<String, Command> eventMap = new HashMap<>();

        
        eventMap.put("placeHigh", new AutoPlaceHighCommand(armSubsystem));
        eventMap.put("eject", new AutoEjectConeCommand(armSubsystem));
        eventMap.put("fold", new AutoFoldCommand(armSubsystem));
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

        return fullAuto;
    }
}
