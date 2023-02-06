package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.DriveCommand;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final AHRS gyroscope = new AHRS(SPI.Port.kMXP);

    //LimeLightSubSystem limelight = new LimeLightSubSystem();
    LimeLightSubSystem limelight;
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    private final Field2d field = new Field2d();
    XboxController driverController;
    XboxController operatorController;
    boolean useLimeLightForPoseCorrection = false;
    DriveCommand driveCommand;
    //private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private double tempX = 3;
    private double tempY = 5;
    private double tempDegrees = 180;

    private boolean autoAiming = false;

    public Gyro getGyro() {
        return gyroscope;
    }

    public void setDriverController(XboxController controller) {
        this.driverController = controller;
    }

    public XboxController getDriverController() {
        return this.driverController;
    }

    public void setOperatorController(XboxController controller) {
        this.operatorController = controller;
    }

    public XboxController getOperatorController() {
        return this.operatorController;
    }

    //public void setDriveCommand(DriveCommand driveCommand) {
    public void setDriveCommand(Command command) {
        if(command instanceof DriveCommand ) {
                this.driveCommand = (DriveCommand) command;
        } else if(command instanceof AutoAimCommand) {
                
        }
    }

    public void setControllers(XboxController driver, XboxController operator) {
        this.driverController = driver;
        this.operatorController = operator;
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        

        frontLeftModule = Mk3SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk3SwerveModuleHelper.GearRatio.MK4219,
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk3SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.MK4219,
                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk3SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.MK4219,
                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk3SwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.MK4219,
                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        // Create the initial versions of the SwerveModulePositions
        // They will just be updated in the future during teleop and autonomous
        swerveModulePositions[0] = new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle()));
        swerveModulePositions[1] = new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle()));        
        swerveModulePositions[2] = new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle()));      
        swerveModulePositions[3] = new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()));      

        //odometry = new SwerveDriveOdometry(
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, 
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                //swerveModulePositions,
                new SwerveModulePosition[]{
                        new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
            },
             new Pose2d(0, 0, 
             gyroscope.getRotation2d())
        );

        //System.out.println("----------> just created the odometry, poseX is: " + odometry.getPoseMeters().getX() + ", poseY is: " + odometry.getPoseMeters().getY());

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        //shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose X", () -> poseEstimator.getEstimatedPosition().getX());
        //shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
        shuffleboardTab.addNumber("Pose Y", () -> poseEstimator.getEstimatedPosition().getY());

        shuffleboardTab.addNumber("FL Velocity", () -> frontLeftModule.getDriveVelocity());
        shuffleboardTab.addNumber("FR Velocity", () -> frontRightModule.getDriveVelocity());
        shuffleboardTab.addNumber("BL Velocity", () -> backLeftModule.getDriveVelocity());
        shuffleboardTab.addNumber("BR Velocity", () -> backRightModule.getDriveVelocity());

        shuffleboardTab.addBoolean("Field Oriented", () -> this.driveCommand.getFieldOriented());
        shuffleboardTab.addBoolean("LimeLight Field Correction", () -> this.useLimeLightForPoseCorrection);

        SmartDashboard.putData(field);

        setPose(
                new Pose2d(
                        new Translation2d(tempX, tempY), 
                        Rotation2d.fromDegrees(tempDegrees)
                )
        );
    }

    public LimeLightSubSystem getLimeLight() {
        return this.limelight;
    }

    public void setLimeLight(LimeLightSubSystem limelight) {
        this.limelight = limelight;
    }

    public Pose2d getPose() {
        //System.out.println("-----------------> getPose called, x: " + odometry.getPoseMeters().getX() + ", y: " + odometry.getPoseMeters().getY());
        //return odometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        //System.out.println("-----------------> setPose called x: " + pose.getX() + ", y: " + pose.getY());

        /*swerveModulePositions[0].distanceMeters = frontLeftModule.getPosition();        
        swerveModulePositions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());

        swerveModulePositions[1].distanceMeters = frontRightModule.getPosition();        
        swerveModulePositions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());

        swerveModulePositions[2].distanceMeters = backLeftModule.getPosition();        
        swerveModulePositions[3].angle = new Rotation2d(backLeftModule.getSteerAngle());

        swerveModulePositions[3].distanceMeters = backRightModule.getPosition();        
        swerveModulePositions[3].angle = new Rotation2d(backRightModule.getSteerAngle());*/

        /*odometry.resetPosition(
                //pose.getRotation(),
                gyroscope.getRotation2d(),
                //swerveModulePositions,
        new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
            },
                pose
        );*/

        poseEstimator.resetPosition(
                //pose.getRotation(),
                gyroscope.getRotation2d(),
                //swerveModulePositions,
        new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
            },
                pose
        );
    }

    public void resetPose() {
        System.out.println("------------------> resetPose() called");

        /*swerveModulePositions[0].distanceMeters = frontLeftModule.getPosition();        
        swerveModulePositions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());

        swerveModulePositions[1].distanceMeters = frontRightModule.getPosition();        
        swerveModulePositions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());

        swerveModulePositions[2].distanceMeters = backLeftModule.getPosition();        
        swerveModulePositions[3].angle = new Rotation2d(backLeftModule.getSteerAngle());

        swerveModulePositions[3].distanceMeters = backRightModule.getPosition();        
        swerveModulePositions[3].angle = new Rotation2d(backRightModule.getSteerAngle());*/

        //odometry.resetPosition(
        poseEstimator.resetPosition(
                //Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                gyroscope.getRotation2d(),
                //swerveModulePositions,
                new SwerveModulePosition[]{
                        new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
            },
        new Pose2d(0,0,gyroscope.getRotation2d()));
    }

    public SwerveDriveKinematics getKinimatics() {
        return this.kinematics;
    }

    public void setModuleStates(SwerveModuleState[] states) {

        System.out.println("----------------->setModuleStates() called");

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    public void zeroGyroscope() {
        /*odometry.resetPosition(
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                Rotation2d.fromDegrees(gyroscope.getFusedHeading())
        );*/

        swerveModulePositions[0].distanceMeters = frontLeftModule.getPosition();        
        swerveModulePositions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());

        swerveModulePositions[1].distanceMeters = frontRightModule.getPosition();        
        swerveModulePositions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());

        swerveModulePositions[2].distanceMeters = backLeftModule.getPosition();        
        swerveModulePositions[3].angle = new Rotation2d(backLeftModule.getSteerAngle());

        swerveModulePositions[3].distanceMeters = backRightModule.getPosition();        
        swerveModulePositions[3].angle = new Rotation2d(backRightModule.getSteerAngle());

        gyroscope.reset();

        //odometry.resetPosition(
        poseEstimator.resetPosition(
                //Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                gyroscope.getRotation2d(),
                swerveModulePositions,
                new Pose2d(
                        //odometry.getPoseMeters().getTranslation(),
                        poseEstimator.getEstimatedPosition().getTranslation(),
                        Rotation2d.fromDegrees(gyroscope.getFusedHeading())
                )
        );
    }

    public Rotation2d getRotation() {
        //return odometry.getPoseMeters().getRotation();
        return poseEstimator.getEstimatedPosition().getRotation();
        //return gyroscope.getRotation2d();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        //System.out.println("---------------------> drive called");
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {

        if(this.driverController.getRawButtonPressed(Constants.DRIVER_BUTTON_DISABLE_LED)) {
                limelight.toggleLED();
        }

        if(this.driverController.getRawButtonPressed(Constants.DRIVER_BUTTON_RESET_GYRO)) {
                //zeroGyroscope();
        }

        if(this.driverController.getRawButtonPressed(Constants.DRIVER_BUTTON_TOGGLE_FIELD_ORIENTED)) {
                this.driveCommand.toggleFieldOriented();
        }

        if(this.driverController.getRawButtonPressed(Constants.DRIVER_BUTTON_TOGGLE_LIMELIGHT_MODE)) {
                this.limelight.togglePipeline();
        }
        
        limelight.periodic();
        //double[] limelightpose = limelight.getBotPose();

        if(this.driverController.getRawButtonPressed(Constants.DRIVER_BUTTON_TOGGLE_LIMELIGHT_POSITION_CORRECTION)) {
                if(this.useLimeLightForPoseCorrection) {
                        this.useLimeLightForPoseCorrection = false;
                } else {
                        this.useLimeLightForPoseCorrection = true;
                }
        }

        
        // Check for auto aim
        if(this.driverController.getRightTriggerAxis() > 0) {

            double temp = this.driverController.getRightTriggerAxis();

            //System.out.println(temp);
            
            //if(!autoAiming) {
                AutoAimCommand autoAimCommand = new AutoAimCommand(
                        this, 
                        () -> this.limelight.getTargetX(), 
                        () -> this.limelight.getTargetY(),
                        () -> this.limelight.getTargetRotation()
                );

                autoAimCommand.execute();
            //}

            autoAiming = true;
        } else {
                autoAiming = false;
        }

        if(limelight.canSeeTarget() && useLimeLightForPoseCorrection) {
                poseEstimator.addVisionMeasurement(limelight.getBluePose().toPose2d(), limelight.getTimeStamp());
        }

        //odometry.update(
        poseEstimator.update(
                gyroscope.getRotation2d(),
                //swerveModulePositions
                new SwerveModulePosition[]{
                        new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
                }
        );

        //field.setRobotPose(odometry.getPoseMeters());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        var p = poseEstimator.getEstimatedPosition();
        this.states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
