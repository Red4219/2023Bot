package frc.robot.subsystems;


import frc.robot.Constants;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.SPI;
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

    LimeLightSubSystem limelight = new LimeLightSubSystem();
    SwerveModuleState[] states = new SwerveModuleState[4];

    //Pose2d currentPose = new Pose2d();

    public Gyro getGyro() {
        return gyroscope;
    }

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    /*private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyroscope.getFusedHeading()), new SwerveModulePosition[]{
        new SwerveModulePosition(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
        new SwerveModulePosition(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
        new SwerveModulePosition(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
        new SwerveModulePosition(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
    });*/

    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

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

        //frontLeftModule.

        odometry = new SwerveDriveOdometry(
                kinematics, 
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[]{
                        new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
            }, new Pose2d(0, 0, gyroscope.getRotation2d()));

        System.out.println("----------> just created the odometry, poseX is: " + odometry.getPoseMeters().getX() + ", poseY is: " + odometry.getPoseMeters().getY());

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());

        shuffleboardTab.addNumber("FL Velocity", () -> frontLeftModule.getDriveVelocity());
        shuffleboardTab.addNumber("FR Velocity", () -> frontRightModule.getDriveVelocity());
        shuffleboardTab.addNumber("BL Velocity", () -> backLeftModule.getDriveVelocity());
        shuffleboardTab.addNumber("BR Velocity", () -> backRightModule.getDriveVelocity());
    }

    public Pose2d getPose() {
        System.out.println("-----------------> getPose called, x: " + odometry.getPoseMeters().getX() + ", y: " + odometry.getPoseMeters().getY());
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        System.out.println("-----------------> setPose called x: " + pose.getX() + ", y: " + pose.getY());
        odometry.resetPosition(pose.getRotation(), 
        new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
            },
        pose);
    }

    public void resetPose() {
        System.out.println("------------------> resetPose() called");
        odometry.resetPosition(
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[]{
                        new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
            },
        new Pose2d(0,0,gyroscope.getRotation2d()));
    }

    public SwerveDriveKinematics getKinimatics() {

        System.out.println("----------------->getKinimatics() called");

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
        /*odometry.resetPosition(
                //Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                gyroscope.getRotation2d(),
                new SwerveModulePosition[]{},
                new Pose2d(
                        odometry.getPoseMeters().getTranslation(), 
                        Rotation2d.fromDegrees(gyroscope.getFusedHeading())
                )
        );*/
    }

    public Rotation2d getRotation() {
        //return odometry.getPoseMeters().getRotation();
        return gyroscope.getRotation2d();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        /*odometry.update(Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
        );*/

        //System.out.println("gyro is: " + gyroscope.getFusedHeading());

        limelight.periodic();
        double[] limelightpose = limelight.getBotPose();

        //odometry.resetPosition(gyroscope.getRotation2d(), null, Pose2d);
        //Pose2d limelight_pose = new Pose2d(limelightpose[0], limelightpose[1], gyroscope.getRotation2d());

        /*odometry.resetPosition(gyroscope.getRotation2d(),
        new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeftModule.getDriveVelocity(), new Rotation2d(Math.toRadians(frontLeftModule.getSteerAngle()))),
                new SwerveModulePosition(frontRightModule.getDriveVelocity(), new Rotation2d(Math.toRadians(frontRightModule.getSteerAngle()))),
                new SwerveModulePosition(backLeftModule.getDriveVelocity(), new Rotation2d(Math.toRadians(backLeftModule.getSteerAngle()))),
                new SwerveModulePosition(backRightModule.getDriveVelocity(), new Rotation2d(Math.toRadians(backRightModule.getSteerAngle())))
        },
        limelight_pose);*/

        //System.out.println("getDriveVelocity: " + frontLeftModule.getDriveVelocity());
        //System.out.println("getDriveVelocity: " + frontRightModule.getDriveVelocity());
        //System.out.println("getDriveVelocity: " + backLeftModule.getDriveVelocity());
        //System.out.println("getDriveVelocity: " + backRightModule.getDriveVelocity());

        //frontLeftModule.

        odometry.update(
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                //gyroscope.getRotation2d(),
                //getRotation(),
                new SwerveModulePosition[]{
                        new SwerveModulePosition(frontLeftModule.getPosition(), new Rotation2d(frontLeftModule.getSteerAngle())),
                        //new SwerveModulePosition(Math.abs(frontLeftModule.getDriveVelocity()), new Rotation2d(frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(frontRightModule.getPosition(), new Rotation2d(frontRightModule.getSteerAngle())),
                        //new SwerveModulePosition(Math.abs(frontRightModule.getDriveVelocity()), new Rotation2d(frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(backLeftModule.getPosition(), new Rotation2d(backLeftModule.getSteerAngle())),
                        //new SwerveModulePosition(Math.abs(backLeftModule.getDriveVelocity()), new Rotation2d(backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(backRightModule.getPosition(), new Rotation2d(backRightModule.getSteerAngle()))
                        //new SwerveModulePosition(Math.abs(backRightModule.getDriveVelocity()), new Rotation2d(backRightModule.getSteerAngle()))
                }
        );

        //currentPose =  currentPose.minus(newPose);
        //currentPose = new Pose2d(currentPose.minus(newPose).getTranslation(), currentPose.minus(newPose).getTranslation().getAngle());

        

        //currentPose = currentPose.plus(newPose.getTranslation().);
        //edu.wpi.first.math.geometry.Transform2d transform2d = new Transform2d();

        this.states = kinematics.toSwerveModuleStates(chassisSpeeds);

        /*this.currentPose = odometry.update(
                //Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                //gyroscope.getRotation2d(),
                getRotation(),
                new SwerveModulePosition[]{
                        new SwerveModulePosition(states[0].speedMetersPerSecond, new Rotation2d(states[0].angle.getRadians())),
                        new SwerveModulePosition(states[1].speedMetersPerSecond, new Rotation2d(states[1].angle.getRadians())),
                        new SwerveModulePosition(states[2].speedMetersPerSecond, new Rotation2d(states[2].angle.getRadians())),
                        new SwerveModulePosition(states[3].speedMetersPerSecond, new Rotation2d(states[3].angle.getRadians())),
                }
        );*/

        /*chassisSpeeds = kinematics.toChassisSpeeds(new SwerveModuleState[]{
                new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
        });*/

        //SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        

        //var frontLeftOptimized = SwerveModuleState.optimize(frontLeftModule,
           //new Rotation2d(m_turningEncoder.getDistance()));

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    private double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / 2048;
        double wheelRotations = motorRotations / 8.14/1;
        double positionMeters = wheelRotations * (2 * Math.PI * (2 * 0.0254));
        return positionMeters;
      }

}
