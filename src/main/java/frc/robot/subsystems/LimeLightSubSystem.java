package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.sql.Timestamp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubSystem extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tl = table.getEntry("tl");
    NetworkTableEntry getpipe = table.getEntry("getpipe");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    NetworkTableEntry botpose = table.getEntry("botpose");
    NetworkTableEntry botpose_wpiblue = table.getEntry("botpose_wpiblue");
    NetworkTableEntry botpose_wpired = table.getEntry("botpose_wpired");
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Limelight");


    double tempX = 0.0;
    double tempY = 0.0;

    NetworkTableEntry ledModeTableEntry = table.getEntry("ledMode");

    Pose3d pose = new Pose3d();
    Pose3d poseBlue = new Pose3d();
    Pose3d poseRed = new Pose3d();
    Translation3d tran3d;
    Rotation3d rot3d;

    boolean ledMode = true;
    boolean canSeeAprilTags = false;

    double x, y, area = 0.0;
    long v, latency = 0;
    double[] botpose_array = new double[6];
    double[] botpose_array_wpiblue = new double[6];
    double[] botpose_array_wpired = new double[6];

    double ledOff = 1;
    double ledOn = 3;

    long _pipeline = 0;

    long timeStamp = 0;

    public LimeLightSubSystem() {
        //post to smart dashboard periodically
        shuffleboardTab.addDouble("LimelightX", () -> x);
        shuffleboardTab.addDouble("LimelightY", () -> y);
        shuffleboardTab.addDouble("LimelightArea", () -> area);
        shuffleboardTab.addDouble("Botpose 0", () -> botpose_array[0]);
        shuffleboardTab.addDouble("Botpose 1", () -> botpose_array[1]);
        shuffleboardTab.addDouble("Botpose 2", () -> botpose_array[2]);
        shuffleboardTab.addDouble("Botpose 3", () -> botpose_array[3]);
        shuffleboardTab.addDouble("Botpose 4", () -> botpose_array[4]);
        shuffleboardTab.addDouble("Botpose 5", () -> botpose_array[5]);
        shuffleboardTab.addDouble("pose X", () -> pose.getX());
        shuffleboardTab.addDouble("pose Y", () -> pose.getY());
        shuffleboardTab.addCamera("Limelight_camera", "LL_Camera", "http://10.42.19.11:5800");
        shuffleboardTab.addBoolean("April Tags", () -> canSeeAprilTags);
        shuffleboardTab.addInteger("Latency", () -> latency);
        shuffleboardTab.addInteger("Pipeline", () -> _pipeline);
        shuffleboardTab.addDouble("wpiblue[0]", () -> botpose_array_wpiblue[0]);

        //SmartDashboard.putBoolean("April Tag Visible", canSeeAprilTags);
    }

    @Override
    public void periodic() {
        //read values periodically
        x = table.getEntry("tx").getDouble(0.0);
        y = table.getEntry("ty").getDouble(0.0);
        v = table.getEntry("tv").getInteger(0);
        latency = table.getEntry("tl").getInteger(0);
        area = table.getEntry("ta").getDouble(0.0);
        botpose_array = table.getEntry("botpose").getDoubleArray(botpose_array);
        botpose_array_wpiblue = table.getEntry("botpose_wpiblue").getDoubleArray(botpose_array_wpiblue);
        botpose_array_wpired = table.getEntry("botpose_wpired").getDoubleArray(botpose_array_wpired);
        _pipeline = table.getEntry("getpipe").getInteger(0);

        //System.out.println("length: " + botpose_array.length);

        // this is needed when the limelight can't see any april tags and loses its pose
        //if(botpose_array.length <= 0) {
        if(v == 0 || botpose_array.length <= 0) {
            botpose_array = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            canSeeAprilTags = false;
        } else {

            // the field is 1654 long, by 802 wide
            if(botpose_array[0] > 0) {
                //tempX = 8.27 - botpose_array[0];
                tempX = (1654/2 - botpose_array[0]*100)/100;
            } else {
                //tempX = 8.27 + botpose_array[0];
                tempX = (1654/2 + botpose_array[0]*100)/100;
            }

            if(botpose_array[1] > 0) {
                //tempY = 4.01 - botpose_array[1];
                tempY = (802/2 - botpose_array[1] * 100)/100;
            } else {
                //tempY = 4.01 + botpose_array[1];
                tempY = (802/2 + botpose_array[1] * 100)/100;
            }

            //tran3d = new Translation3d(botpose_array[0], botpose_array[1], botpose_array[2]);
            tran3d = new Translation3d(tempX, tempY, botpose_array[2]);
            rot3d = new Rotation3d(botpose_array[3], botpose_array[4], botpose_array[5]);
            pose = new Pose3d(tran3d, rot3d);

            canSeeAprilTags = true;
        }

        tran3d = new Translation3d(botpose_array_wpiblue[0], botpose_array_wpiblue[1], botpose_array_wpiblue[2]);
        rot3d = new Rotation3d(botpose_array_wpiblue[3], botpose_array_wpiblue[4], botpose_array_wpiblue[5]);
        poseBlue = new Pose3d(tran3d, rot3d);

        this.timeStamp = System.currentTimeMillis() - latency;
    }

    public Pose3d getBotPose() {
        return this.pose;
    }

    public Pose3d getBluePose() {
        return this.poseBlue;
    }

    public Pose3d getRedPose() {
        return this.poseRed;
    }

    public boolean toggleLED() {

        if(ledMode == true) {
            ledMode = false;
            ledModeTableEntry.setDouble(ledOff);
        } else {
            ledMode = true;
            ledModeTableEntry.setDouble(ledOn);
        }

        return ledMode;
    }

    boolean canSeeAprilTags() {
        return this.canSeeAprilTags;
    }

    long getTimeStamp() {
        return this.timeStamp;
    }

    public double getTargetX() {
        return x;
    }

    public double getTargetY() {
        return y;
    }

    public double getTargetRotation() {
        return 0.0;
    }

    public long getPipeline() {
        return _pipeline;
    }

    public void setPipeline(long pipeline) {
        this._pipeline = pipeline;
    }

    public void togglePipeline() {

        System.out.println("togglePipeline called");

        if(_pipeline == 0) {
            _pipeline = 1;
        } else {
            _pipeline = 0;
        }

        pipeline.setDouble(_pipeline);
    }
}
