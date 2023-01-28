package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    NetworkTableEntry botpose = table.getEntry("botpose");
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Limelight");

    Pose3d pose = new Pose3d();

    double x, y, area = 0.0;
    double[] botpose_array = new double[6];

    public LimeLightSubSystem() {
        //table = NetworkTableInstance.getDefault().getTable("limelight");
        //tx = table.getEntry("tx");
        //ty = table.getEntry("ty");
        //ta = table.getEntry("ta");
        //botpose = table.getEntry("botpose");
        

        //shuffleboardTab = Shuffleboard.getTab("Limelight");

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
        shuffleboardTab.addCamera("Limelight_camera", "LL_Camera", "http://10.42.19.102:5800");
    }

    @Override
    public void periodic() {
        //read values periodically
        x = table.getEntry("tx").getDouble(0.0);
        y = table.getEntry("ty").getDouble(0.0);
        area = table.getEntry("ta").getDouble(0.0);

        botpose_array = table.getEntry("botpose").getDoubleArray(botpose_array);

        //System.out.println(botpose_array.length);

        // this is needed when the limelight can't see any april tags and loses its pose
        if(botpose_array.length <= 0) {
            botpose_array = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        } else {

            Translation3d tran3d = new Translation3d(botpose_array[0], botpose_array[1], botpose_array[2]);

            Rotation3d rot3d = new Rotation3d(botpose_array[3], botpose_array[4], botpose_array[5]);

            pose = new Pose3d(tran3d, rot3d);

            //System.out.println("PoseX: " + pose.getX() + ", poseY: " + pose.getY());
        }

        //System.out.println(botpose_array[0]);
    }

    public double[] getBotPose() {
        return botpose_array;
    }
}
