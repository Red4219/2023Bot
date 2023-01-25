package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    double x, y, area = 0.0;
    double[] botpose_array = new double[3];

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
    }

    @Override
    public void periodic() {
        //read values periodically
        x = table.getEntry("tx").getDouble(0.0);
        y = table.getEntry("ty").getDouble(0.0);
        area = table.getEntry("ta").getDouble(0.0);

        botpose_array = table.getEntry("botpose").getDoubleArray(botpose_array);

        // this is needed when the limelight can't see any april tags and loses its pose
        if(botpose_array.length <= 0) {
            botpose_array = new double[]{0.0, 0.0, 0.0};
        }

        //System.out.println(botpose_array[0]);
    }

    public double[] getBotPose() {
        return botpose_array;
    }
}
