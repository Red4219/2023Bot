package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// When the code starts, it loads a new instance of Robot from Robot.java

public class Main {
    public static void main(String[] args) {
        RobotBase.startRobot(Robot::new);
    }
}
