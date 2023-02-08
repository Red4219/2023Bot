package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax baseMotor = new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax highMotor = new CANSparkMax(Constants.ARM_HIGH_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    private final RelativeEncoder baseEncoder = baseMotor.getEncoder();
    private final RelativeEncoder highEncoder = highMotor.getEncoder();
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

    public ArmSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Arm");

        shuffleboardTab.addNumber("Base", () -> baseEncoder.getPosition());
        shuffleboardTab.addNumber("High", () -> highEncoder.getPosition());
        shuffleboardTab.addNumber("Wrist", () -> wristEncoder.getPosition());
    }

    public void setHighMotor(double speed) {
        highMotor.set(speed);
    }

    public void setBaseMotor(double speed) {
        baseMotor.set(speed);
    }

    public void setWristMotor(double speed) {
        wristMotor.set(speed);
    }

    public void highPosition() {
        
    }

    public void midPosition() {

    }

    public void lowPosition() {

    }
    
    @Override
    public void periodic() {
    }


}
 