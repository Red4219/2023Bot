package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax baseMotor = new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax highMotor = new CANSparkMax(Constants.ARM_HIGH_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.ARM_WRIST_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.ARM_INTAKE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    //private CANSparkMax baseMotor;
    //private CANSparkMax highMotor;
    //private CANSparkMax wristMotor;
    //private CANSparkMax intakeMotor;

    private final RelativeEncoder baseEncoder = baseMotor.getEncoder();
    private final RelativeEncoder highEncoder = highMotor.getEncoder();
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    //private RelativeEncoder baseEncoder;
    //private RelativeEncoder highEncoder;
    //private RelativeEncoder wristEncoder;
    //private RelativeEncoder intakeEncoder;

    double rightTriggerValue, leftTriggerValue = 0;

    XboxController operatorController;

    //
    // Simulator Stuff
    //
    //private EncoderSim baseEncoderSim;
    //private EncoderSim highEncoderSim;
    //private EncoderSim wristEncoderSim;
    //private EncoderSim intakeEncoderSim;
    private REVPhysicsSim revPhysicsSim;

    public ArmSubsystem(XboxController operatorController) {

        // If we are in a simulation, set up the simulated encoders
        /*if(RobotBase.isReal() == false) {
            
            //baseEncoderSim = new En
            //highEncoderSim = new EncoderSim((Encoder) highEncoder);
            //wristEncoderSim = new EncoderSim((Encoder) wristEncoder);
            //intakeEncoderSim = new EncoderSim((Encoder) intakeEncoder);
        } else {
            baseMotor = new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
            highMotor = new CANSparkMax(Constants.ARM_HIGH_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
            wristMotor = new CANSparkMax(Constants.ARM_WRIST_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
            intakeMotor = new CANSparkMax(Constants.ARM_INTAKE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

            baseEncoder = baseMotor.getEncoder();
            highEncoder = highMotor.getEncoder();
            wristEncoder = wristMotor.getEncoder();
            intakeEncoder = intakeMotor.getEncoder();
        }*/

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Arm");

        shuffleboardTab.addNumber("Base", () -> baseEncoder.getPosition());
        shuffleboardTab.addNumber("High", () -> highEncoder.getPosition());
        shuffleboardTab.addNumber("Wrist", () -> wristEncoder.getPosition());
        shuffleboardTab.addNumber("Intake", () -> intakeEncoder.getPosition());
        shuffleboardTab.addNumber("Intake RT", () -> rightTriggerValue);
        shuffleboardTab.addNumber("Intake LT", () -> leftTriggerValue);

        this.operatorController = operatorController;
    }

    public void setHighMotor(double speed) {
        highMotor.set(speed);
        highMotor.setVoltage(speed);
        //System.out.println("encoder value: " + highEncoder.getPosition() + ", speed: " + speed);
    }

    public RelativeEncoder getHighEncoder() {
        return this.highEncoder;
    }

    public SparkMaxPIDController getHighMotorPIDController() {
        return highMotor.getPIDController();
    }

    public void setBaseMotor(double speed) {
        //System.out.println("encoder value: " + baseEncoder.getPosition());
        baseMotor.set(speed);
    }

    public SparkMaxPIDController getBaseMotorPIDController() {
        return baseMotor.getPIDController();
    }

    public void setWristMotor(double speed) {
        wristMotor.set(speed);
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    public SparkMaxPIDController getIntakeMotorPIDController() {
        return intakeMotor.getPIDController();
    }
    
    @Override
    public void periodic() {

        // Check if the operator is pulling the triggers to run the intake
        rightTriggerValue = this.operatorController.getRightTriggerAxis();
        leftTriggerValue = this.operatorController.getRightTriggerAxis();

        if(rightTriggerValue > 0) {
            intakeMotor.set(rightTriggerValue * Constants.ARM_INTAKE_SPEED_MULTIPLIER);
        } else if(leftTriggerValue > 0) {
            intakeMotor.set(-leftTriggerValue * Constants.ARM_INTAKE_SPEED_MULTIPLIER);
        }

        //System.out.println("encoder value: " + baseEncoder.getPosition());
    }

    public void setRevPhysicsSim(REVPhysicsSim sim) {
        this.revPhysicsSim = sim;

        //sim.addSparkMax(baseMotor, DCMotor.getNEO(1));
        //sim.addSparkMax(highMotor, DCMotor.getNEO(1));

        System.out.println("ArmSubsystem::setRevPhysicsSim() called");
        REVPhysicsSim.getInstance().addSparkMax(highMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(baseMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(wristMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNEO(1));

        /*sim.addSparkMax(
            new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless), 
            DCMotor.getNEO(1)
        );*/
    }

}
 