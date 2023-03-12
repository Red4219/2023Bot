package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax baseMotor = new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax highMotor1 = new CANSparkMax(Constants.ARM_HIGH_MOTOR_ID_1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax highMotor2 = new CANSparkMax(Constants.ARM_HIGH_MOTOR_ID_2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wristMotor1 = new CANSparkMax(Constants.ARM_WRIST_MOTOR_ID_1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wristMotor2 = new CANSparkMax(Constants.ARM_WRIST_MOTOR_ID_2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.ARM_INTAKE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    // Setup the grouped motors (two motors that act as one)
    private final MotorControllerGroup baseGroup = new MotorControllerGroup(baseMotor);
    private final MotorControllerGroup highGroup = new MotorControllerGroup(highMotor1, highMotor2);
    private final MotorControllerGroup wristGroup = new MotorControllerGroup(wristMotor1, wristMotor2);

    //private CANSparkMax baseMotor;
    //private CANSparkMax highMotor;
    //private CANSparkMax wristMotor;
    //private CANSparkMax intakeMotor;

    private final RelativeEncoder baseEncoder = baseMotor.getEncoder();
    private final RelativeEncoder highEncoder1 = highMotor1.getEncoder();
    private final RelativeEncoder highEncoder2 = highMotor2.getEncoder();
    private final RelativeEncoder wristEncoder = wristMotor1.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private PIDController pidHigh = new PIDController(3.0, 0.0, 0.3);
    private PIDController pidWrist = new PIDController(3.0, 0.0, 0.3);
    private PIDController pidBase = new PIDController(3.0, 0.0, 0.3);

    private int LAST_PRESET_ID = Constants.OPERATOR_BUTTON_FOLD;

    private boolean usingPresetSetting = true;

    //private RelativeEncoder baseEncoder;
    //private RelativeEncoder highEncoder;
    //private RelativeEncoder wristEncoder;
    //private RelativeEncoder intakeEncoder;

    double rightTriggerValue = 0;
    double leftTriggerValue = 0;
    double rightY = 0;

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
        shuffleboardTab.addNumber("High", () -> highEncoder1.getPosition());
        shuffleboardTab.addNumber("Wrist", () -> wristEncoder.getPosition());
        shuffleboardTab.addNumber("Intake", () -> intakeEncoder.getPosition());
        shuffleboardTab.addNumber("Intake RT", () -> rightTriggerValue);
        shuffleboardTab.addNumber("Intake LT", () -> leftTriggerValue);
        shuffleboardTab.addNumber("RightY", () -> rightY);

        wristMotor1.setIdleMode(IdleMode.kBrake);
        wristMotor2.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        highMotor1.setIdleMode(IdleMode.kBrake);
        highMotor1.setInverted(true);
        highMotor2.setIdleMode(IdleMode.kBrake);
        baseMotor.setIdleMode(IdleMode.kBrake);

        this.operatorController = operatorController;

    }

    public void setHighMotor(double speed) {
        highMotor1.set(speed);
        highMotor1.setVoltage(speed);
        //System.out.println("encoder value: " + highEncoder.getPosition() + ", speed: " + speed);
    }

    public RelativeEncoder getHighEncoder() {
        return this.highEncoder1;
    }

    public SparkMaxPIDController getHighMotorPIDController() {
        return highMotor1.getPIDController();
    }

    public void setBaseMotor(double speed) {
        //System.out.println("encoder value: " + baseEncoder.getPosition());
        baseMotor.set(speed);
    }

    public SparkMaxPIDController getBaseMotorPIDController() {
        return baseMotor.getPIDController();
    }

    public void setWristMotor(double speed) {
        //wristMotor1.set(speed);
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
        leftTriggerValue = this.operatorController.getLeftTriggerAxis();

        //System.out.println("right: " + rightTriggerValue + ", left: " + leftTriggerValue);
        //System.out.println("encoder: " + this.highEncoder1.getPosition());

        //pid = new PIDController(0.5, 0.5, 0.5);

        //this works 
        /*double pidSetting = pid.calculate(highEncoder1.getPosition(), -4.5) / 100;
        //System.out.println("setting: " + pidSetting + ", position: " + highEncoder1.getPosition() + ", stick: " + operatorController.getLeftY());
        highGroup.set(pidSetting);*/

        // Move the intake
        /*if(rightTriggerValue < 0.0 || rightTriggerValue > 0.0) {
            intakeMotor.set(rightTriggerValue);
        } else*/ if(leftTriggerValue < 0.0 || leftTriggerValue > 0.0) {
            intakeMotor.set(-leftTriggerValue);
        } else {
            intakeMotor.set(0.0);
        }
        
        // Move the wrist
        //if(operatorController.getRightY() > 0.0 || operatorController.getRightY() < 0.0) {
        if(rightTriggerValue > 0.1 || rightTriggerValue < 0.1) {
            //wristMotor1.set(operatorController.getRightY() * Constants.ARM_INTAKE_WRIST_MULTIPLIER);
            //wristGroup.set(operatorController.getRightY() * Constants.ARM_INTAKE_WRIST_MULTIPLIER);
            //wristGroup.set(rightTriggerValue * Constants.ARM_INTAKE_WRIST_MULTIPLIER);
            wristGroup.set(rightTriggerValue);
            //rightY = operatorController.getRightY();
        } else {
            wristGroup.set(0.0);
        }



        //System.out.println(operatorController.getLeftY());
        //highGroup.set(0.02);

        //System.out.println("setting to: " + operatorController.getLeftY() * Constants.ARM_HIGH_BAR_MULTIPLIER);

        // Move the high bar
        if(operatorController.getLeftY() > 0.1 || operatorController.getLeftY() < -0.1) {
            highGroup.set(operatorController.getLeftY() * Constants.ARM_HIGH_BAR_MULTIPLIER);
            usingPresetSetting = false;
        } else {
            highGroup.set(0.0);
        }

        if(operatorController.getRawButtonPressed(Constants.OPERATOR_BUTTON_HIGH)) {
            usingPresetSetting = true;
            LAST_PRESET_ID = Constants.OPERATOR_BUTTON_HIGH;
        }

        if(operatorController.getRawButtonPressed(Constants.OPERATOR_BUTTON_MID)) {
            usingPresetSetting = true;
            LAST_PRESET_ID = Constants.OPERATOR_BUTTON_MID;
        }

        if(usingPresetSetting) {

            double pidHighSetting = 0.0;
            double pidWristSetting = 0.0;
            double pidBaseSetting = 0.0;

            switch(LAST_PRESET_ID) {
                case Constants.OPERATOR_BUTTON_HIGH:
                pidHighSetting = pidHigh.calculate(highEncoder1.getPosition(), Constants.ARM_HIGH_ENCODER_VALUE) / 100;
                    highGroup.set(pidHighSetting);
                pidWristSetting = pidWrist.calculate(wristEncoder.getPosition(), Constants.ARM_HIGH_WRIST_ENCODER_VALUE) / 100;
                    wristGroup.set(pidWristSetting);
                break;
                case Constants.OPERATOR_BUTTON_MID:
                pidHighSetting = pidHigh.calculate(highEncoder1.getPosition(), Constants.ARM_MID_ARM_ENCODER_VALUE) / 100;
                    highGroup.set(pidHighSetting);
                pidWristSetting = pidWrist.calculate(wristEncoder.getPosition(), Constants.ARM_MID_WRIST_ENCODER_VALUE) / 100;
                    wristGroup.set(pidWristSetting);
                break;
            }
        }
        
        //System.out.println("operatorController.getLeftY()" + operatorController.getLeftY());
        //System.out.println("rightTriggerValue: " + rightTriggerValue + ", leftTriggerValue: " + leftTriggerValue);
        //System.out.println("highEncoder1::Position: " + highEncoder1.getPosition());
    }

    public void setRevPhysicsSim(REVPhysicsSim sim) {
        this.revPhysicsSim = sim;

        //sim.addSparkMax(baseMotor, DCMotor.getNEO(1));
        //sim.addSparkMax(highMotor, DCMotor.getNEO(1));

        System.out.println("ArmSubsystem::setRevPhysicsSim() called");
        REVPhysicsSim.getInstance().addSparkMax(highMotor1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(baseMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(wristMotor1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNEO(1));

        /*sim.addSparkMax(
            new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless), 
            DCMotor.getNEO(1)
        );*/
    }

}
 