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

    private final CANSparkMax baseMotor1 = new CANSparkMax(Constants.ARM_BASE_MOTOR_ID_1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax baseMotor2 = new CANSparkMax(Constants.ARM_BASE_MOTOR_ID_2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax highMotor1 = new CANSparkMax(Constants.ARM_HIGH_MOTOR_ID_1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax highMotor2 = new CANSparkMax(Constants.ARM_HIGH_MOTOR_ID_2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wristMotor1 = new CANSparkMax(Constants.ARM_WRIST_MOTOR_ID_1, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax wristMotor2 = new CANSparkMax(Constants.ARM_WRIST_MOTOR_ID_2, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.ARM_INTAKE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    // Setup the grouped motors (two motors that act as one)
    private final MotorControllerGroup baseGroup = new MotorControllerGroup(baseMotor1, baseMotor2);
    private final MotorControllerGroup highGroup = new MotorControllerGroup(highMotor1, highMotor2);
    private final MotorControllerGroup wristGroup = new MotorControllerGroup(wristMotor1, wristMotor2);

    //private CANSparkMax baseMotor;
    //private CANSparkMax highMotor;
    //private CANSparkMax wristMotor;
    //private CANSparkMax intakeMotor;

    private final RelativeEncoder baseEncoder1 = baseMotor1.getEncoder();
    private final RelativeEncoder highEncoder1 = highMotor1.getEncoder();
    private final RelativeEncoder highEncoder2 = highMotor2.getEncoder();
    private final RelativeEncoder wristEncoder = wristMotor1.getEncoder();
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private PIDController pidHigh = new PIDController(2.8, 4.0, 0.0);
    private PIDController pidWrist = new PIDController(3.0, 0.0, 0.0);
    private PIDController pidBase = new PIDController(2.5, 2.0, 0.3);

    double rightStickValue = 0;
    double triggerValues = 0;
    double rightY = 0;
    double leftY = 0;

    double wristTargetPosition = Constants.ARM_FOLD_WRIST_ENCODER_VALUE;
    double armTargetPosition = Constants.ARM_FOLD_ARM_ENCODER_VALUE;
    double baseTargetPosition = Constants.ARM_FOLD_BASE_ENCODER_VALUE;

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

        shuffleboardTab.addNumber("Base", () -> baseEncoder1.getPosition());
        shuffleboardTab.addNumber("High", () -> highEncoder1.getPosition());
        shuffleboardTab.addNumber("Wrist", () -> wristEncoder.getPosition());
        shuffleboardTab.addNumber("Intake", () -> intakeEncoder.getPosition());
        shuffleboardTab.addNumber("RightStick", () -> rightStickValue);
        shuffleboardTab.addNumber("Intake Triggers", () -> triggerValues);
        shuffleboardTab.addNumber("RightY", () -> rightY);
        shuffleboardTab.addNumber("LeftY", () -> leftY);

        wristMotor1.setIdleMode(IdleMode.kBrake);
        wristMotor2.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        highMotor1.setIdleMode(IdleMode.kBrake);
        highMotor1.setInverted(true);
        highMotor2.setIdleMode(IdleMode.kBrake);
        baseMotor1.setIdleMode(IdleMode.kBrake);
        baseMotor2.setIdleMode(IdleMode.kBrake);
        baseMotor2.setInverted(true);

        this.operatorController = operatorController;
    }

    public void setHighMotor(double speed) {
        highMotor1.set(speed);
        highMotor1.setVoltage(speed);
    }

    public RelativeEncoder getHighEncoder() {
        return this.highEncoder1;
    }

    public SparkMaxPIDController getHighMotorPIDController() {
        return highMotor1.getPIDController();
    }

    public void setBaseMotor(double speed) {
        //System.out.println("encoder value: " + baseEncoder.getPosition());
        //baseMotor.set(speed);
    }

    public SparkMaxPIDController getBaseMotorPIDController() {
        return baseMotor1.getPIDController();
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
        rightStickValue = this.operatorController.getRightX();
        triggerValues = this.operatorController.getLeftTriggerAxis();

        // Move the intake
        if(triggerValues < 0.0 || triggerValues > 0.0) {
            intakeMotor.set(-triggerValues);
        } else {
            intakeMotor.set(0.0);
        }
        
        // Wrist
        if(rightStickValue > 0.1 || rightStickValue < -0.1) {
            wristTargetPosition += (rightStickValue * .5);
        } 

        double wristtemp = 0.0;
        wristtemp = pidWrist.calculate(wristEncoder.getPosition(), wristTargetPosition) / 100;
        wristGroup.set(wristtemp);

        // Arm
        if(operatorController.getLeftY() > 0.1 || operatorController.getLeftY() < -0.1) {
            armTargetPosition += (operatorController.getLeftY() * .2);
        } 

        double armtemp = 0.0;
        armtemp = pidHigh.calculate(highEncoder1.getPosition(), armTargetPosition) / 100;
        highGroup.set(armtemp);

        // Base
        if(operatorController.getRightBumper()) {
            baseTargetPosition += .05;
        } else if(operatorController.getLeftBumper()) {
            baseTargetPosition -= .05;
        }

        double basetemp = 0.0;
        basetemp = pidBase.calculate(baseEncoder1.getPosition(), baseTargetPosition) / 100;
        //baseGroup.set(basetemp);

        // High
        if(operatorController.getRawButtonPressed(Constants.OPERATOR_BUTTON_HIGH)) {
            wristTargetPosition = Constants.ARM_HIGH_WRIST_ENCODER_VALUE;
            armTargetPosition = Constants.ARM_HIGH_ENCODER_VALUE;
            baseTargetPosition = Constants.ARM_HIGH_BASE_ENCODER_VALUE;
        }

        // Mid
        if(operatorController.getRawButtonPressed(Constants.OPERATOR_BUTTON_MID)) {
            wristTargetPosition = Constants.ARM_HIGH_WRIST_ENCODER_VALUE;
            armTargetPosition = Constants.ARM_MID_ARM_ENCODER_VALUE;
            baseTargetPosition = Constants.ARM_MID_BASE_ENCODER_VALUE;
        }

        // Fold
        if(operatorController.getRawButtonPressed(Constants.OPERATOR_BUTTON_FOLD)) {
            wristTargetPosition = Constants.ARM_FOLD_WRIST_ENCODER_VALUE;
            armTargetPosition = Constants.ARM_FOLD_ARM_ENCODER_VALUE;
            baseTargetPosition = Constants.ARM_FOLD_BASE_ENCODER_VALUE;
        }

        // Low
        if(operatorController.getRawButtonPressed(Constants.OPERATOR_BUTTON_LOW)) {
            wristTargetPosition = Constants.ARM_LOW_WRIST_ENCODER_VALUE;
            armTargetPosition = Constants.ARM_LOW_ARM_ENCODER_VALUE;
            baseTargetPosition = Constants.ARM_LOW_BASE_ENCODER_VALUE;
        }

        // LowLow
        if(operatorController.getRawButtonPressed(Constants.OPERATOR_BUTTON_LOWLOW)) {
            wristTargetPosition = Constants.ARM_LOWLOW_WRIST_ENCODER_VALUE;
            armTargetPosition = Constants.ARM_LOWLOW_ARM_ENCODER_VALUE;
            baseTargetPosition = Constants.ARM_LOWLOW_BASE_ENCODER_VALUE;
        }

        /*if(operatorController.getRightBumper() && usingPresetSetting != true) {
            baseGroup.set(0.1);
        } else if(operatorController.getLeftBumper() && usingPresetSetting != true) {
            baseGroup.set(-0.1);
        } else if(usingPresetSetting != true) {
            baseGroup.set(0.0);
        }*/
    }

    public void setRevPhysicsSim(REVPhysicsSim sim) {
        this.revPhysicsSim = sim;

        //sim.addSparkMax(baseMotor, DCMotor.getNEO(1));
        //sim.addSparkMax(highMotor, DCMotor.getNEO(1));

        System.out.println("ArmSubsystem::setRevPhysicsSim() called");
        REVPhysicsSim.getInstance().addSparkMax(highMotor1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(baseMotor1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(wristMotor1, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNEO(1));

        /*sim.addSparkMax(
            new CANSparkMax(Constants.ARM_BASE_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless), 
            DCMotor.getNEO(1)
        );*/
    }

}
 