// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  // private final boolean DEBUG_PID = false;
  private final boolean DEBUG = true;
  private final boolean DEBUG_ENCODER = true;

  public double LastArmSetting = 0;
  public double LastElevatorSetting = 0;
  public double LastWristSetting = 0;

  private CANSparkMax liftArmSpark;
  private CANSparkMax wristSpark;
  private CANSparkMax extendElevatorSpark;

  private SparkMaxPIDController m_ApidController;
  private SparkMaxPIDController m_EpidController;
  private SparkMaxPIDController m_WpidController;

  public RelativeEncoder m_Aencoder;
  public RelativeEncoder m_Eencoder;
  public RelativeEncoder m_Wencoder;

  public double kEP, kEI, kED, kEIz, kEFF, kEMaxOutput, kEMinOutput;
  public double kAP, kAI, kAD, kAIz, kAFF, kAMaxOutput, kAMinOutput; 
  public double kWP, kWI, kWD, kWIz, kWFF, kWMaxOutput, kWMinOutput;    
  public double AmaxVel, AmaxAcc, AminVel, AallowedErr;
  public double WmaxVel, WmaxAcc, WminVel, WallowedErr;
  public double EmaxVel, EmaxAcc, EminVel, EallowedErr;

  public Arm() {
    liftArmSpark = new CANSparkMax(11, MotorType.kBrushless);
    extendElevatorSpark = new CANSparkMax(12, MotorType.kBrushless);
    wristSpark = new CANSparkMax(14, MotorType.kBrushless);
 
    liftArmSpark.restoreFactoryDefaults();  
    liftArmSpark.setInverted(false);
    liftArmSpark.setIdleMode(IdleMode.kBrake);
    liftArmSpark.setSmartCurrentLimit(80);
    m_ApidController = liftArmSpark.getPIDController();
    m_Aencoder = liftArmSpark.getEncoder();

    extendElevatorSpark.restoreFactoryDefaults();  
    extendElevatorSpark.setInverted(false);
    extendElevatorSpark.setIdleMode(IdleMode.kBrake);
    extendElevatorSpark.setSmartCurrentLimit(20);
    m_EpidController = extendElevatorSpark.getPIDController();
    m_Eencoder = extendElevatorSpark.getEncoder();

    wristSpark.restoreFactoryDefaults();  
    wristSpark.setInverted(false);
    wristSpark.setIdleMode(IdleMode.kBrake);
    wristSpark.setSmartCurrentLimit(60);
    m_WpidController = wristSpark.getPIDController();
    m_Wencoder = wristSpark.getEncoder();

    kEP = 0.00007;
    // kEP = 0.7;
    kEI = 0;
    kED = 0;
    kEIz = 0;
    kEFF = 0;
    kEMaxOutput = 1;
    kEMinOutput = -1;

    m_EpidController.setP(kEP, 0);
    m_EpidController.setI(kEI, 0);
    m_EpidController.setD(kED, 0);
    m_EpidController.setIZone(kEIz, 0);
    m_EpidController.setFF(kEFF, 0);
    m_EpidController.setOutputRange(kEMinOutput, kEMaxOutput, 0);

    // Smart Motion Coefficients
    EmaxVel = 10000; // rpm
    EmaxAcc = 200000;
    EminVel = 0;
    EallowedErr = 1.0;
  
    // // Set SmartMotion parameters
    m_EpidController.setSmartMotionMaxVelocity(EmaxVel, 0);
    m_EpidController.setSmartMotionMinOutputVelocity(EminVel, 0);
    m_EpidController.setSmartMotionMaxAccel(EmaxAcc, 0);
    m_EpidController.setSmartMotionAllowedClosedLoopError(EallowedErr, 0);

    // !!!!! START: Control coefficients for Slot 1 ONLY!!!
    kEP = 0.7;
    kEI = 0;
    kED = 0;
    kEIz = 0;
    kEFF = 0;
    kEMaxOutput = 1;
    kEMinOutput = -1;

    m_EpidController.setP(kEP, 1);
    m_EpidController.setI(kEI, 1);
    m_EpidController.setD(kED, 1);
    m_EpidController.setIZone(kEIz, 1);
    m_EpidController.setFF(kEFF, 1);
    m_EpidController.setOutputRange(kEMinOutput, kEMaxOutput, 1);
    // !!!!! END: Control coefficients for Slot 1 ONLY!!!

    kAP = 0.00007;
    // kAP = 0.7;
    kAI = 0;
    kAD = 0;
    kAIz = 0;
    kAFF = 0;
    kAMaxOutput = 1;
    kAMinOutput = -1;
    
    m_ApidController.setP(kAP);
    m_ApidController.setI(kAI);
    m_ApidController.setD(kAD);
    m_ApidController.setIZone(kAIz);
    m_ApidController.setFF(kAFF);
    m_ApidController.setOutputRange(kAMinOutput, kAMaxOutput);

    // Smart Motion Coefficients
    AmaxVel = 5000; // rpm
    AmaxAcc = 60000;
    AminVel = 0;
    AallowedErr = 1.0;
  
    // Set SmartMotion parameters
    m_ApidController.setSmartMotionMaxVelocity(AmaxVel, 0);
    m_ApidController.setSmartMotionMinOutputVelocity(AminVel, 0);
    m_ApidController.setSmartMotionMaxAccel(AmaxAcc, 0);
    m_ApidController.setSmartMotionAllowedClosedLoopError(AallowedErr, 0);
    
    // !!!!! START: Control coefficients for Slot 1 ONLY!!!
    kAP = 0.7;
    kAI = 0;
    kAD = 0;
    kAIz = 0;
    kAFF = 0;
    kAMaxOutput = 1;
    kAMinOutput = -1;

    m_ApidController.setP(kAP, 1);
    m_ApidController.setI(kAI, 1);
    m_ApidController.setD(kAD, 1);
    m_ApidController.setIZone(kAIz, 1);
    m_ApidController.setFF(kAFF, 1);
    m_ApidController.setOutputRange(kAMinOutput, kAMaxOutput, 1);
    // !!!!! END: Control coefficients for Slot 1 ONLY!!!

    kWP = 0.00007;
    // kWP = 0.7;
    kWI = 0;
    kWD = 0;
    kWIz = 0;
    kWFF = 0;
    kWMaxOutput = 1;
    kWMinOutput = -1;
    
    m_WpidController.setP(kWP);
    m_WpidController.setI(kWI);
    m_WpidController.setD(kWD);
    m_WpidController.setIZone(kWIz);
    m_WpidController.setFF(kWFF);
    m_WpidController.setOutputRange(kWMinOutput, kWMaxOutput);

    // Smart Motion Coefficients
    WmaxVel = 7000; // rpm
    WmaxAcc = 100000;
    WminVel = 0;
    WallowedErr = 1.0;
  
    // Set SmartMotion parameters
    m_WpidController.setSmartMotionMaxVelocity(WmaxVel, 0);
    m_WpidController.setSmartMotionMinOutputVelocity(WminVel, 0);
    m_WpidController.setSmartMotionMaxAccel(WmaxAcc, 0);
    m_WpidController.setSmartMotionAllowedClosedLoopError(WallowedErr, 0);
    
    // !!!!! START: Control coefficients for Slot 1 ONLY!!!
    kWP = 0.7;
    kWI = 0;
    kWD = 0;
    kWIz = 0;
    kWFF = 0;
    kWMaxOutput = 1;
    kWMinOutput = -1;

    m_WpidController.setP(kWP, 1);
    m_WpidController.setI(kWI, 1);
    m_WpidController.setD(kWD, 1);
    m_WpidController.setIZone(kWIz, 1);
    m_WpidController.setFF(kWFF, 1);
    m_WpidController.setOutputRange(kWMinOutput, kWMaxOutput, 1);
    // !!!!! END: Control coefficients for Slot 1 ONLY!!!

    // Enable motor and set position to zero
    // m_pidController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG_ENCODER) {
      SmartDashboard.putNumber("LiftArm Encoder", m_Aencoder.getPosition());
      SmartDashboard.putNumber("Elevator Encoder", m_Eencoder.getPosition());
      SmartDashboard.putNumber("Wrist Encoder", m_Wencoder.getPosition());
    }
  }

  public void SetArm(double Arm_Position, double Elevator_Position, double Wrist_Position) {
    // AmaxVel = SmartDashboard.getNumber("maxVel", 4000);
    // AmaxAcc = SmartDashboard.getNumber("MaxAcc", 10000);
    // AminVel = SmartDashboard.getNumber("MinVel", 0);
    // AallowedErr = SmartDashboard.getNumber("AllowedErr", 0);

    // Set SmartMotion parameters
    // m_ApidController.setSmartMotionMaxVelocity(AmaxVel, 0);
    // m_ApidController.setSmartMotionMinOutputVelocity(AminVel, 0);
    // m_ApidController.setSmartMotionMaxAccel(AmaxAcc, 0);
    // m_ApidController.setSmartMotionAllowedClosedLoopError(AallowedErr, 0);
    
    // EmaxVel = SmartDashboard.getNumber("EmaxVel", 4000);
    // EmaxAcc = SmartDashboard.getNumber("EMaxAcc", 10000);
    // EminVel = SmartDashboard.getNumber("EMinVel", 0);
    // EallowedErr = SmartDashboard.getNumber("EAllowedErr", 0);

    // Set SmartMotion parameters
    // m_ApidController.setSmartMotionMaxVelocity(EmaxVel, 0);
    // m_ApidController.setSmartMotionMinOutputVelocity(EminVel, 0);
    // m_ApidController.setSmartMotionMaxAccel(EmaxAcc, 0);
    // m_ApidController.setSmartMotionAllowedClosedLoopError(EallowedErr, 0);
    
    // WmaxVel = SmartDashboard.getNumber("WmaxVel", 4000);
    // WmaxAcc = SmartDashboard.getNumber("WMaxAcc", 10000);
    // WminVel = SmartDashboard.getNumber("WMinVel", 0);
    // WallowedErr = SmartDashboard.getNumber("WAllowedErr", 0);

    // Set SmartMotion parameters
    // m_ApidController.setSmartMotionMaxVelocity(WmaxVel, 0);
    // m_ApidController.setSmartMotionMinOutputVelocity(WminVel, 0);
    // m_ApidController.setSmartMotionMaxAccel(WmaxAcc, 0);
    // m_ApidController.setSmartMotionAllowedClosedLoopError(WallowedErr, 0);
    
    
    // Skip servo position update if user passes in the hold state flag
    if (Arm_Position != Constants.kHOLD_SERVO_STATE) {
      // m_ApidController.setReference(Arm_Position, CANSparkMax.ControlType.kPosition, 0);
      m_ApidController.setReference(Arm_Position, CANSparkMax.ControlType.kSmartMotion, 0);
      LastArmSetting = Arm_Position;
    } else {
      // m_ApidController.setReference(LastArmSetting, CANSparkMax.ControlType.kPosition, 0);
      m_ApidController.setReference(LastArmSetting, CANSparkMax.ControlType.kSmartMotion, 0);
    }

    if (Elevator_Position != Constants.kHOLD_SERVO_STATE) {
      // m_EpidController.setReference(Elevator_Position, CANSparkMax.ControlType.kPosition, 0);
      m_EpidController.setReference(Elevator_Position, CANSparkMax.ControlType.kSmartMotion, 0);
      LastElevatorSetting = Elevator_Position;
    } else {
      // m_EpidController.setReference(LastElevatorSetting, CANSparkMax.ControlType.kPosition, 0);
      m_EpidController.setReference(LastElevatorSetting, CANSparkMax.ControlType.kSmartMotion, 0);
    }

    if (Wrist_Position != Constants.kHOLD_SERVO_STATE) {
      // m_WpidController.setReference(Wrist_Position, CANSparkMax.ControlType.kPosition, 0);
      m_WpidController.setReference(Wrist_Position, CANSparkMax.ControlType.kSmartMotion, 0);
      LastWristSetting = Wrist_Position;
    } else {
      // m_WpidController.setReference(LastWristSetting, CANSparkMax.ControlType.kPosition, 0);
      m_WpidController.setReference(LastWristSetting, CANSparkMax.ControlType.kSmartMotion, 0);
    }

    if (DEBUG) {
      // SmartDashboard.putNumber("SetPoint Arm", Arm_Position);
      // SmartDashboard.putNumber("SetPoint Elevator", Elevator_Position);
      // SmartDashboard.putNumber("SetPoint Wrist", Wrist_Position);
      // SmartDashboard.putNumber("LastArmSetting", LastArmSetting);
      // SmartDashboard.putNumber("LastElevatorSetting", LastElevatorSetting);
      // SmartDashboard.putNumber("LastWristSetting", LastWristSetting);
      SmartDashboard.putNumber("Temp Arm", liftArmSpark.getMotorTemperature());
      SmartDashboard.putNumber("Temp Elevator",  extendElevatorSpark.getMotorTemperature());
      SmartDashboard.putNumber("Temp Wrist",  wristSpark.getMotorTemperature());
      SmartDashboard.putNumber("Current Arm", liftArmSpark.getOutputCurrent());
      SmartDashboard.putNumber("Current WRist", wristSpark.getOutputCurrent());
      SmartDashboard.putNumber("Current Elevator", extendElevatorSpark.getOutputCurrent());
    }
  }

  public void StopArm() {
    liftArmSpark.stopMotor();
    extendElevatorSpark.stopMotor();
    wristSpark.stopMotor();

  }

  public void PullBackElevator() {
    m_EpidController.setReference(0.1, CANSparkMax.ControlType.kDutyCycle, 1);
    m_WpidController.setReference(3.0, CANSparkMax.ControlType.kPosition, 1);
    // wristSpark.stopMotor();
    
  }
  public void PullBackArm() {
    m_EpidController.setReference(0.1, CANSparkMax.ControlType.kDutyCycle, 1);
    m_WpidController.setReference(-0.2, CANSparkMax.ControlType.kDutyCycle, 1);
    // wristSpark.stopMotor();
    
  }

  public void ZeroEncoders() {
    
    StopArm();// YOU NEED TO TURN OF THE MOTORS BEFORE YOU SET THE POSITION!!!
    m_Aencoder.setPosition(0.0);
    m_Eencoder.setPosition(1.0);
    m_Wencoder.setPosition(-3.0);
    StopArm();
    
  }

  public double getArmPosition() {
    return m_Aencoder.getPosition();
  }

  public double getElevatorPosition() {
    return m_Eencoder.getPosition();
  }

  public double getWristPosition() {
    return m_Wencoder.getPosition();
  }

  public void HoldArm() {
    // Set servo hold position for all axies (Using control settings in SLOT1)
    m_ApidController.setReference(LastArmSetting, CANSparkMax.ControlType.kPosition, 1);

    m_EpidController.setReference(LastElevatorSetting, CANSparkMax.ControlType.kPosition, 1);

    m_WpidController.setReference(LastWristSetting, CANSparkMax.ControlType.kPosition, 1);

    if (DEBUG) {
      SmartDashboard.putNumber("LastArmSetting", LastArmSetting);
      SmartDashboard.putNumber("LastElevatorSetting", LastElevatorSetting);
      SmartDashboard.putNumber("LastWristSetting", LastWristSetting);
      SmartDashboard.putNumber("Temp Arm", liftArmSpark.getMotorTemperature());
      SmartDashboard.putNumber("Temp Elevator",  extendElevatorSpark.getMotorTemperature());
      SmartDashboard.putNumber("Temp Wrist",  wristSpark.getMotorTemperature());
      SmartDashboard.putNumber("Current Arm", liftArmSpark.getOutputCurrent());
      SmartDashboard.putNumber("Current WRist", wristSpark.getOutputCurrent());
      SmartDashboard.putNumber("Current Elevator", extendElevatorSpark.getOutputCurrent());
    }
  }

}
