// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
    import com.revrobotics.RelativeEncoder;
    import com.revrobotics.SparkMaxPIDController;

/**
 *
 */
public class SGripper extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    private final boolean DEBUG_PID = false;
    private final boolean DEBUG = false;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private CANSparkMax gripperSpark;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private SparkMaxPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;    
    /**
    *
    */
    public SGripper() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
gripperSpark = new CANSparkMax(13, MotorType.kBrushless);
 
 gripperSpark.restoreFactoryDefaults();  
gripperSpark.setInverted(false);
gripperSpark.setIdleMode(IdleMode.kCoast);
// gripperSpark.burnFlash();
  


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    gripperSpark.setSmartCurrentLimit(10);
    m_pidController = gripperSpark.getPIDController();

    m_encoder = gripperSpark.getEncoder();

    // PID coefficients
    kP = 1;
    kI = 1e-4;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    //set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    if (DEBUG_PID) {
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    }
    
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
         // This method will be called once per scheduler run
         if (DEBUG) {
         SmartDashboard.putNumber("Gripper Encoder", m_encoder.getPosition());
         SmartDashboard.putNumber("GripperOutput", gripperSpark.getAppliedOutput());
         SmartDashboard.putNumber("Scale Factor", m_encoder.getPositionConversionFactor());
         }
         if (DEBUG_PID) {
         // read PID coefficients from SmartDashBoard
         double p = SmartDashboard.getNumber("P Gain", 0);
         double i = SmartDashboard.getNumber("I Gain", 0);
         double d = SmartDashboard.getNumber("D Gain", 1);
         double iz = SmartDashboard.getNumber("I Zone", 0);
         double ff = SmartDashboard.getNumber("Feed Forward", 0);
         double max = SmartDashboard.getNumber("Max Output", 0);
         double min = SmartDashboard.getNumber("Min Output", 0);
 
         // if PID coefficients on SmartDashboard have changed, write new values to controller
         if((p != kP)) {m_pidController.setP(p); kP = p;}
         if((i != kI)) {m_pidController.setI(i); kI = i;}
         if((d != kD)) {m_pidController.setD(d); kD = d;}
         if((iz != kIz)) {m_pidController.setIZone(iz); kIz = iz;}
         if((ff != kFF)) {m_pidController.setFF(ff); kFF = ff;}
         if((max != kMaxOutput) || (min != kMinOutput)) {
             m_pidController.setOutputRange(min, max);
             kMinOutput = min; kMaxOutput = max;
         }
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void SetGripper(double Position) {
        m_pidController.setReference(Position, CANSparkMax.ControlType.kPosition);
        if (DEBUG) {
        SmartDashboard.putNumber("SetPoint", Position);
        }
    }
    public void StopGripper() {
        gripperSpark.stopMotor();
    }
    

}

