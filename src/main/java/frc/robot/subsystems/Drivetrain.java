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


// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer; import edu.wpi.first.wpilibj.DriverStation; import com.kauailabs.navx.frc.AHRS; import edu.wpi.first.wpilibj.SPI; import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;


/**
 *
 */
public class Drivetrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    private final boolean DEBUG = false;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private CANSparkMax leftFrontSparkMax;
private CANSparkMax leftRearSparkMax;
private MotorControllerGroup leftMotor;
private CANSparkMax rightFrontSparkMax;
private CANSparkMax rightRearSparkMax;
private MotorControllerGroup rightMotor;
private DifferentialDrive differentialDrive;
private AHRS navigator;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = .319181581324 * 2; // meters
  private static final double kWheelDiameter = (6 /*diam, inches*/ * .0254); // meters
  public static final double kWheelCircumfrence = kWheelDiameter * Math.PI; // meters
  private static final double GearRatio = 8.5;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;
    
    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  
    private final DifferentialDriveKinematics m_kinematics =
        new DifferentialDriveKinematics(kTrackWidth);
  
    public final DifferentialDriveOdometry m_odometry;
    
    
    
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    private final DifferentialDriveWheelSpeeds DebugSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

    private double MeasuredDistance;
    private double MeasuredDegrees;

    /**
    *
    */
    public Drivetrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftFrontSparkMax = new CANSparkMax(9, MotorType.kBrushless);
 
 leftFrontSparkMax.restoreFactoryDefaults();  
 leftFrontSparkMax.setSmartCurrentLimit(80);
leftFrontSparkMax.setInverted(false);
leftFrontSparkMax.setIdleMode(IdleMode.kCoast);
leftFrontSparkMax.burnFlash();
  

leftRearSparkMax = new CANSparkMax(10, MotorType.kBrushless);
 
 leftRearSparkMax.restoreFactoryDefaults();  
 leftRearSparkMax.setSmartCurrentLimit(80);
leftRearSparkMax.setInverted(false);
leftRearSparkMax.setIdleMode(IdleMode.kCoast);
leftRearSparkMax.burnFlash();
  

leftMotor = new MotorControllerGroup(leftFrontSparkMax, leftRearSparkMax  );
 addChild("Left Motor",leftMotor);
 

rightFrontSparkMax = new CANSparkMax(8, MotorType.kBrushless);
 
 rightFrontSparkMax.restoreFactoryDefaults();  
 rightFrontSparkMax.setSmartCurrentLimit(80);
rightFrontSparkMax.setInverted(false);
rightFrontSparkMax.setIdleMode(IdleMode.kCoast);
rightFrontSparkMax.burnFlash();
  

rightRearSparkMax = new CANSparkMax(7, MotorType.kBrushless);
 
 rightRearSparkMax.restoreFactoryDefaults();  
 rightRearSparkMax.setSmartCurrentLimit(80);
rightRearSparkMax.setInverted(false);
rightRearSparkMax.setIdleMode(IdleMode.kCoast);
rightRearSparkMax.burnFlash();
  

rightMotor = new MotorControllerGroup(rightFrontSparkMax, rightRearSparkMax  );
 addChild("Right Motor",rightMotor);
 

differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
 addChild("Differential Drive",differentialDrive);
 differentialDrive.setSafetyEnabled(true);
differentialDrive.setExpiration(0.1);
differentialDrive.setMaxOutput(1.0);
differentialDrive.setDeadband(0.1);


try { navigator = new AHRS(Port.kUSB);} catch (RuntimeException ex ) {DriverStation.reportError( ex.getMessage(), true);} Timer.delay(1.0);
//  LiveWindow.addSensor("Drivetrain", "Navigator", navigator);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    navigator.reset();
     // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    m_leftEncoder = leftFrontSparkMax.getEncoder();
    m_rightEncoder = rightFrontSparkMax.getEncoder();
    // m_rightEncoder.setInverted(true); // Crashed program!

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    // m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_leftEncoder.setPositionConversionFactor(1.0);
    m_rightEncoder.setPositionConversionFactor(1.0);

    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
    //resets Encoders

    m_odometry =
    new DifferentialDriveOdometry(
        navigator.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (DEBUG == true) {
          SmartDashboard.putNumber( "GyroAngle", navigator.getAngle());
          SmartDashboard.putNumber( "Navigator", navigator.getPitch());
          SmartDashboard.putNumber( "leftEncoder", m_leftEncoder.getPosition());
          SmartDashboard.putNumber( "rightEncoder", m_rightEncoder.getPosition());
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }
    public void Drive(double steering, double throttle) {
        
        differentialDrive.arcadeDrive(throttle, steering);
        // SmartDashboard.putNumber("steering", steering);
        // SmartDashboard.putNumber("throttle", throttle);
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

 /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // final double leftFeedforward = m_feedforward.calculate(speeds.leftInchesPerSecond);
    // final double rightFeedforward = m_feedforward.calculate(speeds.rightInchesPerSecond);

    // final double leftOutput =
    //     m_leftPIDController.calculate(m_leftEncoder.getVelocity(), speeds.leftInchesPerSecond);
    // final double rightOutput =
    //     m_rightPIDController.calculate(m_rightEncoder.getVelocity(), speeds.rightInchesPerSecond);

    // leftMotor.setVoltage(leftOutput + leftFeedforward);
    // rightMotor.setVoltage(rightOutput + rightFeedforward);
    leftMotor.set(speeds.leftMetersPerSecond);
    rightMotor.set(speeds.rightMetersPerSecond);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    // var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));

    DebugSpeeds.leftMetersPerSecond = 0.1;
    DebugSpeeds.rightMetersPerSecond = 0.1;

    // setSpeeds(wheelSpeeds);
    setSpeeds(DebugSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    
    // m_odometry.update(
    //     navigator.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    
  }
  public double checkdistanceMeters() {

    MeasuredDistance = kWheelCircumfrence/GearRatio*m_leftEncoder.getPosition();
    // SmartDashboard.putNumber("MeasuredDistanceInches", MeasuredDistance / .0254);
    return MeasuredDistance;
  }

  public double checkdistanceInches() {

    MeasuredDistance = kWheelCircumfrence/GearRatio*m_leftEncoder.getPosition() / .0254;
    // SmartDashboard.putNumber("MeasuredDistanceInches", MeasuredDistance);
    return MeasuredDistance;
  }
  public double checkdegrees(){
    MeasuredDegrees = navigator.getAngle();
    // SmartDashboard.putNumber("MeasuredDegrees", MeasuredDegrees);
    return MeasuredDegrees;
  }

  // public double checkdegrees() {
    // return navigator.getRotation2d();
  // }
}

