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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class Drivetrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_VictorSPX leftFront;
private WPI_VictorSPX leftRear;
private CANSparkMax leftFrontSparkMax;
private CANSparkMax leftRearSparkMax;
private MotorControllerGroup leftMotor;
private WPI_VictorSPX rightFront;
private WPI_VictorSPX rightRear;
private CANSparkMax rightFrontSparkMax;
private CANSparkMax rightRearSparkMax;
private MotorControllerGroup rightMotor;
private DifferentialDrive differentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    public Drivetrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftFront = new WPI_VictorSPX(1);
 
 

leftRear = new WPI_VictorSPX(4);
 
 

leftFrontSparkMax = new CANSparkMax(7, MotorType.kBrushed);
 
 leftFrontSparkMax.restoreFactoryDefaults();  
leftFrontSparkMax.setInverted(false);
leftFrontSparkMax.setIdleMode(IdleMode.kCoast);
leftFrontSparkMax.burnFlash();
  

leftRearSparkMax = new CANSparkMax(8, MotorType.kBrushed);
 
 leftRearSparkMax.restoreFactoryDefaults();  
leftRearSparkMax.setInverted(false);
leftRearSparkMax.setIdleMode(IdleMode.kCoast);
leftRearSparkMax.burnFlash();
  

leftMotor = new MotorControllerGroup(leftFrontSparkMax, leftRearSparkMax  );
 addChild("Left Motor",leftMotor);
 

rightFront = new WPI_VictorSPX(2);
 
 

rightRear = new WPI_VictorSPX(3);
 
 

rightFrontSparkMax = new CANSparkMax(9, MotorType.kBrushed);
 
 rightFrontSparkMax.restoreFactoryDefaults();  
rightFrontSparkMax.setInverted(false);
rightFrontSparkMax.setIdleMode(IdleMode.kCoast);
rightFrontSparkMax.burnFlash();
  

rightRearSparkMax = new CANSparkMax(10, MotorType.kBrushed);
 
 rightRearSparkMax.restoreFactoryDefaults();  
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



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }
    public void Drive(double steering, double throttle) {
        
        differentialDrive.arcadeDrive(steering, throttle);
        SmartDashboard.putNumber("steering", steering);
        SmartDashboard.putNumber("throttle", throttle);
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // private static void TeleopDrive(double steering, double throttle) {
    // }

}

