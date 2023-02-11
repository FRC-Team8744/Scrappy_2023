// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.util.function.DoubleSupplier;


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Drivetrain;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class AutonomousCommand extends CommandBase {

    public DoubleSupplier m_steering;
    public DoubleSupplier m_throttle;
    public Double m_DistanceInches;
    public Double m_startpoint;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final Drivetrain m_drivetrain;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public AutonomousCommand(Drivetrain subsystem, double DriveDistance) {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
   
      
        
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_drivetrain = subsystem;
        addRequirements(m_drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    m_DistanceInches = DriveDistance ;
    m_startpoint = 0.0;
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       m_startpoint = m_drivetrain.checkdistanceInches();
        // m_drivetrain.Drive(0.2, 0.0);//throttle, steering
        // if (m_drivetrain.checkdistance() >= 1.0) m_drivetrain.Drive(0.0, 0.2);
        m_drivetrain.Drive(0.0, 0.0);//throttle, steering
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // m_drivetrain.Drive(0.2, 0.0);//throttle, steering
        if (m_DistanceInches < 0) {
            m_drivetrain.Drive(0.0, -0.4);
        }
       else {
             m_drivetrain.Drive(0.0, 0.4);
       }


        // if (m_drivetrain.checkdistance() >= 1.0) m_drivetrain.Drive(0.0, 0.2);
        // m_drivetrain.Drive(0.2, 0.0);//throttle, steering
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.Drive(0.0, 0.0);
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
       if (m_DistanceInches < 0) {
            if (m_drivetrain.checkdistanceInches() <= m_startpoint + m_DistanceInches) return true;
        } else {
            if (m_drivetrain.checkdistanceInches() >= m_startpoint + m_DistanceInches) return true;

        }
        return false;
        // if (m_drivetrain.checkdegrees() >= 90.0) return true;
        // else return false;
    
    }


   
}
