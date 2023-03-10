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
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

import com.revrobotics.REVLibError;

import frc.robot.Constants;
import frc.robot.RobotContainer;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.SLiftArm;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class LiftArm extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final SLiftArm m_sLiftArm;
    private double m_Armheight;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public LiftArm(SLiftArm subsystem, double Armheight ) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_Armheight = Armheight;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_sLiftArm = subsystem;
        addRequirements(m_sLiftArm);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_Armheight == Constants.kARM_LIFT) {
            if (RobotContainer.getInstance().joystick1.getRawButtonPressed(6) == true) {
                m_sLiftArm.SetArm(m_sLiftArm.LastArmSetting + Constants.kARM_MOVE_RATE);
            }
        } else if (m_Armheight == Constants.kARM_LOWER) {
            if (RobotContainer.getInstance().joystick1.getRawButtonPressed(4) == true) {
                m_sLiftArm.SetArm(m_sLiftArm.LastArmSetting - Constants.kARM_MOVE_RATE);
            }
        } else {
            m_sLiftArm.SetArm(m_Armheight);
        }
       
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_sLiftArm.StopArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
    
   
}
