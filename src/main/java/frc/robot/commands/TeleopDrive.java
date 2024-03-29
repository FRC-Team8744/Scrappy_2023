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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Drivetrain;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class TeleopDrive extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final Drivetrain m_drivetrain;
    private DoubleSupplier m_steering;
    private DoubleSupplier m_throttle;
    private DoubleSupplier m_Limiter;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private final boolean DEBUG = false;

private Double limitvalue;
private Double steeringvalue;
private Double throttlevalue;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public TeleopDrive(DoubleSupplier steering, DoubleSupplier throttle, DoubleSupplier Limiter, Drivetrain subsystem) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_steering = steering;
        m_throttle = throttle;
        m_Limiter = Limiter;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
    limitvalue = 0.0;
    steeringvalue = 0.0;
    throttlevalue = 0.0;
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_drivetrain = subsystem;
        addRequirements(m_drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drivetrain.updateOdometry();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.updateOdometry();
        // limitvalue=(-m_Limiter.getAsDouble()/2)+0.5;
        limitvalue=(m_Limiter.getAsDouble())-1.0;
        steeringvalue = (.75*limitvalue*m_steering.getAsDouble());
        throttlevalue = (limitvalue*m_throttle.getAsDouble());
        if ((Math.abs(steeringvalue) < 0.1) && (Math.abs(throttlevalue) < 0.1)) {
            m_drivetrain.Drive(0, 0);
            m_drivetrain.BrakeNow();
        } else {
            m_drivetrain.Drive(steeringvalue, throttlevalue);
        }

        if (DEBUG == true) {
            SmartDashboard.putNumber("limiter value", m_Limiter.getAsDouble());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       m_drivetrain.Drive(0.0,0.0);
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
