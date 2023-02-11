// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Drivetrain;

public class  AutoTurnToAngle extends CommandBase {


    public DoubleSupplier m_steering;
    public DoubleSupplier m_throttle;
    public Double m_TurnAngle;
    public Double m_startangle;

    private final Drivetrain m_drivetrain;
  /** Creates a new AutoTurnToAngle. */
    public AutoTurnToAngle(Drivetrain subsystem, double AngleToTurn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = subsystem;
    addRequirements(m_drivetrain);


m_TurnAngle = AngleToTurn ;
m_startangle = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startangle = m_drivetrain.checkdegrees();
        // m_drivetrain.Drive(0.2, 0.0);//throttle, steering
        // if (m_drivetrain.checkdistance() >= 1.0) m_drivetrain.Drive(0.0, 0.2);
        m_drivetrain.Drive(0.0, 0.0);//throttle, steering
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // m_drivetrain.Drive(0.2, 0.0);//throttle, steering
     if (m_TurnAngle > 0) {
      m_drivetrain.Drive(0.2, 0.0);
  }
 else {
       m_drivetrain.Drive(-0.2, 0.0);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.Drive(0.0, 0.0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_TurnAngle < 0) {
      if (m_drivetrain.checkdegrees() <= m_startangle + m_TurnAngle) return true;
  } else {
      if (m_drivetrain.checkdegrees() >= m_startangle + m_TurnAngle) return true;

  }
  return false;
  // if (m_drivetrain.checkdegrees() >= 90.0) return true;
  // else return false;
  }
  @Override
  public boolean runsWhenDisabled() {
      return false;
  }
}

