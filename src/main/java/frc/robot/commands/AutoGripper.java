// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SGripper;

public class AutoGripper extends CommandBase {
  public Double m_grip;

  private final SGripper m_sGripper;
  /** Creates a new AutoGripper. */
  public AutoGripper(SGripper subsystem, double Tightness) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_sGripper = subsystem;
    addRequirements(m_sGripper);

    m_grip = Tightness;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_sGripper.SetGripper( m_grip);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
