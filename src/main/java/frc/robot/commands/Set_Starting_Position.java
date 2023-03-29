// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Set_Starting_Position extends CommandBase {
  private final Drivetrain m_drivetrain;

  /** Creates a new Set_Starting_Position. */
  public Set_Starting_Position(Drivetrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = subsystem;
    addRequirements(m_drivetrain);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setStartPositionDirection();  // Sets the position (angle) of the robot at the start of the match
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
