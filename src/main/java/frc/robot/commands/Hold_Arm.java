// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Hold_Arm extends CommandBase {
  /** Creates a new Hold_Arm. */
  private final Arm m_Arm;

  public Hold_Arm(Arm ArmSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = ArmSub;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.HoldArm();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
