// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SExtendElevator;

public class AutoExtendElevator extends CommandBase {


  public Double m_ExtendDistance;
  // public Double m_ElevatorExtention; 
  

  private final SExtendElevator m_sExtendElevator;

  /** Creates a new AutoExtendElevator. */
  public AutoExtendElevator(SExtendElevator subsystem, double Extend) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_sExtendElevator = subsystem;
    addRequirements(m_sExtendElevator);

    m_ExtendDistance = Extend;
   
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
    m_sExtendElevator.SetExtendElevator( m_ExtendDistance);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
