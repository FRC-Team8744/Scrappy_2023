// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Zero_Arm_Encoders extends CommandBase {
      private final Arm m_Arm;
      private final Timer m_Timer = new Timer();
/** Creates a new Zero_Arm_Encoders. */
  public Zero_Arm_Encoders(Arm ArmSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = ArmSub;
    addRequirements(m_Arm);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Timer", m_Timer.get());
  if (m_Timer.get() < 0.5 ){
    m_Arm.PullBackElevator();
  } else if (m_Timer.get() < 1.0) {
    m_Arm.PullBackArm();
  } else m_Arm.ZeroEncoders();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Timer.get() > 1.1) {
      // SmartDashboard.putString("Zero Debug", "Zero Arm Is Finished");
      return true;}
    else return false;

  }
}
