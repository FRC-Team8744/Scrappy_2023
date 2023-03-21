// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Move_Arm_LowPreset extends SequentialCommandGroup {
  /** Creates a new Move_Arm_LowPreset. */
  public Move_Arm_LowPreset(Arm m_Arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Move_Arm(m_Arm, 25.8, 0.0, 14.0).withInterruptBehavior(InterruptionBehavior.kCancelSelf).unless(() -> (m_Arm.m_Aencoder.getPosition() < 25.8)),
      new Move_Arm(m_Arm, 0.0, 0.0, 14.0).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
  }
}
