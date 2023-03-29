// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoZeroSequence extends SequentialCommandGroup {
  /** Creates a new AutoZeroSequence. */
  public AutoZeroSequence(Arm m_Arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Move_Arm(m_Arm, 0, 3, -8.1).withInterruptBehavior(InterruptionBehavior.kCancelSelf).withTimeout(2),
      new Zero_Arm_Encoders(m_Arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new Move_Arm(m_Arm, 0, 0, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf)

    );
  }
}
