// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.border.LineBorder;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SExtendElevator;
import frc.robot.subsystems.SGripper;
import frc.robot.subsystems.SLiftArm;
import frc.robot.subsystems.SWrist;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ARMHigh extends ParallelCommandGroup {
  /** Creates a new ARMHigh. */
  public ARMHigh(SWrist m_sWrist, SExtendElevator m_sExtendElevator, SLiftArm m_sLiftArm, SGripper m_sGripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new Wrist(m_sWrist, -30).withTimeout(0.5),
      new LIFTSequence( m_sLiftArm).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new ExtendElevator(m_sExtendElevator, -86).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new Wrist(m_sWrist, -26).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
  }
}
