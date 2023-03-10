// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SExtendElevator;
import frc.robot.subsystems.SGripper;
import frc.robot.subsystems.SLiftArm;
import frc.robot.subsystems.SWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NudgeArm extends ParallelCommandGroup {
  /** Creates a new NudgeArm. */
  private double ArmPosition, ElevatorPosition, WristPosition;

  public NudgeArm(int NudgeType, SWrist m_sWrist, SExtendElevator m_sExtendElevator, SLiftArm m_sLiftArm, SGripper m_sGripper) {
    // ArmPosition = RobotContainer.getInstance().GlobalLastArmSetting;
    // ElevatorPosition = RobotContainer.getInstance().GlobalLastElevatorSetting;
    // WristPosition = RobotContainer.getInstance().GlobalLastWristSetting;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LiftArm(m_sLiftArm, ArmPosition).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new ExtendElevator(m_sExtendElevator, ElevatorPosition).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new Wrist(m_sWrist, -7).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
  }
}
