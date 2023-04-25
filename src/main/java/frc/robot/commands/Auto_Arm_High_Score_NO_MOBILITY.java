// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SExtendElevator;
import frc.robot.subsystems.SGripper;
import frc.robot.subsystems.SLiftArm;
import frc.robot.subsystems.SWrist;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Arm_High_Score_NO_MOBILITY extends SequentialCommandGroup {
  /** Creates a new Auto_Arm_High_Score. */
  public Auto_Arm_High_Score_NO_MOBILITY(Drivetrain m_drivetrain, Arm m_Arm, SGripper m_sGripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new AutoZeroSequence(m_Arm),
      new Move_Arm_HighPreset(m_Arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new Gripper(m_sGripper, -0.5).withInterruptBehavior(InterruptionBehavior.kCancelSelf).withTimeout(.25),
      new Move_Arm(m_Arm, Constants.kHOLD_SERVO_STATE, Constants.kHOLD_SERVO_STATE, 10).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new Move_Arm(m_Arm, Constants.kHOLD_SERVO_STATE, 0, 10).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new Move_Arm(m_Arm, 0, 0, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
      new AutonomousCommand( m_drivetrain, -88.0)
      // new WaitCommand(2),
      // new AutonomousCommand(m_drivetrain, 86.0)
      
    );
  }
}
