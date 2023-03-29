// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SExtendElevator;
import frc.robot.subsystems.SGripper;
import frc.robot.subsystems.SLiftArm;
import frc.robot.subsystems.SWrist;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShort extends SequentialCommandGroup {
  /** Creates a new AutoShort. */
  public AutoShort(Drivetrain m_drivetrain, Arm m_Arm, SGripper m_sGripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new LiftArm(m_sLiftArm, 62),
      // new ExtendElevator(m_sExtendElevator, -86),
      // new Wrist(m_sWrist, -26),
      // new Gripper(m_sGripper, -20), 
      // new Wrist(m_sWrist, 0),
      // new ExtendElevator(m_sExtendElevator, -2),
      // new LiftArm(m_sLiftArm, 0),
      new Gripper(m_sGripper, 0.5).withTimeout(1.0), 

      new AutonomousCommand( m_drivetrain, -168.0),
      new WaitCommand(2)
      // new AutonomousCommand(m_drivetrain, 84.0)
    );
  }
}
