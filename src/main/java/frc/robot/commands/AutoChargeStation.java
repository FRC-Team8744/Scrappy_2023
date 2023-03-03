// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.DocFlavor.SERVICE_FORMATTED;

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
public class AutoChargeStation extends SequentialCommandGroup {

  // private final Drivetrain m_drivetrain;
  // private final SWrist m_sWrist;
  // private final SGripper m_sGripper;
  // private final SExtendElevator m_sExtendElevator;
  // private final SLiftArm m_sLiftArm;
  /** Creates a new AutoDrive. */
  public AutoChargeStation(Drivetrain m_drivetrain, SWrist m_sWrist, SExtendElevator m_sExtendElevator, SLiftArm m_sLiftArm, SGripper m_sGripper) {
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
      
      new AutonomousCommand( m_drivetrain, -160.0),
      new WaitCommand(2),
      new AutonomousCommand(m_drivetrain, 102.0)
      );
  }

}
