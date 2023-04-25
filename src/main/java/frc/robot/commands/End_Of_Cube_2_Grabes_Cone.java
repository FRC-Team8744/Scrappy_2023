// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SGripper;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class End_Of_Cube_2_Grabes_Cone extends ParallelCommandGroup {
  /** Creates a new End_Of_Cube_2_Grabes_Cone. */
  public End_Of_Cube_2_Grabes_Cone(Drivetrain m_drivetrain, SGripper m_sGripper, Arm m_Arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SlowAutonomousCommand(m_drivetrain, 6),
      new Move_Arm(m_Arm, Constants.kHOLD_SERVO_STATE, Constants.kHOLD_SERVO_STATE, 14),
      new Gripper(m_sGripper, -0.5).withTimeout(3)
    );
  }
}
