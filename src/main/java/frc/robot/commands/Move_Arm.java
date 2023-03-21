// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class Move_Arm extends CommandBase {
  /** Creates a new Move_Arm. */
  private final Arm m_Arm;
  private double m_ArmPosition;
  private double m_ElevatorPosition;
  private double m_WristPosition;

  public Move_Arm(Arm ArmSub, double Arm_Position, double Elevator_Position, double Wrist_Position) {
    m_ArmPosition = Arm_Position;
    m_ElevatorPosition = Elevator_Position;
    m_WristPosition = Wrist_Position;

    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = ArmSub;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putString("Arm State", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Only adjust the lift arm position
    if (m_ArmPosition == Constants.kARM_LIFT) {
      if (RobotContainer.getInstance().gamePad.getRawButtonPressed(4) == true) {
          m_Arm.SetArm(m_Arm.LastArmSetting + Constants.kARM_MOVE_RATE, Constants.kHOLD_SERVO_STATE, Constants.kHOLD_SERVO_STATE);
      }
    } else if (m_ArmPosition == Constants.kARM_LOWER) {
      if (RobotContainer.getInstance().gamePad.getRawButtonPressed(1) == true) {
          m_Arm.SetArm(m_Arm.LastArmSetting - Constants.kARM_MOVE_RATE, Constants.kHOLD_SERVO_STATE, Constants.kHOLD_SERVO_STATE);
      }
    // Only adjust the elevator position
    } else if (m_ElevatorPosition == Constants.kElevator_Nudge_UP) {
      if (RobotContainer.getInstance().gamePad.getRawButtonPressed(7) == true) {
        m_Arm.SetArm(Constants.kHOLD_SERVO_STATE, m_Arm.LastElevatorSetting + Constants.kElevator_MOVE_RATE, Constants.kHOLD_SERVO_STATE);
      }
    } else if (m_ElevatorPosition == Constants.kElevator_Nudge_Down) {
      if (RobotContainer.getInstance().gamePad.getRawButtonPressed(8) == true) {
        m_Arm.SetArm(Constants.kHOLD_SERVO_STATE, m_Arm.LastElevatorSetting - Constants.kElevator_MOVE_RATE, Constants.kHOLD_SERVO_STATE);
      }
    // Only adjust the elevator position
    } else if (m_WristPosition == Constants.kWrist_Tilt_UP) {
      if (RobotContainer.getInstance().gamePad.getRawButtonPressed(3) == true) {
        m_Arm.SetArm(Constants.kHOLD_SERVO_STATE, Constants.kHOLD_SERVO_STATE, m_Arm.LastWristSetting - Constants.kWRIST_MOVE_RATE);
      }
    } else if (m_WristPosition == Constants.kWrist_Tilt_DOWN) {
      if (RobotContainer.getInstance().gamePad.getRawButtonPressed(2) == true) {
        m_Arm.SetArm(Constants.kHOLD_SERVO_STATE, Constants.kHOLD_SERVO_STATE, m_Arm.LastWristSetting + Constants.kWRIST_MOVE_RATE);
      }
    } else {
      // Set all arm joints normally
        m_Arm.SetArm(m_ArmPosition, m_ElevatorPosition, m_WristPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Math.abs(m_Arm.m_Aencoder.getPosition() - m_Arm.LastArmSetting) < 1.0) &&
    (Math.abs(m_Arm.m_Eencoder.getPosition() - m_Arm.LastElevatorSetting) < 1.0) &&
    (Math.abs(m_Arm.m_Wencoder.getPosition() - m_Arm.LastWristSetting) < 1.0)) {
      // SmartDashboard.putString("Arm State", "ArmDone!");
      return true;
    } else return false;
  }
}
