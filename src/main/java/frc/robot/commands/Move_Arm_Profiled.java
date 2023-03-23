// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class Move_Arm_Profiled extends CommandBase {
  // Creates a new set of trapezoidal motion profile constraints
  public double MaxLiftVelocity = 100.0; // Max velocity in revolutions per second
  public double MaxLiftAccel = 30.0; // Max acceleration in revolutions per second squared
  private TrapezoidProfile.Constraints m_lift_constraints = new TrapezoidProfile.Constraints(MaxLiftVelocity, MaxLiftAccel);

  private TrapezoidProfile.State m_lift_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_lift_start = new TrapezoidProfile.State();

  private TrapezoidProfile m_lift_profile;

  public double MaxElevatorVelocity = 100.0; // Max velocity in revolutions per second
  public double MaxElevatorAccel = 30.0; // Max acceleration in revolutions per second squared
  private TrapezoidProfile.Constraints m_elevator_constraints = new TrapezoidProfile.Constraints(MaxElevatorVelocity, MaxElevatorAccel);

  private TrapezoidProfile.State m_elevator_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_elevator_start = new TrapezoidProfile.State();

  private TrapezoidProfile m_elevator_profile;


  public double MaxWristVelocity = 100.0; // Max velocity in revolutions per second
  public double MaxWristAccel = 50.0; // Max acceleration in revolutions per second squared
  private TrapezoidProfile.Constraints m_wrist_constraints = new TrapezoidProfile.Constraints(MaxWristVelocity, MaxWristAccel);

  private TrapezoidProfile.State m_wrist_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_wrist_start = new TrapezoidProfile.State();

  private TrapezoidProfile m_wrist_profile;


  private static double TimeStep = 0.02; // 20ms per scheduler run
  private double ClockTick = 0;

  private final Arm m_Arm;
  private double m_ArmPosition;
  private double m_ElevatorPosition;
  private double m_WristPosition;


  /** Creates a new Move_Arm_Profiled. */
  public Move_Arm_Profiled(Arm ArmSub, double Arm_Position, double Elevator_Position, double Wrist_Position) {
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
    // START DEBUG: Get limits from Shuffleboard
    // MaxLiftVelocity = SmartDashboard.getNumber("MaxLiftVelocity", 100.0);
    // MaxLiftAccel = SmartDashboard.getNumber("MaxLiftAccel", 30.0);
    // // Required to get text boxes in the window if not there already
    // SmartDashboard.putNumber("MaxLiftVelocity", MaxLiftVelocity);
    // SmartDashboard.putNumber("MaxLiftAccel", MaxLiftAccel);
    // m_lift_constraints = new TrapezoidProfile.Constraints(MaxLiftVelocity, MaxLiftAccel);
    // END DEBUG: Get limits from Shuffleboard

    // Profile states contain both position and velocity!
    if (m_ArmPosition == Constants.kHOLD_SERVO_STATE) {
      m_lift_goal = new TrapezoidProfile.State(m_Arm.LastArmSetting, 0);
    } else {
      m_lift_goal = new TrapezoidProfile.State(m_ArmPosition, 0);
    }
    m_lift_start = new TrapezoidProfile.State(m_Arm.getArmPosition(), 0);

    // Create a motion profile limited by the constraints.
    m_lift_profile = new TrapezoidProfile(m_lift_constraints, m_lift_goal, m_lift_start);

    // START DEBUG: Get limits from Shuffleboard
    // MaxElevatorVelocity = SmartDashboard.getNumber("MaxElevatorVelocity", 100.0);
    // MaxElevatorAccel = SmartDashboard.getNumber("MaxElevatorAccel", 30.0);
    // // Required to get text boxes in the window if not there already
    // SmartDashboard.putNumber("MaxElevatorVelocity", MaxElevatorVelocity);
    // SmartDashboard.putNumber("MaxElevatorAccel", MaxElevatorAccel);
    // m_elevator_constraints = new TrapezoidProfile.Constraints(MaxElevatorVelocity, MaxElevatorAccel);
    // END DEBUG: Get limits from Shuffleboard

    // Profile states contain both position and velocity!
    if (m_ElevatorPosition == Constants.kHOLD_SERVO_STATE) {
      m_elevator_goal = new TrapezoidProfile.State(m_Arm.LastElevatorSetting, 0);
    } else {
      m_elevator_goal = new TrapezoidProfile.State(m_ElevatorPosition, 0);
    }
    m_elevator_start = new TrapezoidProfile.State(m_Arm.getElevatorPosition(), 0);

    // Create a motion profile limited by the constraints.
    m_elevator_profile = new TrapezoidProfile(m_elevator_constraints, m_elevator_goal, m_elevator_start);

    // START DEBUG: Get limits from Shuffleboard
    // MaxWristVelocity = SmartDashboard.getNumber("MaxWristVelocity", 100.0);
    // MaxWristAccel = SmartDashboard.getNumber("MaxWristAccel", 30.0);
    // // Required to get text boxes in the window if not there already
    // SmartDashboard.putNumber("MaxWristVelocity", MaxWristVelocity);
    // SmartDashboard.putNumber("MaxWristAccel", MaxWristAccel);
    // m_wrist_constraints = new TrapezoidProfile.Constraints(MaxWristVelocity, MaxWristAccel);
    // END DEBUG: Get limits from Shuffleboard

    // Profile states contain both position and velocity!
    if (m_WristPosition == Constants.kHOLD_SERVO_STATE) {
      m_wrist_goal = new TrapezoidProfile.State(m_Arm.LastWristSetting, 0);
    } else {
      m_wrist_goal = new TrapezoidProfile.State(m_WristPosition, 0);
    }
    m_wrist_start = new TrapezoidProfile.State(m_Arm.getWristPosition(), 0);

    // Create a motion profile limited by the constraints.
    m_wrist_profile = new TrapezoidProfile(m_wrist_constraints, m_wrist_goal, m_wrist_start);

    ClockTick = 0;
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.SetArm(m_lift_profile.calculate(ClockTick).position, m_elevator_profile.calculate(ClockTick).position, m_wrist_profile.calculate(ClockTick).position);

    ClockTick = ClockTick + TimeStep;  // Advance time for next calculation
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.StopArm();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if ((m_lift_profile.calculate(ClockTick) == m_lift_goal)
    //   && (m_wrist_profile.calculate(ClockTick) == m_wrist_goal)
    //   && (m_elevator_profile.calculate(ClockTick) == m_elevator_goal)) {
    if (m_lift_profile.isFinished(ClockTick)
    && m_wrist_profile.isFinished(ClockTick)
    && m_elevator_profile.isFinished(ClockTick)) {
        return true;
    } else return false;
  }

}
