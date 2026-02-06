// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUp extends Command {
  private final ClimbSubsystem m_climb;

  public ClimbUp(ClimbSubsystem climbSubsystem) {
    m_climb = climbSubsystem;
    addRequirements(m_climb);
  }

  @Override
  public void initialize() {
    m_climb.setClimb(ClimbConstants.kClimbMotorUpPercent);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_climb.stopClimb();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
