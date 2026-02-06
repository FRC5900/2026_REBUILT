// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUp extends Command {
  private final IntakeSubsystem m_intake;

  public SpinUp(IntakeSubsystem intakeSubsystem) {
    m_intake = intakeSubsystem;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setIntakeLauncherRoller(IntakeConstants.kLaunchPercent);
    m_intake.setFeederRoller(IntakeConstants.kIndexerSpinUpPreLaunchPercent);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
