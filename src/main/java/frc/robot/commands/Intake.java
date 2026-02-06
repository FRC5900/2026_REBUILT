// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
  private final IntakeSubsystem m_intake;

  public Intake(IntakeSubsystem intakeSubsystem) {
    m_intake = intakeSubsystem;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setIntakeLauncherRoller(IntakeConstants.kIntakeLaunchPercent);
    m_intake.setFeederRoller(IntakeConstants.kIndexerIntakingPercent);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeLauncherRoller(0);
    m_intake.setFeederRoller(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
