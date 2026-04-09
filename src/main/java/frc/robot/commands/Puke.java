// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Puke extends Command {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;

  public Puke(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    m_shooter = shooterSubsystem;
    m_intake = intakeSubsystem;
    addRequirements(m_shooter, m_intake);
  }

  @Override
  public void initialize() {
    m_shooter.setShooterRoller(ShooterConstants.kShooterPukePower);
    m_shooter.setIndexer(ShooterConstants.kIndexerLaunchPower);
    m_intake.setIntake(-IntakeConstants.kIntakePower);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterRoller(0);
    m_shooter.setIndexer(0);
    m_intake.setIntake(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
