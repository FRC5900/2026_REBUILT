// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Puke extends Command {
  private final ShooterSubsystem m_shooter;

  public Puke(ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setShooterRoller(-ShooterConstants.kShooterPukePower);
    m_shooter.setIndexer(-ShooterConstants.kIndexerLaunchPower);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterRoller(0);
    m_shooter.setIndexer(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
