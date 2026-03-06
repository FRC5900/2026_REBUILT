// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnBy180 extends Command {
  private final DriveSubsystem m_drive;
  private double m_targetAngle;

  public TurnBy180(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_targetAngle = m_drive.getHeading() + 180.0;
  }

  @Override
  public void execute() {
    double error = m_targetAngle - m_drive.getHeading();
    // Normalize error to [-180, 180]
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;

    double turnSpeed = MathUtil.clamp(
        DriveConstants.kTurnP * error,
        -DriveConstants.kTurnMaxSpeed,
        DriveConstants.kTurnMaxSpeed);

    m_drive.arcadeDrive(0, turnSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    double error = m_targetAngle - m_drive.getHeading();
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;
    return Math.abs(error) < DriveConstants.kTurnTolerance;
  }
}
