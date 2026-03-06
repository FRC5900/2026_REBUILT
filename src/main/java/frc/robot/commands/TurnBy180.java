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
  private double m_turnDirection; // set once in initialize, never changes

  public TurnBy180(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    double current = m_drive.getHeading();
    m_targetAngle = current + 180.0;
    // Normalize target into (-180, 180] to match gyro output range
    if (m_targetAngle > 180)  m_targetAngle -= 360;
    if (m_targetAngle < -180) m_targetAngle += 360;

    // Pick direction once and never change it — avoids the ±180 oscillation where
    // the P controller reverses every loop because error flips between +180 and -180.
    double diff = m_targetAngle - current;
    if (diff > 180)  diff -= 360;
    if (diff < -180) diff += 360;
    m_turnDirection = diff >= 0 ? 1.0 : -1.0;
  }

  @Override
  public void execute() {
    m_drive.arcadeDrive(0, -DriveConstants.kTurnMaxSpeed * m_turnDirection);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    double error = m_targetAngle - m_drive.getHeading();
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;
    return Math.abs(error) <= DriveConstants.kTurnTolerance;
  }
}
