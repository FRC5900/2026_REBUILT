// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Intake extends Command {
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;

  public Intake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    m_intake = intakeSubsystem;
    m_shooter = shooterSubsystem;
    addRequirements(m_intake, m_shooter);
  }

  @Override
  public void initialize() {
    m_intake.setIntake(IntakeConstants.kIntakePower);
    m_shooter.setIndexer(ShooterConstants.kIndexerIntakingPower);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.setIndexer(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
// hello :3 -Jaiden
// Hello -Thati
//hola -Justin
// john hello - mike-eel