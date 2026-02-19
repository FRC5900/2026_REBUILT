// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.Puke;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  public RobotContainer() {
    configureButtonBindings();

    m_drive.setDefaultCommand(
        new RunCommand(
            () -> {
              double forward =
                  m_driverController.getRightTriggerAxis()
                      - m_driverController.getLeftTriggerAxis();
              double turn = m_driverController.getRightX();

              forward =
                  MathUtil.applyDeadband(forward, Constants.DriveConstants.kDeadband)
                      * Constants.DriveConstants.kMaxOutput;
              turn =
                  MathUtil.applyDeadband(turn, Constants.DriveConstants.kDeadband)
                      * Constants.DriveConstants.kMaxOutput;

              m_drive.arcadeDrive(forward, turn);
            },
            m_drive));
  }

  private void configureButtonBindings() {
    new JoystickButton(m_operatorController, 5) // Left Bumper, Intake
        .whileTrue(new Intake(m_shooter));

    new JoystickButton(m_operatorController, 6) // Right Bumper, Launch
        .whileTrue(new LaunchSequence(m_shooter));

    new JoystickButton(m_operatorController, 1) // A, Puke
        .whileTrue(new Puke(m_shooter));

    new POVButton(m_driverController, 180) // D-Pad Down, Climb Down
        .whileTrue(new ClimbDown(m_climber));

    new POVButton(m_driverController, 0) // D-Pad Up, Climb Up
        .whileTrue(new ClimbUp(m_climber));

    m_climber.setDefaultCommand(m_climber.run(() -> m_climber.stopClimb()));

    // Operator controller - Right Trigger runs intake at 90% speed
    m_intake.setDefaultCommand(
        new RunCommand(
            () -> {
              double trigger = m_operatorController.getRightTriggerAxis();
              if (trigger > 0.1) {
                double indexerSpeed = m_shooter.getIndexerSpeed();
                m_intake.setIntake(indexerSpeed * 0.9);
              } else {
                m_intake.stop();
              }
            },
            m_intake));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
