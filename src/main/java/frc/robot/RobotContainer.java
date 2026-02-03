// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_driverController = new XboxController(0);

  private final DriveSubsystem m_drive = new DriveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    m_drive.setDefaultCommand(
        new RunCommand(
            () -> {
              double forward =
                  m_driverController.getRightTriggerAxis()
                      - m_driverController.getLeftTriggerAxis();

              double turn = m_driverController.getLeftX();

              forward =
                  MathUtil.applyDeadband(forward, Constants.DriveConstants.kDeadband)
                      * Constants.DriveConstants.kMaxOutput;
              turn =
                  MathUtil.applyDeadband(turn, Constants.DriveConstants.kDeadband)
                      * Constants.DriveConstants.kMaxOutput;

              SmartDashboard.putNumber("RT", m_driverController.getRightTriggerAxis());
              SmartDashboard.putNumber("LT", m_driverController.getLeftTriggerAxis());
              SmartDashboard.putNumber("Forward", forward);
              SmartDashboard.putNumber("Turn", turn);

              m_drive.arcadeDrive(forward, turn);
            },
            m_drive));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
