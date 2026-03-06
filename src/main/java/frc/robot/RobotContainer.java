// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AlignToClimb;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.Puke;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//pathplanner stuff
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  private final SlewRateLimiter m_turnLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kTurnSlewRate);
  private final SlewRateLimiter m_forwardLimiter = 
      new SlewRateLimiter(Constants.DriveConstants.kForwardSlewRate);

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    private final VisionSubsystem m_vision = new VisionSubsystem();

   private SendableChooser<Command> m_autoChooser;
  public RobotContainer() {
    configureButtonBindings();

    //PATHPLANNER \//\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
   

    NamedCommands.registerCommand(
    "Intake",
    new Intake(m_intake, m_shooter));

    NamedCommands.registerCommand(
    "Puke",
    new Puke(m_shooter));

    NamedCommands.registerCommand(
    "Launch",
    new LaunchSequence(m_shooter, m_intake));
    NamedCommands.registerCommand(
    "Climb Up",
    new ClimbUp(m_climber));
    NamedCommands.registerCommand(
    "Climb Down",
    new ClimbDown(m_climber));




    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
   
    //PATHPLANNER /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

m_drive.setDefaultCommand(
    new RunCommand(
        () -> {
          double forward =
              m_driverController.getHID().getRightTriggerAxis()
                  - m_driverController.getHID().getLeftTriggerAxis();
          double turn = m_driverController.getHID().getRightX();

          forward =
              MathUtil.applyDeadband(forward, Constants.DriveConstants.kDeadband)
                  * Constants.DriveConstants.kMaxOutput;

          //instant stop smooth acceleration
          if (forward == 0.0) {
            m_forwardLimiter.reset(0.0);
          } else {
            forward = m_forwardLimiter.calculate(forward);
          }

          turn =
              MathUtil.applyDeadband(turn, Constants.DriveConstants.kDeadband)
                  * Constants.DriveConstants.kMaxOutput;
          turn = m_turnLimiter.calculate(turn);

          m_drive.arcadeDrive(forward, turn);
        },
        m_drive));
  }

  private void configureButtonBindings() {
    // Operator controls
    m_operatorController.leftBumper() // Left Bumper, Intake (toggle)
        .toggleOnTrue(new Intake(m_intake, m_shooter));

    m_operatorController.rightBumper() // Right Bumper, Launch
        .whileTrue(new LaunchSequence(m_shooter, m_intake));

    m_operatorController.a() // A, Puke
        .whileTrue(new Puke(m_shooter));

    // Driver controls
    m_driverController.povDown() // D-Pad Down, Climb Down
        .whileTrue(new ClimbDown(m_climber));

    m_driverController.povUp() // D-Pad Up, Climb Up
        .whileTrue(new ClimbUp(m_climber));

    m_driverController.a() // A Button, Align to Hub
        .whileTrue(new AlignToHub(m_drive, m_vision));

    m_driverController.b() // B Button, Align to Climb
        .whileTrue(new AlignToClimb(m_drive, m_vision));

    // Default commands
    m_shooter.setDefaultCommand(m_shooter.run(() -> m_shooter.stop()));
    m_intake.setDefaultCommand(m_intake.run(() -> m_intake.stop()));
    m_climber.setDefaultCommand(m_climber.run(() -> m_climber.stopClimb()));
  }

  public Command getAutonomousCommand() {
  return m_autoChooser.getSelected();
  }
}
