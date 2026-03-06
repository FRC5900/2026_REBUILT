package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbToClimbPosition extends Command {
  private final ClimbSubsystem m_climb;
  private double m_speed;

  public ClimbToClimbPosition(ClimbSubsystem climbSubsystem) {
    m_climb = climbSubsystem;
    addRequirements(m_climb);
  }

  @Override
  public void initialize() {
    double dir = m_climb.getDirectionTo(ClimbConstants.kClimbPosition);
    m_speed = dir > 0 ? ClimbConstants.kClimbMotorUpPower : ClimbConstants.kClimbMotorDownPower;
    m_climb.setClimb(m_speed);
  }

  @Override
  public void execute() {
    m_climb.setClimb(m_speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_climb.stopClimb();
  }

  @Override
  public boolean isFinished() {
    return m_climb.isAtPosition(ClimbConstants.kClimbPosition);
  }
}
