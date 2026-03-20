package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDown extends Command {
  private final ClimbSubsystem m_climb;

  public ClimbDown(ClimbSubsystem climbSubsystem) {
    m_climb = climbSubsystem;
    addRequirements(m_climb);
  }
//forcing commit - justin hkb
  @Override
  public void initialize() {
    m_climb.setClimb(ClimbConstants.kClimbMotorDownPower);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_climb.stopClimb();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
