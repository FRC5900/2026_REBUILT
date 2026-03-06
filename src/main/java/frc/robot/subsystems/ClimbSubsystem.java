package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax m_climbMotor;
  private final DutyCycleEncoder m_encoder;

  public ClimbSubsystem() {
    m_climbMotor = new SparkMax(ClimbConstants.kClimbMotorCanId, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(ClimbConstants.kEncoderDIOPort);

    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(ClimbConstants.kClimbMotorLimit);
    climbConfig.idleMode(IdleMode.kBrake);
    m_climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimb(double power) {
    m_climbMotor.set(power);
  }

  public void runClimb(double speed) {
    m_climbMotor.set(speed);
  }

  public void stopClimb() {
    m_climbMotor.set(0);
  }

  public double getPosition() {
    return (m_encoder.get() + ClimbConstants.kEncoderOffset) % 1.0;
  }

  // Returns the motor direction needed to reach target: +1 = up, -1 = down.
  // Wrap-aware so it works correctly even if the encoder crosses 0.
  public double getDirectionTo(double target) {
    double diff = target - getPosition();
    if (diff > 0.5) diff -= 1.0;
    if (diff < -0.5) diff += 1.0;
    return Math.signum(diff);
  }

  // Returns true when within tolerance of target, wrap-aware.
  public boolean isAtPosition(double target) {
    double diff = Math.abs(getPosition() - target);
    return Math.min(diff, 1.0 - diff) <= ClimbConstants.kPositionTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Raw", m_encoder.get());
    SmartDashboard.putNumber("Climb Position", getPosition());
  }
}
