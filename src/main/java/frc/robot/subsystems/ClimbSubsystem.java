package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax m_climbMotor;

  public ClimbSubsystem() {
    m_climbMotor = new SparkMax(ClimbConstants.kClimbMotorCanId, MotorType.kBrushless);

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
}
