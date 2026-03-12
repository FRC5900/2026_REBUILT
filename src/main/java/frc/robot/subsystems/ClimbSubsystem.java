package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax m_rightClimbMotor;
  private final SparkMax m_leftClimbMotor;
  private final RelativeEncoder m_encoder;

  public ClimbSubsystem() {
    m_rightClimbMotor = new SparkMax(ClimbConstants.kRightClimbMotorCanId, MotorType.kBrushless);
    m_leftClimbMotor = new SparkMax(ClimbConstants.kLeftClimbMotorCanId, MotorType.kBrushless);
    m_encoder = m_rightClimbMotor.getEncoder();

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.smartCurrentLimit(ClimbConstants.kClimbMotorLimit);
    rightConfig.idleMode(IdleMode.kBrake);
    m_rightClimbMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(ClimbConstants.kClimbMotorLimit);
    leftConfig.idleMode(IdleMode.kBrake);
    m_leftClimbMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder.setPosition(0.0);
  }

  public void setClimb(double power) {
    m_rightClimbMotor.set(power);
    m_leftClimbMotor.set(-power);
  }

  public void runClimb(double speed) {
    m_rightClimbMotor.set(speed);
    m_leftClimbMotor.set(-speed);
  }

  public void stopClimb() {
    m_rightClimbMotor.set(0);
    m_leftClimbMotor.set(0);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

@Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Position", getPosition());
  }
}
