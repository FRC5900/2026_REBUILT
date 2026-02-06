package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_leftIntake;
  private final SparkMax m_rightIntake;
  private final SparkMax m_indexer;

  public IntakeSubsystem() {
    m_leftIntake = new SparkMax(IntakeConstants.kLeftIntakeCanId, MotorType.kBrushless);
    m_rightIntake = new SparkMax(IntakeConstants.kRightIntakeCanId, MotorType.kBrushless);
    m_indexer = new SparkMax(IntakeConstants.kIndexerCanId, MotorType.kBrushless);

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(IntakeConstants.kIndexerLimit);
    m_indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.smartCurrentLimit(IntakeConstants.kLauncherLimit);
    launcherConfig.voltageCompensation(12);
    launcherConfig.idleMode(IdleMode.kCoast);

    m_rightIntake.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherConfig.inverted(true);
    m_leftIntake.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeLauncherRoller(double power) {
    m_leftIntake.set(power);
    m_rightIntake.set(power);
  }

  public void setFeederRoller(double power) {
    m_indexer.set(power);
  }

  public void stop() {
    m_indexer.set(0);
    m_leftIntake.set(0);
    m_rightIntake.set(0);
  }
}
