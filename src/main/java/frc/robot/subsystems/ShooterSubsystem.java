// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_leftShooter;
  private final SparkMax m_rightShooter;
  private final SparkMax m_indexer;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  public ShooterSubsystem() {
    m_leftShooter = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
    m_rightShooter = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);
    m_indexer = new SparkMax(ShooterConstants.kIndexerCanId, MotorType.kBrushless);

    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.smartCurrentLimit(ShooterConstants.kIndexerLimit);
    m_indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.smartCurrentLimit(ShooterConstants.kShooterLimit);
    shooterConfig.voltageCompensation(12);
    shooterConfig.idleMode(IdleMode.kCoast);

    m_rightShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterConfig.inverted(true);
    m_leftShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftEncoder = m_leftShooter.getEncoder();
    m_rightEncoder = m_rightShooter.getEncoder();
    
    SmartDashboard.putNumber("Shooter Bus Voltage", m_leftShooter.getBusVoltage());
  }

  public void setShooterRoller(double power) {
    m_leftShooter.set(power);
    m_rightShooter.set(power);
  }
  
  public void setIndexer(double power) {
    m_indexer.set(power);
  }

  public double getIndexerPower() {
    return m_indexer.get();
  }

  public double getShooterVelocity() {
    return (Math.abs(m_leftEncoder.getVelocity()) + Math.abs(m_rightEncoder.getVelocity())) / 2.0;
  }

  public boolean isAtSpeed() {
    return getShooterVelocity() >= (ShooterConstants.kShooterTargetRPM - ShooterConstants.kShooterSpeedTolerance);
  }

  public void stop() {
    m_indexer.set(0);
    m_leftShooter.set(0);
    m_rightShooter.set(0);
  }
}
