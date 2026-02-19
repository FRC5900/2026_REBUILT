// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {


  //SparkMaxs 
  private final SparkMax m_frontLeft = 
    new SparkMax(Constants.DriveConstants.kFrontLeftCanId, MotorType.kBrushless);
  private final SparkMax m_rearLeft = 
    new SparkMax(Constants.DriveConstants.kRearLeftCanId, MotorType.kBrushless);
  private final SparkMax m_frontRight = 
    new SparkMax(Constants.DriveConstants.kFrontRightCanId, MotorType.kBrushless);
  private final SparkMax m_rearRight = 
    new SparkMax(Constants.DriveConstants.kRearRightCanId, MotorType.kBrushless);

  private final DifferentialDrive m_drive;

  public DriveSubsystem() {
   

     m_drive =
        new DifferentialDrive(
            left -> {
              m_frontLeft.set(left);
              m_rearLeft.set(left);
            },
            right -> {
              m_frontRight.set(-right);
              m_rearRight.set(-right);
            });
  }

  public void arcadeDrive(double forward, double turn) {
    m_drive.arcadeDrive(forward, -turn, true);
  }

}
