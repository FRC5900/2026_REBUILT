// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.RelativeEncoder;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;


import com.studica.frc.AHRS;



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

  // NEO encoders 
private final RelativeEncoder m_leftEncoder = m_frontLeft.getEncoder();
private final RelativeEncoder m_rightEncoder = m_frontRight.getEncoder();

// Gyro
private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);


private final DifferentialDriveKinematics m_kinematics =
    new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidthMeters);


private final DifferentialDriveOdometry m_odometry;


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

            m_gyro.reset();


            m_leftEncoder.setPosition(0.0);
            m_rightEncoder.setPosition(0.0);


m_odometry =
    new DifferentialDriveOdometry(
        getHeadingRotation2d(),
        leftDistanceMeters(),
        rightDistanceMeters());


            RobotConfig config;
            try {
            config = RobotConfig.fromGUISettings();
            System.out.println("PathPlanner: RobotConfig loaded OK");
            } catch (Exception e) {
            System.out.println("PathPlanner: RobotConfig FAILED - " + e.getMessage());
            e.printStackTrace();
            config = null;
            }

            if (config != null) {
            AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, ff) -> driveRobotRelative(speeds),
            new PPLTVController(0.02),
            config,
            () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent()
                && alliance.get() == DriverStation.Alliance.Red;
            },
            this);
            System.out.println("PathPlanner: AutoBuilder configured OK");
    }
  }


  private double leftDistanceMeters() {
  return m_leftEncoder.getPosition()
      * (Math.PI * Constants.DriveConstants.kWheelDiameterMeters)
      / Constants.DriveConstants.kGearRatio;
  }

private double rightDistanceMeters() {
  return m_rightEncoder.getPosition()
      * (Math.PI * Constants.DriveConstants.kWheelDiameterMeters)
      / Constants.DriveConstants.kGearRatio;
}

private double leftVelocityMetersPerSec() {
  return (m_leftEncoder.getVelocity() / 60.0)
      * (Math.PI * Constants.DriveConstants.kWheelDiameterMeters)
      / Constants.DriveConstants.kGearRatio;
}

private double rightVelocityMetersPerSec() {
  return (m_rightEncoder.getVelocity() / 60.0)
      * (Math.PI * Constants.DriveConstants.kWheelDiameterMeters)
      / Constants.DriveConstants.kGearRatio;
}



            @Override
            public void periodic() {
            m_odometry.update(
    getHeadingRotation2d(),
    leftDistanceMeters(),
    rightDistanceMeters());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
    SmartDashboard.putNumber("Gyro Raw Yaw", m_gyro.getYaw());
  }




            

    
  

  public void arcadeDrive(double forward, double turn) {
    m_drive.arcadeDrive(forward, -turn, true);
  }



    public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

public void resetPose(Pose2d pose) {
  m_leftEncoder.setPosition(0.0);
  m_rightEncoder.setPosition(0.0);
  m_odometry.resetPosition(
      getHeadingRotation2d(),
      leftDistanceMeters(),
      rightDistanceMeters(),
      pose);
}

public ChassisSpeeds getRobotRelativeSpeeds() {
  DifferentialDriveWheelSpeeds ws =
      new DifferentialDriveWheelSpeeds(
          leftVelocityMetersPerSec(),
          rightVelocityMetersPerSec());

  return m_kinematics.toChassisSpeeds(ws);
}

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds =
        m_kinematics.toWheelSpeeds(speeds);

    double leftPercent =
        wheelSpeeds.leftMetersPerSecond
            / Constants.DriveConstants.kMaxSpeedMetersPerSec;

    double rightPercent =
        wheelSpeeds.rightMetersPerSecond
            / Constants.DriveConstants.kMaxSpeedMetersPerSec;

    leftPercent = MathUtil.clamp(leftPercent, -1.0, 1.0);
    rightPercent = MathUtil.clamp(rightPercent, -1.0, 1.0);

    m_drive.tankDrive(leftPercent, rightPercent, false);
  }


    private Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(-m_gyro.getYaw());
    }

  public double getHeading() {
    return -m_gyro.getYaw();
  }



}
