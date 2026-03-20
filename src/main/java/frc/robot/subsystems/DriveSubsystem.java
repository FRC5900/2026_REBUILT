// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

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
private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
private final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

// Gyro
private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);


private final DifferentialDriveKinematics m_kinematics =
    new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidthMeters);


private final DifferentialDriveOdometry m_odometry;
private double m_targetHeading = 0.0;
  

  public DriveSubsystem() {
   
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(true);
    m_frontRight.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rearRight.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_drive =
        new DifferentialDrive(
            left -> {
              m_frontLeft.set(left);
              m_rearLeft.set(left);
            },
            right -> {
              m_frontRight.set(right);
              m_rearRight.set(right);
            });
        m_drive.setSafetyEnabled(false);

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
            () -> false,
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
    SmartDashboard.putNumber("Drive/FL Encoder Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Drive/RL Encoder Position", m_rearLeftEncoder.getPosition());
    SmartDashboard.putNumber("Drive/FR Encoder Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Drive/RR Encoder Position", m_rearRightEncoder.getPosition());
    SmartDashboard.putNumber("Drive/FL Encoder Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/RL Encoder Velocity", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/FR Encoder Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/RR Encoder Velocity", m_rearRightEncoder.getVelocity());
    SmartDashboard.putNumber("Drive/Left Distance (m)", leftDistanceMeters());
    SmartDashboard.putNumber("Drive/Right Distance (m)", rightDistanceMeters());
    SmartDashboard.putNumber("Drive/Left Speed (m/s)", leftVelocityMetersPerSec());
    SmartDashboard.putNumber("Drive/Right Speed (m/s)", rightVelocityMetersPerSec());
    SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Gyro Pitch Rate (deg/s)", m_gyro.getRawGyroY());
    SmartDashboard.putNumber("Speed Limit - Bump", getBumpSpeedFactor());
    SmartDashboard.putNumber("Speed Limit - Rocking", getRockingSpeedFactor());
    SmartDashboard.putNumber("Speed Limit - Wheel Slip", getWheelSlipSpeedFactor());
  }

  // multiplies speeds down when going over bump
  private double getBumpSpeedFactor() {
    double pitch = Math.abs(m_gyro.getPitch());
    if (pitch <= Constants.DriveConstants.kBumpPitchThreshold) return 1.0;
    if (pitch >= Constants.DriveConstants.kBumpMaxPitch) return Constants.DriveConstants.kBumpMinSpeedFactor;
    double t = (pitch - Constants.DriveConstants.kBumpPitchThreshold)
        / (Constants.DriveConstants.kBumpMaxPitch - Constants.DriveConstants.kBumpPitchThreshold);
    return 1.0 - t * (1.0 - Constants.DriveConstants.kBumpMinSpeedFactor);
  }

  // multiplies speeds down when rocked
  private double getRockingSpeedFactor() {
    double pitchRate = Math.abs(m_gyro.getRawGyroY());
    if (pitchRate <= Constants.DriveConstants.kRockPitchRateThreshold) return 1.0;
    if (pitchRate >= Constants.DriveConstants.kRockMaxPitchRate) return Constants.DriveConstants.kRockMinSpeedFactor;
    double t = (pitchRate - Constants.DriveConstants.kRockPitchRateThreshold)
        / (Constants.DriveConstants.kRockMaxPitchRate - Constants.DriveConstants.kRockPitchRateThreshold);
    return 1.0 - t * (1.0 - Constants.DriveConstants.kRockMinSpeedFactor);
  }

  // multiplies speeds down when wheels are slipping
  private double getWheelSlipSpeedFactor() {
    double wheelSpeed = Math.abs((leftVelocityMetersPerSec() + rightVelocityMetersPerSec()) / 2.0);
    double imuSpeed = Math.abs(m_gyro.getVelocityX());
    double excess = wheelSpeed - imuSpeed;
    if (excess <= Constants.DriveConstants.kSlipThreshold) return 1.0;
    if (excess >= Constants.DriveConstants.kSlipMaxExcess) return Constants.DriveConstants.kSlipMinSpeedFactor;
    double t = (excess - Constants.DriveConstants.kSlipThreshold)
        / (Constants.DriveConstants.kSlipMaxExcess - Constants.DriveConstants.kSlipThreshold);
    return 1.0 - t * (1.0 - Constants.DriveConstants.kSlipMinSpeedFactor);
  }

  // failsafe; finds most restrictive
  public void arcadeDrive(double forward, double turn) {
    if (Math.abs(turn) < 0.05 && Math.abs(forward) > 0.05) {
      // dirft cirrebctuinb
      double headingError = m_targetHeading - getHeading();
      if (headingError > 180) headingError -= 360;
      if (headingError < -180) headingError += 360;
      double correction = MathUtil.clamp(
          headingError * Constants.DriveConstants.kDriftCorrectionP,
          -Constants.DriveConstants.kDriftCorrectionMaxTurn,
          Constants.DriveConstants.kDriftCorrectionMaxTurn);
      turn = correction;
    } else if (Math.abs(turn) >= 0.05) {
      // ignore normal turning
      m_targetHeading = getHeading();
    }

    double forwardLimit = Math.min(getBumpSpeedFactor(), getRockingSpeedFactor());
    m_drive.arcadeDrive(forward * forwardLimit, -turn, true);
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
