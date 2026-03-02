// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //fix michaels code :D -- Justin
  //this comment did indeed fix michaels github desktop :D -- michael
  public static final class DriveConstants {
    //SparkMaxs
    public static final int kFrontLeftCanId = 4; //no clue what these are on the test board .-. -- Justin
    public static final int kRearLeftCanId = 5;
    public static final int kFrontRightCanId = 2;
    public static final int kRearRightCanId = 3;

    public static final double kDeadband = 0.08;
    public static final double kMaxOutput = 1; // speed i lowered it for you lol
    public static final double kTurnSlewRate = 3.0; // units per second
    public static final double kForwardSlewRate = 3.0; // units per second


    //Pathplanner stuff
    public static final double kTrackWidthMeters = 0.5906; 
    public static final double kWheelDiameterMeters = 0.1524; // 6" wheels = 0.1524m
    public static final double kGearRatio = 10.71; //motor rotations per wheel rotation     WE NEED TO FIGURE THIS OUT :D -- Justin
    public static final double kMaxSpeedMetersPerSec = 1;
  }

  
  public static final class IntakeConstants {
    //Intake SparkMaxs
    public static final int kIntakeCanId = 10;

    //Amp Limits
    public static final int kIntakeLimit = 40;

    //Intake Speeds
    public static final double kIntakePower = -0.9;
  }

  public static final class ClimbConstants {
    //SparkMaxs
    public static final int kClimbMotorCanId = 9;

    //Amp Limits
    public static final int kClimbMotorLimit = 40;

    //Climb Speeds
    public static final double kClimbMotorDownPower = 0.7;
    public static final double kClimbMotorUpPower = -0.7;
  }

  public static final class ShooterConstants {
    //SparkMaxs
    public static final int kLeftShooterCanId = 6;
    public static final int kRightShooterCanId = 7;
    public static final int kIndexerCanId = 8;

    //Amp Limits
    public static final int kIndexerLimit = 40;
    public static final int kShooterLimit = 40;

    //Indexer Constants
    public static final double kIndexerIntakingPower = -0.8;
    public static final double kIndexerLaunchPower = 0.6;
    public static final double kIndexerSpinUp = -0.5;
    public static final double kIndexSpinUpTime = 0.75;

    //Launcher Speeds
    public static final double kShooterLaunchPower = 0.6;
    public static final double kLaunchPower = 0.85;
    public static final double kShooterPukePower = -0.8;

    //Intake

    public static final double kIntakeShootPower = -0.5;

  }

  public static final class LEDConstants {
    public static final int kBlinkinPort = 0;
  }

  public static final class VisionConstants {
    
    public static final String kCamera0Name = "BW_1";
    public static final String kCamera1Name = "BW_2";

    //robot transforms to camera (x forward, y left, z up)

    //camera 0 - front left

    public static final double kCamera0X = 0.2;  // meters forward
    public static final double kCamera0Y = 0.15;  // meters left
    public static final double kCamera0Z = 0.2;  // meters up
    public static final double kCamera0Roll = 0.0;
    public static final double kCamera0Pitch = -0.4;
    public static final double kCamera0Yaw = 0.0; 

    //camera 1 - front right

    public static final double kCamera1X = 0.2;  // meters forward
    public static final double kCamera1Y = -0.15;  // meters right
    public static final double kCamera1Z = 0.2; // meters up
    public static final double kCamera1Roll = 0.0;
    public static final double kCamera1Pitch = -0.4;
    public static final double kCamera1Yaw = 0.0;  

    //filtering thresholds
    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxZError = 0.75;

    //deviation thresholds
    public static final double kLinearStdDevBaseline = 0.02;
    public static final double kAngularStdDevBaseline = 0.06;

    //camera biases
    public static final double kCamera0StdDevFactor = 1.0;
    public static final double kCamera1StdDevFactor = 1.0;

    //rotation)
    public static final double kAlignRotationP = 0.05;
    public static final double kAlignRotationI = 0.0;
    public static final double kAlignRotationD = 0.0;
    public static final double kAlignRotationTolerance = 2.0;

    //distance
    public static final double kAlignDistanceP = 1.0;
    public static final double kAlignDistanceI = 0.0;
    public static final double kAlignDistanceD = 0.0;
    public static final double kShootingDistanceMeters = 0.3048;
    public static final double kDistanceToleranceMeters = 0.05;

    //tags
    public static final int kRedHubTagId = 10;
    public static final int kBlueHubTagId = 26;
  }

  private Constants() {}
}
