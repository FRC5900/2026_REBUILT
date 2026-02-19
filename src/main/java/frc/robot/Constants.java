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
  //fix michaels code :D
  //this comment did indeed fix michaels github desktop :D -- michael
  public static final class DriveConstants {
    //SparkMaxs
    public static final int kFrontLeftCanId = 4; //no clue what these are on the test board .-.
    public static final int kRearLeftCanId = 5;
    public static final int kFrontRightCanId = 2;
    public static final int kRearRightCanId = 3;

    public static final double kDeadband = 0.08;
    public static final double kMaxOutput = 1; // speed i lowered it for you lol
    public static final double kTurnSlewRate = 3.0; // units per second
  }

  
  public static final class IntakeConstants {
    //Intake SparkMaxs
    public static final int kIntakeCanId = 10;

    //Amp Limits
    public static final int kIntakeLimit = 40;

    //Intake Speeds
    public static final double kIntakePower = 0.9;
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
    
  }

  private Constants() {}
}
