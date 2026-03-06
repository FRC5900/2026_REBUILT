package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.DualTagResult;

public class AlignToClimb extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final PIDController m_rotationController;
  private final PIDController m_distanceController;
  private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(2.0);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(2.0);

  public AlignToClimb(DriveSubsystem drive, VisionSubsystem vision) {
    m_drive = drive;
    m_vision = vision;

    m_rotationController = new PIDController(
        VisionConstants.kAlignRotationP,
        VisionConstants.kAlignRotationI,
        VisionConstants.kAlignRotationD);
    m_rotationController.setTolerance(VisionConstants.kAlignRotationTolerance);
    m_rotationController.setSetpoint(0);

    m_distanceController = new PIDController(
        VisionConstants.kAlignDistanceP,
        VisionConstants.kAlignDistanceI,
        VisionConstants.kAlignDistanceD);
    m_distanceController.setTolerance(VisionConstants.kDistanceToleranceMeters);
    m_distanceController.setSetpoint(VisionConstants.kClimbDistanceMeters);

    addRequirements(drive);
  }

  private int[] getTargetTagIds() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      SmartDashboard.putString("Climb/Alliance", "Blue");
      return new int[] { VisionConstants.kBlueClimbTagLeft, VisionConstants.kBlueClimbTagRight };
    } else {
      SmartDashboard.putString("Climb/Alliance", alliance.isPresent() ? "Red" : "NOT SET - using Red");
      return new int[] { VisionConstants.kRedClimbTagLeft, VisionConstants.kRedClimbTagRight };
    }
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Climb/CommandRunning", true);
    m_rotationController.reset();
    m_distanceController.reset();
    m_forwardLimiter.reset(0);
    m_rotationLimiter.reset(0);
  }

  @Override
  public void execute() {
    int[] tagIds = getTargetTagIds();
    DualTagResult result = m_vision.getDualTagData(tagIds[0], tagIds[1]);

    SmartDashboard.putNumber("Climb/LeftTag", tagIds[0]);
    SmartDashboard.putNumber("Climb/RightTag", tagIds[1]);
    SmartDashboard.putBoolean("Climb/HasLeftTag", result.hasLeftTag);
    SmartDashboard.putBoolean("Climb/HasRightTag", result.hasRightTag);
    SmartDashboard.putBoolean("Climb/HasBothTags", result.hasBothTags());

    if (result.hasAnyTag()) {
      double yaw = result.yaw;
      double distanceMeters = result.distance;

      // camera offset for singular tag
      if (!result.hasBothTags()) {
        double cameraYOffset = m_vision.getLastTargetCameraYOffset();
        double offsetCorrection = Math.toDegrees(Math.atan2(cameraYOffset, distanceMeters));
        yaw = yaw - offsetCorrection;
        SmartDashboard.putNumber("Climb/CameraOffset", cameraYOffset);
      } else {
        SmartDashboard.putNumber("Climb/CameraOffset", 0);
      }

      // Center on tag(s)
      double rotationOutput = -m_rotationController.calculate(yaw);
      rotationOutput = MathUtil.clamp(rotationOutput, -0.4, 0.4);
      rotationOutput = m_rotationLimiter.calculate(rotationOutput);

      // Calculate forward distance
      double forwardOutput = m_distanceController.calculate(distanceMeters);
      forwardOutput = MathUtil.clamp(forwardOutput, -0.4, 0.4);
      forwardOutput = m_forwardLimiter.calculate(forwardOutput);

      m_drive.arcadeDrive(forwardOutput, rotationOutput);

      // SMARTYDASHBOARDY
      SmartDashboard.putNumber("Climb/Distance", distanceMeters);
      SmartDashboard.putNumber("Climb/Yaw", yaw);
      SmartDashboard.putNumber("Climb/ForwardOutput", forwardOutput);
      SmartDashboard.putNumber("Climb/RotationOutput", rotationOutput);
      SmartDashboard.putBoolean("Climb/Aligned", isAligned());
    } else {
      m_drive.arcadeDrive(0, 0);
      SmartDashboard.putBoolean("Climb/Aligned", false);
      SmartDashboard.putNumber("Climb/Distance", -1);
      SmartDashboard.putNumber("Climb/Yaw", 0);
      SmartDashboard.putNumber("Climb/ForwardOutput", 0);
      SmartDashboard.putNumber("Climb/RotationOutput", 0);
    }
  }

  public boolean isAligned() {
    return m_rotationController.atSetpoint() && m_distanceController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    SmartDashboard.putBoolean("Climb/CommandRunning", false);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs while button is held
  }
}
