package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.DualTagResult;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignToHub extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final PIDController m_rotationController;
  private final PIDController m_distanceController;
  private final SlewRateLimiter m_forwardLimiter  = new SlewRateLimiter(2.5);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(2.5);

  private Translation2d m_hubCenter = null;

  private boolean m_hasValidTarget = false;

  public AlignToHub(DriveSubsystem drive, VisionSubsystem vision) {
    m_drive  = drive;
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
    m_distanceController.setSetpoint(VisionConstants.kShootingDistanceMeters);

    addRequirements(drive);
  }

  private int[] getPrimaryTagIds() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      SmartDashboard.putString("Hub/Alliance", "Blue");
      return new int[] { VisionConstants.kBlueHubTagLeft, VisionConstants.kBlueHubTagRight };
    } else {
      SmartDashboard.putString("Hub/Alliance", alliance.isPresent() ? "Red" : "NOT SET - using Red");
      return new int[] { VisionConstants.kRedHubTagLeft, VisionConstants.kRedHubTagRight };
    }
  }

  private int[] getAllHubTagIds() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return VisionConstants.kBlueHubAllTagIds;
    }
    return VisionConstants.kRedHubAllTagIds;
  }


  private Translation2d computeHubCenter(int[] allTagIds) {
    double sumX = 0, sumY = 0;
    int    count = 0;
    for (int id : allTagIds) {
      Optional<Pose3d> pose = m_vision.getTagPose(id);
      if (pose.isPresent()) {
        sumX  += pose.get().getX();
        sumY  += pose.get().getY();
        count++;
      }
    }
    return count > 0 ? new Translation2d(sumX / count, sumY / count) : null;
  }


  private double poseRotationError(Pose2d robotPose) {
    double dx           = m_hubCenter.getX() - robotPose.getX();
    double dy           = m_hubCenter.getY() - robotPose.getY();
    double bearingToHub = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeading = robotPose.getRotation().getDegrees();
    double headingError = bearingToHub - robotHeading;
    // normalize to [-180, 180]
    while (headingError >  180) headingError -= 360;
    while (headingError < -180) headingError += 360;
    return -headingError;
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Hub/CommandRunning", true);
    m_rotationController.reset();
    m_distanceController.reset();
    m_forwardLimiter.reset(0);
    m_rotationLimiter.reset(0);
    m_hasValidTarget = false;

    m_hubCenter = computeHubCenter(getAllHubTagIds());
    SmartDashboard.putBoolean("Hub/HasHubCenter", m_hubCenter != null);
  }

  @Override
  public void execute() {
    int[]       primaryIds   = getPrimaryTagIds();
    int[]       allTagIds    = getAllHubTagIds();
    DualTagResult primary    = m_vision.getDualTagData(primaryIds[0], primaryIds[1]);
    Pose2d visPose   = m_vision.getEstimatedPose(); // null if no tags seen yet
    Pose2d robotPose = (visPose != null) ? visPose : m_drive.getPose();
    boolean hasTrustedPose = (visPose != null);

    SmartDashboard.putBoolean("Hub/HasLeftTag",  primary.hasLeftTag);
    SmartDashboard.putBoolean("Hub/HasRightTag", primary.hasRightTag);
    SmartDashboard.putBoolean("Hub/HasBothTags", primary.hasBothTags());

    // ── ROTATION ─────────────────────────────────────────────────────────

    double  rotationError = 0;
    boolean hasRotTarget  = false;

    if (primary.hasBothTags()) {
      rotationError = primary.yaw;
      hasRotTarget  = true;
      SmartDashboard.putString("Hub/RotSource", "BOTH_PRIMARY");

    } else if (primary.hasAnyTag()) {
      double cameraYOffset    = m_vision.getLastTargetCameraYOffset();
      double offsetCorrection = Math.toDegrees(Math.atan2(cameraYOffset, primary.distance));
      double visionError      = primary.yaw - offsetCorrection;

      if (hasTrustedPose && m_hubCenter != null) {
        double poseError = poseRotationError(robotPose);
        rotationError = 0.6 * visionError + 0.4 * poseError;
        SmartDashboard.putString("Hub/RotSource", "ONE_PRIMARY_BLEND");
      } else {
        rotationError = visionError;
        SmartDashboard.putString("Hub/RotSource", "ONE_PRIMARY");
      }
      hasRotTarget = true;

    } else {
      PhotonTrackedTarget bestTag = m_vision.getBestTagFromSet(allTagIds);

      if (bestTag != null) {
        double cameraYOffset    = m_vision.getLastTargetCameraYOffset();
        double dist             = Math.hypot(
            bestTag.getBestCameraToTarget().getX(),
            bestTag.getBestCameraToTarget().getY());
        double offsetCorrection = Math.toDegrees(Math.atan2(cameraYOffset, dist));
        double visionError      = bestTag.getYaw() - offsetCorrection;

        if (hasTrustedPose && m_hubCenter != null) {
          double poseError = poseRotationError(robotPose);
          rotationError = 0.3 * visionError + 0.7 * poseError;
          SmartDashboard.putString("Hub/RotSource", "SIDE_TAG_BLEND");
          SmartDashboard.putNumber("Hub/SideTagId", bestTag.getFiducialId());
        } else {
          rotationError = visionError;
          SmartDashboard.putString("Hub/RotSource", "SIDE_TAG");
        }
        hasRotTarget = true;

      } else if (hasTrustedPose && m_hubCenter != null) {
        rotationError = poseRotationError(robotPose);
        hasRotTarget  = true;
        SmartDashboard.putString("Hub/RotSource", "POSE_ONLY");
      } else {
        SmartDashboard.putString("Hub/RotSource", "NONE");
      }
    }

    // ── DISTANCE ─────────────────────────────────────────────────────────
    double distanceMeters = -1;
    if (primary.hasAnyTag()) {
      distanceMeters = primary.distance;
    } else {
      PhotonTrackedTarget anyTag = m_vision.getBestTagFromSet(allTagIds);
      if (anyTag != null) {
        distanceMeters = Math.hypot(
            anyTag.getBestCameraToTarget().getX(),
            anyTag.getBestCameraToTarget().getY());
      }
    }

    m_hasValidTarget = hasRotTarget && distanceMeters > 0;

    // ── DRIVE OUTPUTS ─────────────────────────────────────────────────────
    double rotationOutput = 0;
    double forwardOutput  = 0;

    if (hasRotTarget) {
      rotationOutput = -m_rotationController.calculate(rotationError);
      rotationOutput = MathUtil.clamp(rotationOutput,
          -VisionConstants.kAlignRotationMaxOutput, VisionConstants.kAlignRotationMaxOutput);
      rotationOutput = m_rotationLimiter.calculate(rotationOutput);
    }

    if (distanceMeters > 0) {
      forwardOutput = m_distanceController.calculate(distanceMeters);
      forwardOutput = MathUtil.clamp(forwardOutput, -0.5, 0.5);
      forwardOutput = m_forwardLimiter.calculate(forwardOutput);
    }

    m_drive.arcadeDrive(forwardOutput, rotationOutput);

    SmartDashboard.putNumber("Hub/RotationError",  rotationError);
    SmartDashboard.putNumber("Hub/Distance",        distanceMeters);
    SmartDashboard.putNumber("Hub/ForwardOutput",   forwardOutput);
    SmartDashboard.putNumber("Hub/RotationOutput",  rotationOutput);
    SmartDashboard.putBoolean("Hub/Aligned",        isAligned());
  }

  public boolean isAligned() {
    return m_hasValidTarget
        && m_rotationController.atSetpoint()
        && m_distanceController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    SmartDashboard.putBoolean("Hub/CommandRunning", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
