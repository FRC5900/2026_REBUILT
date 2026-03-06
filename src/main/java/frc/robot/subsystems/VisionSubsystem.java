package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera0;
  private final PhotonCamera camera1;
  private final AprilTagFieldLayout fieldLayout;
  private final Transform3d robotToCamera0;
  private final Transform3d robotToCamera1;
  private final BlinkinLEDController ledController;

  private PhotonPipelineResult latestResult0;
  private PhotonPipelineResult latestResult1;
  private boolean hasTarget = false;
  private double targetYaw = 0.0;
  private double targetPitch = 0.0;
  private double targetArea = 0.0;
  private int targetId = -1;

  public VisionSubsystem() {
    camera0 = new PhotonCamera(VisionConstants.kCamera0Name);
    camera1 = new PhotonCamera(VisionConstants.kCamera1Name);
    ledController = BlinkinLEDController.getInstance();

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    robotToCamera0 = new Transform3d(
        VisionConstants.kCamera0X,
        VisionConstants.kCamera0Y,
        VisionConstants.kCamera0Z,
        new Rotation3d(
            VisionConstants.kCamera0Roll,
            VisionConstants.kCamera0Pitch,
            VisionConstants.kCamera0Yaw));

    robotToCamera1 = new Transform3d(
        VisionConstants.kCamera1X,
        VisionConstants.kCamera1Y,
        VisionConstants.kCamera1Z,
        new Rotation3d(
            VisionConstants.kCamera1Roll,
            VisionConstants.kCamera1Pitch,
            VisionConstants.kCamera1Yaw));
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results0 = camera0.getAllUnreadResults();
    if (!results0.isEmpty()) {
      latestResult0 = results0.get(results0.size() - 1);
    }

    List<PhotonPipelineResult> results1 = camera1.getAllUnreadResults();
    if (!results1.isEmpty()) {
      latestResult1 = results1.get(results1.size() - 1);
    }

    PhotonTrackedTarget bestTarget = null;
    double bestArea = 0;

    if (latestResult0 != null && latestResult0.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult0.getTargets()) {
        if (target.getArea() > bestArea) {
          bestArea = target.getArea();
          bestTarget = target;
        }
      }
    }

    if (latestResult1 != null && latestResult1.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult1.getTargets()) {
        if (target.getArea() > bestArea) {
          bestArea = target.getArea();
          bestTarget = target;
        }
      }
    }

    hasTarget = bestTarget != null;
    if (hasTarget) {
      targetYaw = bestTarget.getYaw();
      targetPitch = bestTarget.getPitch();
      targetArea = bestTarget.getArea();
      targetId = bestTarget.getFiducialId();
    } else {
      targetYaw = 0.0;
      targetPitch = 0.0;
      targetArea = 0.0;
      targetId = -1;
    }

    if (hasTarget) {
      ledController.setLawnGreen();
    } else {
      ledController.off();
    }

    // smarty dashboardy

    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
    SmartDashboard.putNumber("Vision/TargetYaw", targetYaw);
    SmartDashboard.putNumber("Vision/TargetPitch", targetPitch);
    SmartDashboard.putNumber("Vision/TargetArea", targetArea);
    SmartDashboard.putNumber("Vision/TargetID", targetId);
    SmartDashboard.putBoolean("Vision/Camera0Connected", camera0.isConnected());
    SmartDashboard.putBoolean("Vision/Camera1Connected", camera1.isConnected());
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getTargetYaw() {
    return targetYaw;
  }

  public double getTargetPitch() {
    return targetPitch;
  }

  public int getTargetId() {
    return targetId;
  }

  //check for targer

  public boolean hasTargetWithId(int tagId) {
    if (latestResult0 != null && latestResult0.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult0.getTargets()) {
        if (target.getFiducialId() == tagId) {
          return true;
        }
      }
    }
    if (latestResult1 != null && latestResult1.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult1.getTargets()) {
        if (target.getFiducialId() == tagId) {
          return true;
        }
      }
    }
    return false;
  }

  
// get target


  private int lastTargetCamera = -1;

  public PhotonTrackedTarget getTargetById(int tagId) {
    PhotonTrackedTarget best = null;
    double bestArea = 0;
    int bestCamera = -1;

    if (latestResult0 != null && latestResult0.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult0.getTargets()) {
        if (target.getFiducialId() == tagId && target.getArea() > bestArea) {
          best = target;
          bestArea = target.getArea();
          bestCamera = 0;
        }
      }
    }
    if (latestResult1 != null && latestResult1.hasTargets()) {
      for (PhotonTrackedTarget target : latestResult1.getTargets()) {
        if (target.getFiducialId() == tagId && target.getArea() > bestArea) {
          best = target;
          bestArea = target.getArea();
          bestCamera = 1;
        }
      }
    }
    lastTargetCamera = bestCamera;
    return best;
  }

  //get y offset
  public double getLastTargetCameraYOffset() {
    if (lastTargetCamera == 0) {
      return VisionConstants.kCamera0Y;
    } else if (lastTargetCamera == 1) {
      return VisionConstants.kCamera1Y;
    }
    return 0.0;
  }

  public double getYawToTag(int tagId) {
    PhotonTrackedTarget target = getTargetById(tagId);
    return target != null ? target.getYaw() : 0.0;
  }

  
  public DualTagResult getDualTagData(int leftTagId, int rightTagId) {
    PhotonTrackedTarget leftTarget = getTargetById(leftTagId);
    PhotonTrackedTarget rightTarget = getTargetById(rightTagId);

    boolean hasLeft = leftTarget != null;
    boolean hasRight = rightTarget != null;

    if (hasLeft && hasRight) {
      // calculate midpoint
      double leftYaw = leftTarget.getYaw();
      double rightYaw = rightTarget.getYaw();
      double avgYaw = (leftYaw + rightYaw) / 2.0;

      Transform3d leftTransform = leftTarget.getBestCameraToTarget();
      Transform3d rightTransform = rightTarget.getBestCameraToTarget();
      double leftDist = Math.hypot(leftTransform.getX(), leftTransform.getY());
      double rightDist = Math.hypot(rightTransform.getX(), rightTransform.getY());
      double avgDist = (leftDist + rightDist) / 2.0;

      return new DualTagResult(avgYaw, avgDist, true, true);
    } else if (hasLeft) {
      // only left tag
      Transform3d transform = leftTarget.getBestCameraToTarget();
      double dist = Math.hypot(transform.getX(), transform.getY());
      return new DualTagResult(leftTarget.getYaw(), dist, true, false);
    } else if (hasRight) {
      // only right tag
      Transform3d transform = rightTarget.getBestCameraToTarget();
      double dist = Math.hypot(transform.getX(), transform.getY());
      return new DualTagResult(rightTarget.getYaw(), dist, false, true);
    }

    return new DualTagResult(0, 0, false, false);
  }

  public static class DualTagResult {
    public final double yaw;
    public final double distance;
    public final boolean hasLeftTag;
    public final boolean hasRightTag;

    public DualTagResult(double yaw, double distance, boolean hasLeft, boolean hasRight) {
      this.yaw = yaw;
      this.distance = distance;
      this.hasLeftTag = hasLeft;
      this.hasRightTag = hasRight;
    }

    public boolean hasAnyTag() {
      return hasLeftTag || hasRightTag;
    }

    public boolean hasBothTags() {
      return hasLeftTag && hasRightTag;
    }
  }

  public Optional<Pose3d> getTagPose(int tagId) {
    return fieldLayout.getTagPose(tagId);
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  public PhotonPipelineResult getLatestResult() {
    
    // find best result, idk if this works :3

    if (latestResult0 != null && latestResult0.hasTargets()) {
      return latestResult0;
    }
    return latestResult1;
  }

  public Transform3d getRobotToCamera0() {
    return robotToCamera0;
  }

  public Transform3d getRobotToCamera1() {
    return robotToCamera1;
  }
}
