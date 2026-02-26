package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignToHub extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final PIDController m_rotationController;
  private final PIDController m_distanceController;

  public AlignToHub(DriveSubsystem drive, VisionSubsystem vision) {
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
    m_distanceController.setSetpoint(VisionConstants.kShootingDistanceMeters);

    addRequirements(drive);
  }

  private int getTargetTagId() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return VisionConstants.kRedHubTagId;
    }
    return VisionConstants.kBlueHubTagId;
  }

  @Override
  public void execute() {
    int targetId = getTargetTagId();
    SmartDashboard.putNumber("Hub/LookingForTag", targetId);

    PhotonTrackedTarget target = m_vision.getTargetById(targetId);

    if (target != null) {
      // Get yaw for rotation alignment
      double yaw = target.getYaw();
        //distance from camera to target
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      double distanceMeters = Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());

      //center on tag
      double rotationOutput = m_rotationController.calculate(yaw);
      rotationOutput = MathUtil.clamp(rotationOutput, -0.4, 0.4);

      //calculate forward distance

      double forwardOutput = m_distanceController.calculate(distanceMeters);
      forwardOutput = MathUtil.clamp(forwardOutput, -0.4, 0.4);

      m_drive.arcadeDrive(forwardOutput, rotationOutput);

      //SMARTY DASHBOARDY

      SmartDashboard.putNumber("Hub/Distance", distanceMeters);
      SmartDashboard.putNumber("Hub/Yaw", yaw);
      SmartDashboard.putBoolean("Hub/Aligned", isAligned());
    } else {
      m_drive.arcadeDrive(0, 0);
      SmartDashboard.putBoolean("Hub/Aligned", false);
      SmartDashboard.putNumber("Hub/Distance", -1);
      SmartDashboard.putNumber("Hub/Yaw", 0);
    }
  }

  public boolean isAligned() {
    return m_rotationController.atSetpoint() && m_distanceController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs while button is held
  }
}
