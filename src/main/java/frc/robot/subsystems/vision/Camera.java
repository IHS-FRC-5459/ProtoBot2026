// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  CameraConstants constants;

  AprilTagFieldLayout kTagLayout;
  PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private double latestTimestamp = 0.0;

  public Camera(String kCameraName) {
    constants = CameraConstants.cameras.get(kCameraName);
    if (!Robot.isSimulation()) {
      try {
        Path path = Paths.get("/home/lvuser/deploy/field.json");
        if (Files.exists(path)) {
          kTagLayout = new AprilTagFieldLayout(path);
        } else {
          System.out.println("File does not exist");
        }
      } catch (Exception e) {
        System.out.println("Error: " + e);
      }
    }
    System.out.println(
        "Cmaera constants: " + constants.kCameraName + " : " + constants.kRobotToCam);
    camera = new PhotonCamera(constants.kCameraName);
    photonEstimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, constants.kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public String getName() {
    return this.constants.kCameraName;
  }

  private Matrix<N3, N1> curStdDevs;

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = constants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = constants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = constants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = constants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  private boolean targetVisible = false;
  private Pose3d latestLocation = new Pose3d();
  private Matrix<N3, N1> estStdDevs = VecBuilder.fill(1000, 1000, 1000);

  public boolean canSeeTarget() {
    return targetVisible;
  }

  private Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  public Matrix<N3, N1> getLatestStdDevs() {
    if (estStdDevs == null) {
      return null;
    }
    return this.estStdDevs;
  }

  public Pose3d getLatestLocation() {
    if (latestLocation == null) {
      return null;
    }
    return this.latestLocation;
  }

  public double getLatestTime() {
    return latestTimestamp;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      Logger.recordOutput("photonvisionLogging/change" + constants.kCameraName, change);
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
      targetVisible = change.getTargets().size() != 0;
      Logger.recordOutput(
          "photonvisionLogging/" + constants.kCameraName + "sees target", targetVisible);
    }
    visionEst.ifPresent(
        est -> {
          // Change our trust in the measurement based on the tags we can see
          this.latestLocation = est.estimatedPose;
          this.estStdDevs = getEstimationStdDevs();
          this.latestTimestamp = est.timestampSeconds;
        });
    Logger.recordOutput(
        "photonvisionLogging/" + constants.kCameraName + " latestLocc", getLatestLocation());
  }
}
