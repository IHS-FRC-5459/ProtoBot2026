// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Whitespace
package frc.robot.subsystems.vision;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  Camera[] cameras;

  Pigeon2 pigeon;
  Drive drive;

  public Vision(String[] cameraNames, Pigeon2 pigeon, Drive drive) {
    // Sets up cameras list
    // I know its sloppy, but it hopefully wors :)
    ArrayList<Camera> camerasList = new ArrayList<Camera>();
    for (String name : cameraNames) {
      camerasList.add(new Camera(name));
    }
    cameras = new Camera[camerasList.size()];
    for (int i = 0; i < camerasList.size(); i++) {
      cameras[i] = camerasList.get(i);
    }

    this.pigeon = pigeon;
    this.drive = drive;
  }

  private boolean poseInField(Pose3d pose) {
    // X check
    double xDeadspace = 0.1; // For each direction
    if (pose.getX() < 0 - xDeadspace || pose.getX() > 17.5387 + xDeadspace) {
      return false;
    }
    // Y check
    double yDeadspace = 0.1; // For each direction
    if (pose.getY() < 0 - yDeadspace || pose.getY() > 8.0518 + yDeadspace) {
      return false;
    }
    // Z check
    double zDeadspace = 0.8; // For each direction
    if (pose.getZ() < 0.095 - zDeadspace || pose.getZ() > 0.095 + zDeadspace) {
      return false;
    }
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
    ArrayList<Matrix> stdDevs = new ArrayList<Matrix>();
    ArrayList<Double> times = new ArrayList<Double>();
    for (int i = 0; i < cameras.length; i++) {
      Logger.recordOutput("photonvisionLogging/check2", true);
      Camera camera = cameras[i];
      camera.periodic();
      // These 3 vars could be slightly differnet time-wise
      Pose3d latestLoc = camera.getLatestLocation();
      Matrix latestStdDev = camera.getLatestStdDevs();
      double latestTime = camera.getLatestTime();
      if (latestLoc != null && latestStdDev != null && camera.canSeeTarget()) {
        if (poseInField(latestLoc)) {
          poses.add(latestLoc.toPose2d());
          stdDevs.add(latestStdDev);
          times.add(latestTime);
          Logger.recordOutput("photonvisionLogging/check1", true);
        } else {
          Logger.recordOutput("photonvisionLogging/check1", false);
        }
      }
    }
    Logger.recordOutput("photonvisionLogging/pigeonRot", pigeon.getRotation2d().getDegrees());
    if (poses.size() == stdDevs.size()) {
      for (int i = 0; i < poses.size(); i++) {
        drive.addVisionMeasurement(poses.get(i), times.get(i), stdDevs.get(i));
      }
    }
    Logger.recordOutput("photonvisionLogging/est Pose", drive.getPose());
  }
}
// Ben was here
