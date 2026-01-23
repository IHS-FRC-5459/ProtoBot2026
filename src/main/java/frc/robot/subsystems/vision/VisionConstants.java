// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "frontleft";
  public static String camera1Name = "frontright";
  public static String camera2Name = "backleft";
  public static String camera3Name = "backright";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          0.27225625, 0.235, 0.259, new Rotation3d(0, Math.toRadians(5), Math.toRadians(-15)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          0.2865, -0.222504, 0.257175, new Rotation3d(0, Math.toRadians(5), Math.toRadians(15)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.28813125,
          0.23495,
          0.273812,
          new Rotation3d(0, Math.toRadians(25), Math.toRadians(-159))); // -1 * ( 90+67
  public static Transform3d robotToCamera3 =
      new Transform3d(
          -0.28829,
          -0.21351875,
          0.2667,
          new Rotation3d(0, Math.toRadians(5), Math.toRadians(135))); // -1 * (-90-45
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        40, // Camera 0
        28, // Camera 1
        47, // Camera 2
        40 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
