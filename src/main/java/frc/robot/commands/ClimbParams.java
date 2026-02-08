// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DistanceCaching;

// pose's x < half field
// then climb goal's y is set to blue alliance goal's y

// poses's y < middle climb structure
// then climb goal's x is set to right alliance goal's x

// Do the other respective ones too ex. >
/** Add your docs here. */
public class ClimbParams {
  private int xMultiplier = 1; // Sign for x-direction
  private int yMultiplier = 1; // Sign for y-direction

  private int omegaMultiplier = 1; // Sign for rotation
  private Pose2d goal;
  private DistanceCaching distCache;

  public ClimbParams(Pose2d estPose) {
    if (estPose.getX() > aprilTagLayout.getFieldLength() / 2) {
      xMultiplier = -1;
      omegaMultiplier = -1;
    }
    if (estPose.getX() > aprilTagLayout.getFieldLength()
        && estPose.getY()
            > 158.85 /*climb struct y */) { // Left red climb align (from looking from blue alliance
      // wall)
      goal = new Pose2d(Inches.of(42), Inches.of(175), new Rotation2d());
      distCache =
          new DistanceCaching(
              Sensors.Distance.backLeftId,
              Sensors.Distance.backRightId,
              Sensors.Distance.xRobotOffsetBack,
              "back");
    }
    if (estPose.getX() < aprilTagLayout.getFieldLength()
        && estPose.getY() > 147.47 /*climb struct y */) { // Left blue climb align
      goal = new Pose2d(Inches.of(582.22), Inches.of(143.535 - 13.5 - 48 - 100), new Rotation2d());
      distCache =
          new DistanceCaching(
              Sensors.Distance.frontLeftId,
              Sensors.Distance.frontRightId,
              Sensors.Distance.xRobotOffsetFront,
              "front");
    }
    if (estPose.getX() > aprilTagLayout.getFieldLength()
        && estPose.getY() < 158.85 /*climb struct y */) { // Right red climb align
      goal = new Pose2d(Inches.of(42), Inches.of(120.5), new Rotation2d());
      distCache =
          new DistanceCaching(
              Sensors.Distance.frontLeftId,
              Sensors.Distance.frontRightId,
              Sensors.Distance.xRobotOffsetFront,
              "front");
    }
    if (estPose.getX() < aprilTagLayout.getFieldLength()
        && estPose.getY() < 147.47 /*climb struct y */) { // Right blue climb align
      goal = new Pose2d(Inches.of(582.22), Inches.of(143.535 - 13.5 - 48 - 100), new Rotation2d());
      distCache =
          new DistanceCaching(
              Sensors.Distance.backLeftId,
              Sensors.Distance.backRightId,
              Sensors.Distance.xRobotOffsetBack,
              "back");
    }
    // if(estPose.getY() > /*middle y coord of climb structure */){
    //
    // }
  }

  public Pose2d getGoal() {
    return goal;
  }
  // These are always 1 or -1
  public int getXMultiplier() {
    return xMultiplier;
  }

  public int getYMultiplier() {
    return yMultiplier;
  }

  public int getOmegaMultiplier() {
    return omegaMultiplier;
  }

  public DistanceCaching getDistCache() {
    return this.distCache;
  }
}
