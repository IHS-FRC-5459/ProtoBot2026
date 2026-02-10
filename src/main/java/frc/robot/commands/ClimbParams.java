// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DistanceCaching;
import org.littletonrobotics.junction.Logger;

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
  private RobotContainer m_robotContainer;

  public ClimbParams(Pose2d estPose, RobotContainer m_robotContainer) {
    this.m_robotContainer = m_robotContainer;
    if (estPose.getX() < aprilTagLayout.getFieldLength() / 2) {
      omegaMultiplier = 1;
      xMultiplier = -1;
      yMultiplier = -1;
    } else {
      yMultiplier = 1;
    }
    if (estPose.getX() >= aprilTagLayout.getFieldLength() / 2
        && estPose.getY()
            >= 4.318 /*climb struct y */) { // Left red climb align (from looking from blue
      // alliance
      // wall)
      Logger.recordOutput("testt/condition", 1);
      goal = new Pose2d(Inches.of(610), Inches.of(205), new Rotation2d());
      distCache = m_robotContainer.getDistanceCacheFront();
    }
    if (estPose.getX() <= aprilTagLayout.getFieldLength() / 2
        && estPose.getY() >= 3.75285 /*climb struct y */) { // Left blue climb align
      Logger.recordOutput("testt/condition", 2);
      goal = new Pose2d(Inches.of(42), Inches.of(182), new Rotation2d());
      distCache = m_robotContainer.getDistanceCacheFront();
    }
    if (estPose.getX() >= aprilTagLayout.getFieldLength() / 2
        && estPose.getY() <= 4.318 /*climb struct y */) { // Right red climb align
      Logger.recordOutput("testt/condition", 3);

      goal = new Pose2d(Inches.of(610), Inches.of(135), new Rotation2d());
      distCache = m_robotContainer.getDistanceCacheFront();
    }
    if (estPose.getX() <= aprilTagLayout.getFieldLength() / 2
        && estPose.getY() <= 3.75285 /*climb struct y */) { // Right blue climb align
      Logger.recordOutput("testt/condition", 4);

      goal = new Pose2d(Inches.of(42), Inches.of(113.5), new Rotation2d());
      distCache = m_robotContainer.getDistanceCacheBack();
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
