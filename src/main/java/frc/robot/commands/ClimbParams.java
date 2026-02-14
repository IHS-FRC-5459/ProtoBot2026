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
import org.littletonrobotics.junction.Logger;

// pose's x < half field
// then climb goal's y is set to blue alliance goal's y

// poses's y < middle climb structure
// then climb goal's x is set to right alliance goal's x

// Do the other respective ones too ex. >
/** Add your docs here. */
public class ClimbParams {
  private int xMultiplier = 1; // Sign for x-direction
  private int yMultiplier = -1; // Sign for y-direction
  private boolean isFront = false;
  private int omegaMultiplier = 1; // Sign for rotation
  private Pose2d goal;
  private DistanceCaching distCache;

  public ClimbParams(Pose2d estPose) {

    double x_pos = estPose.getX();
    double y_pos = estPose.getY();
    double mid_field_x = aprilTagLayout.getFieldLength() / 2;

    if (x_pos <= mid_field_x) { // IF BLUE
      omegaMultiplier = 1;
      xMultiplier = -1;
      yMultiplier = -1;
      if (y_pos >= 3.75285) { // IF right
        Logger.recordOutput("testt/condition", 2);
        goal = new Pose2d(Inches.of(42), Inches.of(177), new Rotation2d());
        isFront = true;
      } else // ELSE left
      {
        Logger.recordOutput("testt/condition", 4);

        goal = new Pose2d(Inches.of(42), Inches.of(117), new Rotation2d());
        isFront = false;
      }
    } // END IF BLUE
    else // ELSE red
    {
      omegaMultiplier = 1;
      xMultiplier = 1;
      yMultiplier = -1;
      if (y_pos <= 4.318) // IF right
      {
        Logger.recordOutput("testt/condition", 1); // Left red climb align
        goal = new Pose2d(Inches.of(42), Inches.of(136), new Rotation2d());
        isFront = true;
      } else // ELSE left
      {

        Logger.recordOutput("testt/condition", 3);
        // Logger.recordOutput("testt/x sign", xMultiplier);

        goal = new Pose2d(Inches.of(42), Inches.of(195), new Rotation2d());
        isFront = false;
      }
    } // END ELSE RED
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

  public boolean getIsFront() {
    return isFront;
  }
}
