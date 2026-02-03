// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.Sensors.Distance.*;
import static frc.robot.commands.DriveCommands.joystickDriveAtAngleCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DistanceCaching;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbRight extends Command {
  DistanceCaching distCache;
  Drive s_drive;
  /** Creates a new Climb. */
  // Climbs the right side of the climb structure(from the perspective of the alliance station)
  public ClimbRight(Drive s_drive, DistanceCaching distCache) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(s_drive);
    this.s_drive = s_drive;
    this.distCache = distCache;
  }

  // Called when the command is initially scheduled.
  // private boolean doneAligningToStart = false;

  @Override
  public void initialize() {
    setIsFirstCall(true);
    // omegaPassed1 = false;
    // yPassed = false;
    // xPassed = false;
    isDone = false;
    // doneAligningToStart = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private boolean isDone = false;
  // private boolean omegaPassed1, yPassed, xPassed = false;

  @Override
  public void execute() {
    Pose2d currPose = s_drive.getPose();
    // want to go to to make sure we are going in with good alignment
    // Blue alliance
    Rotation2d blueClimbRot = new Rotation2d(Math.PI); // Facing the alliance wall
    Rotation2d redClimbRot = new Rotation2d(0);
    // Note, these give a 2ft buffer dtstance
    Pose2d blueClimb = new Pose2d(Inches.of(41.56), Inches.of(35.845 + 13.5 + 48), blueClimbRot);
    Pose2d redClimb = new Pose2d(Inches.of(582.22), Inches.of(143.535 - 13.5 - 48), redClimbRot);
    boolean isBlueAlliance = true;
    boolean omegaPassed = false;
    double directionMult = 1;
    Pose2d climbPose = blueClimb;
    if (currPose.getX() > aprilTagLayout.getFieldLength() / 2) { // Far-side(red)
      climbPose = redClimb;
      isBlueAlliance = false;
    }
    if (isBlueAlliance) {
      directionMult = -1;
    }
    Logger.recordOutput("climb/climbPose", climbPose);
    // X
    int numValid = 0; // Purposely doesnt use distCache to avoid runtime changes
    double yDist = 0;
    // double theta = 0;
    if (distCache.bothValid()) {
      double right = distCache.getRight();
      double left = distCache.getLeft();
      if (right == left) {
        yDist = left;
        // theta = 0;
      } else {
        double beta = Math.atan(Math.abs(right - left) / camDist);
        double alpha = Math.PI / 2 - beta;
        double x = (100 / (Math.tan(alpha)));
        yDist =
            Math.sin(beta) * (Math.max(left, right) + (robotWidth / 2) + x)
                - Math.sqrt(((camDist / 2) * (camDist / 2) + Math.pow(x, 2)));
        /// theta = Math.acos(yDist / (((left + right) / 2) + (robotWidth / 2)));
        // if (right > left) {
        // theta *= -1;
        // }
      }
      numValid = 2;
    } else {
      if (distCache.rightValid()) {
        yDist = distCache.getRight() + (robotWidth / 2);
        numValid = 1;
      } else if (distCache.leftValid()) {
        yDist = distCache.getLeft() + (robotWidth / 2);
        numValid = 1;
      } else {
        numValid = 2;
      }
    }
    DoubleSupplier xSupplier;
    Supplier<Rotation2d> omegaSupplier;
    if (numValid == 2) {
      double y = yDist * directionMult;
      xSupplier = () -> y;
      // double t = theta;
      // if (!isBlueAlliance) {
      // t += Math.PI;
      // }
      // if (t > Math.PI) {
      // t = -(2 * (Math.PI) - t);
      // }
      // double o = t;
      double rot = climbPose.getRotation().getRadians();
      omegaSupplier = () -> new Rotation2d(rot);
      omegaPassed = true;
    } else if (numValid == 1) {
      double y = yDist * directionMult;
      xSupplier = () -> y;
      omegaSupplier = () -> new Rotation2d();
      omegaPassed = false;
    } else {
      omegaPassed = false;
      omegaSupplier = () -> new Rotation2d();
      xSupplier = () -> 0;
    }
    // Works for both alliances
    // Y
    DoubleSupplier ySupplier = () -> -0.1;

    joystickDriveAtAngleCustom(s_drive, xSupplier, ySupplier, omegaSupplier, omegaPassed);
  }
  /*
  * Logger.recordOutput("climb/Done Aligning To Start", doneAligningToStart);
   DoubleSupplier xSupplier, ySupplier;
   Supplier<Rotation2d> omegaSupplier;
   // Note these climb poses are not the poses of the actual climb structure, but the spot where we
   Pose2d currPose = s_dr[
   ive.getPose();
   // want to go to to make sure we are going in with good alignment
   // Blue alliance
   Rotation2d blueClimbRot = new Rotation2d(Math.PI); // Facing the alliance wall
   Rotation2d redClimbRot = new Rotation2d(0);
   // Note, these give a 2ft buffer dtstance
   Pose2d blueClimb = new Pose2d(Inches.of(41.56), Inches.of(35.845 + 13.5 + 48), blueClimbRot);
   Pose2d redClimb = new Pose2d(Inches.of(582.22), Inches.of(143.535 - 13.5 - 48), redClimbRot);
   // These are the y-coordinates of when we can then enable our climbing mechnism
   // 0.584 = 23 in in m. Give ourselves 1 inh away from the climb strut
   double blueTerminate = blueClimb.getY() + 0.584;
   double redTerminate = redClimb.getY() - 0.584;
   boolean isBlueAlliance = true;
   double directionMult = 1;
   if (isBlueAlliance) {
     directionMult = -1;
   }
   Pose2d climbPose = blueClimb;
   if (currPose.getX() > aprilTagLayout.getFieldLength() / 2) { // Far-side(red)
     climbPose = redClimb;
     isBlueAlliance = false;
   }
   Logger.recordOutput("climb/climbPose", climbPose);
   if (!omegaPassed1 || !xPassed) { // Use pose to get within 15 deg
     Rotation2d omega = new Rotation2d(climbPose.getRotation().getRadians());
     Rotation2d deltaOmega =
         new Rotation2d(
             currPose.getRotation().getRadians() - climbPose.getRotation().getRadians());
     omegaSupplier = () -> omega;
     omegaPassed1 = Math.abs(deltaOmega.getDegrees()) < 10 || omegaPassed1;

     double deltaX = -(currPose.getX() - climbPose.getX());
     xSupplier = () -> deltaX;
     Logger.recordOutput("climb/deltaX", deltaX);
     Logger.recordOutput("climb/xPassed", xPassed);

     xPassed = Math.abs(deltaX) < 0.3 || xPassed;
   } else { // Now we can use distance sensors for everything
     double distanceDeadspace = 10; // Distance deadspace of distance sensors
     double distanceOmega = -distCache.getDifference();
     double omegaPassing = 0; // in radians
     if (Math.abs(distanceOmega) > distanceDeadspace) {
       omegaPassing =
           distanceOmega
               / 10; // VERY subject to change, but likely will be some linear transformatoin
     }
     double o = omegaPassing;
     omegaSupplier = () -> new Rotation2d(o); // new Rotation2d(o);
     // Just for blue for now
     double avgDist = distCache.getResult();
     double deltaX = -(avgDist - climbPose.getX());
     Logger.recordOutput("climb/deltaX", deltaX);
     xSupplier = () -> deltaX; // deltaX;
   }
   if (!doneAligningToStart) { // For y
     // distances in meters
     double deltaY = -(currPose.getY() - climbPose.getY());
     if (yPassed) {
       deltaY = 0;
     }
     double y = deltaY;
     Logger.recordOutput("climb/deltaY", deltaY);
     ySupplier = () -> y;
     yPassed = Math.abs(deltaY) < 0.2 || yPassed;
     doneAligningToStart = yPassed && omegaPassed1;
   } else if ((isBlueAlliance && currPose.getY() >= blueTerminate)
       || (!isBlueAlliance && currPose.getY() <= redTerminate)) { // Going towards climb
     // Go towards climb, staying straight by distance sensors
     ySupplier = () -> -0.05;
   } else {
     xSupplier = () -> 0;
     ySupplier = () -> 0;
     omegaSupplier = () -> new Rotation2d();
     isDone = true;
   }
   Logger.recordOutput("climb/Y check passed", yPassed);
   Logger.recordOutput("climb/omega test passed", omegaPassed1);
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
