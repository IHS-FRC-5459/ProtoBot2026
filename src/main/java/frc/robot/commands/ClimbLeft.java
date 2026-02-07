// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.Sensors.Distance.*;
import static frc.robot.commands.DriveCommands.joystickDriveRelativeCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DistanceCaching;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbLeft extends Command {
  DistanceCaching distCache;
  Drive s_drive;
  /** Creates a new Climb. */
  // Climbs the right side of the climb structure(from the perspective of the alliance station)
  public ClimbLeft(Drive s_drive, DistanceCaching distCache) {
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
    Rotation2d blueClimbRot = new Rotation2d(0); // Facing the alliance wall
    Rotation2d redClimbRot = new Rotation2d(Math.PI);
    // Note, the 2ft buffer distance is now gone
    Pose2d blueClimb = new Pose2d(Inches.of(40.625), Inches.of(173), blueClimbRot);
    Pose2d redClimb =
        new Pose2d(Inches.of(582.22), Inches.of(143.535 - 13.5 - 48 - 100), redClimbRot);
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
    double xDist = 0;
    DoubleSupplier omegaSupplier = () -> 0;
    if (distCache.bothValid()) {
      xDist = distCache.getResult();
      numValid = 2;
      double deltaOmega = distCache.getDifference();
      omegaPassed = Math.abs(deltaOmega) > 0.03; // 0.05 is deadaspace
      double deltaSign = 1;
      if (deltaOmega < 0) {
        deltaSign = -1;
      }
      double s = deltaSign;
      double o = deltaOmega;
      Logger.recordOutput("deltaOmega", deltaOmega);
      omegaSupplier = () -> MathUtil.clamp(Math.abs(o), 0.23, 0.7) * s;
    } else {
      omegaPassed = false;
      if (distCache.rightValid()) {
        xDist = distCache.getRight() + (robotWidth / 2);
        numValid = 1;
      } else if (distCache.leftValid()) {
        xDist = distCache.getLeft() + (robotWidth / 2);
        numValid = 1;
      } else {
        numValid = 0;
      }
    }
    DoubleSupplier xSupplier;
    if (numValid == 2 || numValid == 1) {
      double x = (xDist - climbPose.getX()) * directionMult;
      Logger.recordOutput("math/diff", x);
      xSupplier = () -> MathUtil.clamp(x, -0.7, 0.7);
    } else {
      xSupplier = () -> 0;
    }
    // Works for both alliances
    // Y
    double deltaY = (currPose.getY() - climbPose.getY()) * -directionMult; // .4064=16in to m
    DoubleSupplier ySupplier = () -> MathUtil.clamp(-deltaY, -1, 1);
    // omegaPassed = omegaPassed && time < 100; // bad practice, but its fine :)
    joystickDriveRelativeCustom(s_drive, xSupplier, ySupplier, omegaSupplier, omegaPassed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
