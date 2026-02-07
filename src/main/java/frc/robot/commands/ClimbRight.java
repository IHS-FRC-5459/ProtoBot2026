// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
// import static frc.robot.Constants.Sensors.Distance.*;
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
  int time = 0;

  @Override
  public void initialize() {
    time = 0;
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

    // Note, the 2ft buffer distance is now gone
    Pose2d blueClimb = new Pose2d(Inches.of(42), Inches.of(120.5), new Rotation2d());
    Pose2d redClimb =
        new Pose2d(Inches.of(582.22), Inches.of(143.535 - 13.5 - 48 - 100), new Rotation2d());
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
      xDist = distCache.getXDistance();
      numValid = 2;
      double deltaOmega = distCache.getDifference();
      omegaPassed = Math.abs(deltaOmega) > 0.08; // 0.05 is deadaspace
      double deltaSign = 1;
      if (deltaOmega < 0) {
        deltaSign = -1;
      }
      double s = deltaSign;
      double o = deltaOmega;
      Logger.recordOutput("deltaOmega", deltaOmega);
      Logger.recordOutput("Left valid", distCache.leftMeasurementsValid());
      Logger.recordOutput("right valid", distCache.rightMeasurementsValid());
      omegaSupplier = () -> MathUtil.clamp(Math.abs(o), 0.23, 0.7) * s;
    } else {
      omegaPassed = false;
      if (distCache.rightMeasurementsValid()) {
        xDist = distCache.getRightFiltered();
        numValid = 1;
      } else if (distCache.leftMeasurementsValid()) {
        xDist = distCache.getLeftFiltered();
        numValid = 1;
      } else {
        numValid = 0;
      }
    }
    DoubleSupplier xSupplier;
    if (numValid == 2 || numValid == 1) {
      double x = (xDist - climbPose.getX()) * directionMult;
      int xSign = 1;
      if (x < 0) {
        xSign = -1;
      }
      boolean skip = Math.abs(x) < 0.012;
      Logger.recordOutput("x", x);
      Logger.recordOutput("skip", skip);
      x = MathUtil.clamp(Math.abs(x), 0.2, 0.5) * xSign;
      if (skip) {
        x = 0;
      }
      Logger.recordOutput("math/diff", x);
      double xS = x;
      xSupplier = () -> xS;
    } else {
      xSupplier = () -> 0;
    }
    // Works for both alliances
    // Y
    double deltaY = (currPose.getY() - climbPose.getY()) * -directionMult; // .4064=16in to m
    DoubleSupplier ySupplier = () -> MathUtil.clamp(-deltaY, -1, 1);
    // omegaPassed = omegaPassed && time < 100; // bad practice, but its fine :)
    joystickDriveRelativeCustom(s_drive, xSupplier, ySupplier, omegaSupplier, omegaPassed);
    Logger.recordOutput("Xdist", xDist);
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
