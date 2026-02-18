// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import static frc.robot.Constants.Sensors.Distance.*;
import static frc.robot.commands.DriveCommands.joystickDriveRelativeCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DistanceCaching;
import frc.robot.subsystems.DistanceSide;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbAlign extends Command {
  Drive s_drive;
  Climb s_climb;
  DistanceSide sideDistCache;
  PIDController yPID = new PIDController(0.1, 0, 0);
  private final String loggingPrefix = "commands/climb/";
  /** Creates a new Climb. */
  // Climbs the right side of the climb structure(from the perspective of the alliance station)
  public ClimbAlign(Drive s_drive, Climb s_climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(s_drive);
    this.s_drive = s_drive;
    this.s_climb = s_climb;
    this.sideDistCache = s_climb.getDistanceSide();
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
  private boolean step1Done = false;
  // private boolean omegaPassed1, yPassed, xPassed = false;

  @Override
  public void execute() {
    Pose2d currPose = s_drive.getPose();
    ClimbParams climbParams = new ClimbParams(currPose);
    // DistanceCaching distCache = climbParams.getDistCache();
    DistanceCaching distCache = s_climb.getDistanceCacheBack();
    if (climbParams.getIsFront()) {
      distCache = s_climb.getDistanceCacheFront();
    }
    Logger.recordOutput(loggingPrefix + "goal", climbParams.getGoal());
    Logger.recordOutput(loggingPrefix + "leftfiltered", distCache.getLeftFiltered());
    // want to go to to make sure we are going in with good alignment
    // Blue alliance

    // Note, the 2ft buffer distance is now gone
    boolean shouldTurn = false;
    double directionMult = 1;
    Pose2d climbPose = climbParams.getGoal();
    Logger.recordOutput(loggingPrefix + "climbPose", climbPose);
    // X
    int numValidRangeMeasurements = 0; // Purposely doesnt use distCache to avoid runtime changes
    double xDist = 0;
    DoubleSupplier turnCommandSupplier = () -> 0;
    if (distCache.bothValid()) {
      Logger.recordOutput(loggingPrefix + "bothvalid", 2);
      xDist = distCache.getXDistance();
      numValidRangeMeasurements = 2;
      double rangeDiff = distCache.getDifference();
      shouldTurn = Math.abs(rangeDiff) > 0.005; // 0.005 is deadaspace
      double deltaSign = 1;
      if (rangeDiff < 0) {
        deltaSign = -1;
      }
      double deltaSign2 = deltaSign;
      double rangeDiff2 = rangeDiff;
      Logger.recordOutput(loggingPrefix + "rangeDiff", rangeDiff);
      Logger.recordOutput(loggingPrefix + "Leftvalid", distCache.leftMeasurementsValid());
      Logger.recordOutput(loggingPrefix + "rightvalid", distCache.rightMeasurementsValid());
      turnCommandSupplier =
          () ->
              MathUtil.clamp(Math.abs(rangeDiff2), 0.23, 0.7)
                  * deltaSign2
                  * climbParams.getOmegaMultiplier();
    } else {
      Logger.recordOutput(loggingPrefix + "bothvalid", 1);
      shouldTurn = false;
      if (distCache.rightMeasurementsValid()) {
        xDist = distCache.getRightFiltered();
        numValidRangeMeasurements = 1;
      } else if (distCache.leftMeasurementsValid()) {
        xDist = distCache.getLeftFiltered();
        numValidRangeMeasurements = 1;
      } else {
        Logger.recordOutput(loggingPrefix + "bothvalid", 0);
        numValidRangeMeasurements = 0;
      }
    }
    DoubleSupplier xSupplier;
    boolean xSkip = false;
    if (numValidRangeMeasurements == 2 || numValidRangeMeasurements == 1) {
      double x = (xDist - climbPose.getX());
      int xSign = 1;
      if (x < 0) {
        xSign = -1;
      }
      xSkip = Math.abs(x) < 0.012;
      Logger.recordOutput(loggingPrefix + "x", x);
      Logger.recordOutput(loggingPrefix + "xSkip", xSkip);
      x = MathUtil.clamp(Math.abs(x), 0.2, 0.5) * xSign;
      if (xSkip) {
        x = 0;
      }
      Logger.recordOutput(loggingPrefix + "math/diff", x);
      double xS = x;
      xSupplier = () -> xS * climbParams.getXMultiplier();
    } else {
      xSupplier = () -> 0;
    }
    // Works for both alliances
    // Y
    DoubleSupplier ySupplier;
    step1Done = (xSkip && !shouldTurn) || step1Done;
    if (!step1Done) {
      Logger.recordOutput(loggingPrefix + "step1Done", false);
      double deltaY = (currPose.getY() - climbPose.getY()) * -directionMult; // .4064=16in to m
      ySupplier = () -> MathUtil.clamp(-deltaY, -0.5, 0.5) * climbParams.getYMultiplier();
    } else {
      Logger.recordOutput(loggingPrefix + "step1Done", true);
      double deltaY = sideDistCache.getDistanceFiltered() * -directionMult; // .4064=16in to m
      if (Math.abs(deltaY) < 0.1) {
        deltaY = 0;
      }
      double sign = 1;
      if (deltaY < 0) {
        sign = -1;
      }
      double sign2 = sign;
      double deltaY2 = deltaY;
      double yPower =
          MathUtil.clamp(Math.abs(deltaY2), 0.2, 0.5) * climbParams.getStep2YMultiplier() * sign2;
      ySupplier = () -> yPower;
      Logger.recordOutput(loggingPrefix + "yPower", yPower);
      Logger.recordOutput(loggingPrefix + "yDone", false);
      if (deltaY == 0) {
        Logger.recordOutput(loggingPrefix + "yDone", true);
        isDone = true;
        ySupplier = () -> 0;
        Logger.recordOutput(loggingPrefix + "yPower", 0);
      }
    }
    ySupplier = () -> 0;
    turnCommandSupplier = () -> 0;
    // omegaPassed = omegaPassed && time < 100; // bad practice, but its fine :)
    joystickDriveRelativeCustom(s_drive, xSupplier, ySupplier, turnCommandSupplier, shouldTurn);
    Logger.recordOutput(loggingPrefix + "Xdist", xDist);
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
