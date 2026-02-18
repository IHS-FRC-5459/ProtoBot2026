// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import static frc.robot.Constants.Sensors.Distance.*;
import static frc.robot.commands.DriveCommands.joystickDriveRelativeCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  PIDController xPID = new PIDController(2.5, 0.01, 0.03);
  PIDController omegaPID = new PIDController(2.5, 0, 0.1);
  SimpleMotorFeedforward xFF = new SimpleMotorFeedforward(0.12, 0, 0);
  SimpleMotorFeedforward omegaFF = new SimpleMotorFeedforward(0.15, 0, 0);
  SimpleMotorFeedforward yFF = new SimpleMotorFeedforward(0, 0, 0);

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
    xPID.reset();
    yPID.reset();
    omegaPID.reset();
    // doneAligningToStart = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private boolean isDone = false;
  private boolean step1Done = false;
  // private boolean omegaPassed1, yPassed, xPassed = false;

  @Override
  public void execute() {
    double passingX, passingY, passingOmega;
    yPID.setP(SmartDashboard.getNumber("yPID_P", 0.1));
    yPID.setI(SmartDashboard.getNumber("yPID_I", 0));
    yPID.setD(SmartDashboard.getNumber("yPID_D", 0));
    yFF.setKs(SmartDashboard.getNumber("yFF_S", 0));
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
      Logger.recordOutput(loggingPrefix + "rangeDiff", rangeDiff);
      Logger.recordOutput(loggingPrefix + "Leftvalid", distCache.leftMeasurementsValid());
      Logger.recordOutput(loggingPrefix + "rightvalid", distCache.rightMeasurementsValid());
      passingOmega = rangeDiff * climbParams.getOmegaMultiplier();
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
      passingOmega = 0;
    }
    DoubleSupplier xSupplier;
    if (numValidRangeMeasurements == 2 || numValidRangeMeasurements == 1) {
      double x = (xDist - climbPose.getX());
      Logger.recordOutput(loggingPrefix + "x", x);
      passingX = x * climbParams.getXMultiplier();
    } else {
      passingX = 0;
    }
    // Works for both alliances
    // Y
    DoubleSupplier ySupplier;
    Logger.recordOutput(loggingPrefix + "shouldTurn", shouldTurn);
    step1Done = (Math.abs(passingX) < 0.005 && !shouldTurn) || step1Done;
    step1Done = true;
    if (!step1Done) {
      Logger.recordOutput(loggingPrefix + "step1Done", false);
      double deltaY = (currPose.getY() - climbPose.getY()) * -directionMult; // .4064=16in to m
      passingY = 0; // -deltaY * climbParams.getYMultiplier();
    } else {
      Logger.recordOutput(loggingPrefix + "step1Done", true);
      double deltaY = sideDistCache.getDistanceFiltered() * -directionMult; // .4064=16in to m
      passingY = -deltaY * climbParams.getStep2YMultiplier();
      Logger.recordOutput(loggingPrefix + "yDone", false);
      if (deltaY == 0) {
        Logger.recordOutput(loggingPrefix + "yDone", true);
        isDone = true;
        passingY = 0;
      }
    }

    double pidVoltsOmega = omegaPID.calculate(passingOmega, 0) * -1;
    double ffVoltsOmega = omegaFF.calculate(passingOmega, 0);
    Logger.recordOutput(loggingPrefix + "controllers/omega/pidVolts", pidVoltsOmega);
    Logger.recordOutput(loggingPrefix + "controllers/omega/ffVolts", ffVoltsOmega);
    turnCommandSupplier = () -> pidVoltsOmega + ffVoltsOmega;

    double pidVoltsX = xPID.calculate(passingX, 0);
    double ffVoltsX = xFF.calculate(passingX, 0) * -1;
    Logger.recordOutput(loggingPrefix + "controllers/x/pidVolts", pidVoltsX);
    Logger.recordOutput(loggingPrefix + "controllers/x/ffVolts", ffVoltsX);
    xSupplier = () -> pidVoltsX + ffVoltsX;

    double pidVoltsY = yPID.calculate(passingY, 0);
    double ffVoltsY = yFF.calculate(passingY, 0) * -1;
    Logger.recordOutput(loggingPrefix + "controllers/y/pidVolts", pidVoltsY);
    Logger.recordOutput(loggingPrefix + "controllers/y/ffVolts", ffVoltsY);
    ySupplier = () -> pidVoltsY + ffVoltsY;

    Logger.recordOutput(loggingPrefix + "passing/xPassing", passingX);
    Logger.recordOutput(loggingPrefix + "passing/omegaPassing", passingOmega);
    Logger.recordOutput(loggingPrefix + "passing/yPassing", passingY);

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
