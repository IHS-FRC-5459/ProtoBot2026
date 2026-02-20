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
  private final double xFFKs = 0.12;
  private final double omegaFFKs = 0.15;
  private final double omegaPIDI = 0;
  PIDController yPID = new PIDController(0.6, 0.03, 0.1);
  PIDController xPID = new PIDController(1.5, 0.02, 0.03);
  PIDController omegaPID = new PIDController(2.5, omegaPIDI, 0.02);
  SimpleMotorFeedforward xFF = new SimpleMotorFeedforward(xFFKs, 0, 0);
  SimpleMotorFeedforward omegaFF = new SimpleMotorFeedforward(omegaFFKs, 0, 0);
  SimpleMotorFeedforward yFF = new SimpleMotorFeedforward(0.1, 0, 0);
  private final double stoppingDist = 0.28;
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
  boolean isFirstTime = false;
  long startOfTransition = 0;

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
    isFirstTime = true;
    state = 0;
    xFF.setKs(xFFKs);
    omegaFF.setKs(omegaFFKs);
    omegaPID.setI(omegaPIDI);
    // doneAligningToStart = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private boolean isDone = false;
  // 0 = x & omega   1 = turn wheels to 90deg   2 = y
  private int state = 0;
  // private boolean omegaPassed1, yPassed, xPassed = false;

  @Override
  public void execute() {
    double passingX, passingY, passingOmega;
    // xPID.setP(SmartDashboard.getNumber("xPID_P", 0.1));
    // xPID.setI(SmartDashboard.getNumber("xPID_I", 0));
    // xPID.setD(SmartDashboard.getNumber("xPID_D", 0));
    // xFF.setKs(SmartDashboard.getNumber("xFF_S", 0));
    // yPID.setP(SmartDashboard.getNumber("yPID_P", 0.1));
    // yPID.setI(SmartDashboard.getNumber("yPID_I", 0));
    // yPID.setD(SmartDashboard.getNumber("yPID_D", 0));
    // yFF.setKs(SmartDashboard.getNumber("yFF_S", 0));
    // omegaPID.setP(SmartDashboard.getNumber("omegaPID_P", 0.1));
    // omegaPID.setI(SmartDashboard.getNumber("omegaPID_I", 0));
    // omegaPID.setD(SmartDashboard.getNumber("omegaPID_D", 0));
    // omegaFF.setKs(SmartDashboard.getNumber("omegaFF_S", 0));
    // boolean xEnabled = SmartDashboard.getBoolean("xEnabled", false);
    // boolean yEnabled = SmartDashboard.getBoolean("yEnabled", false);
    // boolean omegaEnabled = SmartDashboard.getBoolean("omegaEnabled", false);
    // stoppingDist = SmartDashboard.getNumber("stoppingDist", stoppingDist);
    boolean xEnabled = true;
    boolean yEnabled = true;
    boolean omegaEnabled = true;
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
    boolean xSkip = false;
    if (numValidRangeMeasurements == 2 || numValidRangeMeasurements == 1) {
      double x = (xDist - climbPose.getX());
      xSkip = Math.abs(x) <= 0.012;
      Logger.recordOutput(loggingPrefix + "x", x);
      passingX = x * climbParams.getXMultiplier();
    } else {
      passingX = 0;
    }
    // Works for both alliances
    //
    DoubleSupplier ySupplier;
    // xSkip && !shouldTurn
    boolean doneFirstStage = xSkip && !shouldTurn;
    Logger.recordOutput(loggingPrefix + "doneFirstStage", doneFirstStage);
    Logger.recordOutput(loggingPrefix + "startOfTransition", startOfTransition);
    Logger.recordOutput(
        loggingPrefix + "elapsedTime", System.currentTimeMillis() - startOfTransition);
    if (state == 2 || (state == 1 && System.currentTimeMillis() - startOfTransition > 1000)) {
      state = 2;
    } else if (doneFirstStage || state == 1) {
      state = 1;
      if (isFirstTime) {
        isFirstTime = false;
        startOfTransition = System.currentTimeMillis();
      }
    }
    // step1Done = SmartDashboard.getBoolean("step1Done", true);
    Logger.recordOutput(loggingPrefix + "state", state);
    passingY = 0;
    if (state == 1) {
      passingX = 0;
      passingOmega = 0;
      passingY = 0; // Set later
    } else if (state == 2) {
      passingX = 0;
      passingOmega = 0;
      Logger.recordOutput(loggingPrefix + "step1Done", true);
      double deltaY = sideDistCache.getDistanceFiltered() * -directionMult; // .4064=16in to m
      passingY = -deltaY * climbParams.getStep2YMultiplier();
      Logger.recordOutput(loggingPrefix + "yDone", false);
      if (Math.abs(deltaY) <= stoppingDist) {
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
    Logger.recordOutput(loggingPrefix + "controllers/y/pidVolts", 0);
    Logger.recordOutput(loggingPrefix + "controllers/y/ffVolts", 0);
    ySupplier = () -> pidVoltsY + ffVoltsY;
    if (state == 1) {
      ySupplier = () -> 0.1;
    }
    if (!yEnabled) {
      ySupplier = () -> 0;
    }
    if (!xEnabled) {
      xSupplier = () -> 0;
    }
    if (!omegaEnabled) {
      turnCommandSupplier = () -> 0;
    }

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
