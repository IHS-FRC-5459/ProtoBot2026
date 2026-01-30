// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.commands.DriveCommands.joystickDriveAtAngleCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbRight extends Command {
  TimeOfFlight leftDistSensor, rightDistSensor;
  Drive s_drive;
  /** Creates a new Climb. */
  // Climbs the right side of the climb structure(from the perspective of the alliance station)
  public ClimbRight(Drive s_drive, TimeOfFlight leftDistSensor, TimeOfFlight rightDistSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(s_drive);
    this.s_drive = s_drive;
    this.leftDistSensor = leftDistSensor;
    this.rightDistSensor = rightDistSensor;
  }

  // Called when the command is initially scheduled.
  private boolean doneAligningToStart = false;

  @Override
  public void initialize() {
    setIsFirstCall(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  private boolean isDone = false;

  @Override
  public void execute() {
    DoubleSupplier xSupplier, ySupplier;
    Supplier<Rotation2d> omegaSupplier;
    // Note these climb poses are not the poses of the actual climb structure, but the spot where we
    Pose2d currPose = s_drive.getPose();
    // want to go to to make sure we are going in with good alignment
    // Blue alliance
    Rotation2d blueClimbRot = new Rotation2d(Math.PI); // Facing the alliance wall
    Rotation2d redClimbRot = new Rotation2d(0);
    // Note, these give a 2ft buffer dtstance
    Pose2d blueClimb = new Pose2d(Inches.of(41.56), Inches.of(35.845), blueClimbRot);
    Pose2d redClimb = new Pose2d(Inches.of(582.22), Inches.of(143.535), redClimbRot);
    // These are the y-coordinates of when we can then enable our climbing mechnism
    // 0.584 = 23 in in m. Give ourselves 1 inh away from the climb strut
    double blueTerminate = blueClimb.getY() + 0.584;
    double redTerminate = redClimb.getY() - 0.584;
    boolean isBlueAlliance = true;
    // y = ___ = (end of climb) - (robot width) - (2 ft/ 24 in)
    Pose2d climbPose = blueClimb;
    if (currPose.getX() > aprilTagLayout.getFieldLength() / 2) { // Far-side(red)
      climbPose = redClimb;
      isBlueAlliance = false;
    }
    // Get to approximate starting location. Note that we don't use vision for yaw align(instead use
    // distance sensors)
    if (!doneAligningToStart) {
      // distances in meters
      double deltaX = currPose.getX() - climbPose.getX();
      double deltaY = currPose.getX() - climbPose.getX();

      // Angle calculations(in radians) (use vision pose)
      Rotation2d deltaOmega =
          new Rotation2d(
              currPose.getRotation().getRadians() - climbPose.getRotation().getRadians());
      xSupplier = () -> deltaX;
      ySupplier = () -> deltaY;
      omegaSupplier =
          () ->
              new Rotation2d(
                  deltaOmega.getRadians()
                      / 10); // Once again, VERY subject to change, but likely will be a linear
      // transofrmation for our purposes to keep things simple
      // 0.0508 = 2 in in
      doneAligningToStart =
          Math.abs(deltaX) < 0.0508
              && Math.abs(deltaY) < 0.0508
              && Math.abs(deltaOmega.getDegrees()) < 5;
    } else if ((isBlueAlliance && currPose.getY() >= blueTerminate)
        || (!isBlueAlliance && currPose.getY() <= redTerminate)) {
      // Go towards climb, staying straight by distance sensors
      ySupplier = () -> 0;
      xSupplier = () -> 0.01; // constant, we want to go slow, no pid controller
      double distanceDeadspace = 10; // Distance deadspace of distance sensors
      double distanceOmega = leftDistSensor.getRange() - rightDistSensor.getRange();
      double omegaPassing = 0; // in radians
      if (Math.abs(distanceOmega) > distanceDeadspace) {
        omegaPassing =
            distanceOmega
                / 100.; // VERY subject to change, but likely will be some linear transformatoin
      }
      double o = omegaPassing;
      omegaSupplier = () -> new Rotation2d(o);
    } else {
      xSupplier = () -> 0;
      ySupplier = () -> 0;
      omegaSupplier = () -> new Rotation2d();
      isDone = true;
    }
    joystickDriveAtAngleCustom(s_drive, xSupplier, ySupplier, omegaSupplier);
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
