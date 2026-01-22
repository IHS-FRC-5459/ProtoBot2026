// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.commands.DriveCommands.joystickDriveAtAngle;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAlign extends Command {
  Drive s_drive;
  /** Creates a new ShootAlign. */
  public ShootAlign(Drive s_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Note: I putpoosely didnt put dive in addRequirements because I dont want to lock out Rose
    // from driving translation-wise
    this.s_drive = s_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Note: Assumes CW is posotive andle, CCW is negative angle
    Pose2d currPose = s_drive.getPose();
    // Far-side(red)
    Pose2d hubPose = new Pose2d(Inches.of(182.11), Inches.of(158.84), new Rotation2d(0));
    if (currPose.getX() < aprilTagLayout.getFieldLength() / 2) { // Close side(blue)
      hubPose = new Pose2d(Inches.of(469.11), Inches.of(158.84), new Rotation2d(0));
    }
    double deltaX = -(currPose.getX() - hubPose.getX());
    double deltaY = currPose.getY() - hubPose.getY();
    Rotation2d desiredRot = new Rotation2d(-Math.atan(deltaY / deltaX));
    // x and y suppliers to be changed for future
    DoubleSupplier xSupplier = () -> 0;
    DoubleSupplier ySupplier = () -> 0;
    Supplier<Rotation2d> omegaSupplier = () -> desiredRot;
    Logger.recordOutput("DesiredRot", desiredRot.getDegrees());
    joystickDriveAtAngle(s_drive, null, null, omegaSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
