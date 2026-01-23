// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSub;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private OuttakeSub s_outtake;
  private Drive s_drive;
  /** Creates a new Outtake. */
  public Shoot(OuttakeSub s_outtake, Drive s_drive) {
    // Only uding drive for pos, so dont add req
    addRequirements(s_outtake);
    this.s_outtake = s_outtake;
    this.s_drive = s_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Shoot/Shooter volts", 0);
    Logger.recordOutput("Shoot/Desired shooter volts", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Currently, calculates velocity --> power through assuming fixed angle
    double g = 9.8; // Gravity
    /*
    * with np.errstate(divide='ignore', invalid='ignore'):
       denominator = 2 * np.cos(A_ngle)**2 * (Cx * np.tan(A_ngle) - Cy)
       if denominator <= 0:
           return None
       return np.sqrt(g * Cx**2 / denominator)
    */
    double fixedAng = new Rotation2d(76 * (Math.PI / 180)).getDegrees();
    Pose2d currPose = s_drive.getPose();
    // Far-side(red)
    Pose2d hubPose = new Pose2d(Inches.of(182.11), Inches.of(158.84), new Rotation2d(0));
    if (currPose.getX() < aprilTagLayout.getFieldLength() / 2) { // Close side(blue)
      hubPose = new Pose2d(Inches.of(469.11), Inches.of(158.84), new Rotation2d(0));
    }
    double deltaX = currPose.getX() - hubPose.getX();
    double deltaY = currPose.getY() - hubPose.getY();
    double deltaGround = (Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    double deltaZ = 1.3716; // Hub is 6'6'', robot shooter is 1'6''
    double denominator =
        2 * Math.pow((Math.cos(fixedAng)), 2) * (deltaGround * Math.tan(fixedAng) - deltaZ);
    if (denominator <= 0) {
      return;
    }
    double reqVelocity = Math.sqrt((g * Math.pow(deltaGround, 2)) / denominator);
    Logger.recordOutput("Shoot/req velocity", reqVelocity);

    double reqVoltage = 4 * reqVelocity; // Very bad conversion, TODO: Change
    Logger.recordOutput("Shoot/Shooter volts", (int) MathUtil.clamp(reqVoltage, 0.0, 12.0));
    Logger.recordOutput("Shoot/Desired shooter volts", (int) reqVoltage);
    s_outtake.setVoltage(MathUtil.clamp(reqVoltage, 0.0, 12.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Shoot/Shooter volts", 0);
    Logger.recordOutput("Shoot/Desired shooter volts", 0);

    s_outtake.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
