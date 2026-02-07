// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Vision {
    public static final String[] cameraNames = {"left", "right"};
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()
        };

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(moduleTranslations);
    public static final SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
  }

  public static final String canbus = "rio";

  public static class Sensors {
    public static class Pigeon {
      public static final int id = 13;
    }

    public static class Distance {
      public static final int frontLeftId = 20;
      public static final int frontRightId = 21;
      public static final double xRobotOffsetFront = 0.3302; // 9 in in m
      public static final int backLeftId = 22;
      public static final int backRightId = 23;
      public static final double xRobotOffsetBack = 0.12065; // 4 3/4 in in m
    }
  }

  public static class Motors {
    public static final int intakeId = 15;
    public static final int outtakeId = 14;
    public static final int climbId = 17;
  }
}
