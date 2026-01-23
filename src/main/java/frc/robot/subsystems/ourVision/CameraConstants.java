package frc.robot.subsystems.ourVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.HashMap;

public class CameraConstants {
  public final String kCameraName;
  public final Matrix<N3, N1> kMultiTagStdDevs;
  public final Transform3d kRobotToCam;
  public final Matrix<N3, N1> kSingleTagStdDevs;

  public static final HashMap<String, CameraConstants> cameras =
      new HashMap<String, CameraConstants>();

  public CameraConstants(
      String kCameraName,
      Matrix<N3, N1> kMultiTagStdDevs,
      Transform3d kRobotToCam,
      Matrix<N3, N1> kSingleTagStdDevs) {
    this.kCameraName = kCameraName;
    this.kMultiTagStdDevs = kMultiTagStdDevs;
    this.kRobotToCam = kRobotToCam;
    this.kSingleTagStdDevs = kSingleTagStdDevs;
  }

  static {
    cameras.put(
        "frontleft",
        new CameraConstants(
            FrontLeftCam.kCameraName,
            FrontLeftCam.kMultiTagStdDevs,
            FrontLeftCam.kRobotToCam,
            FrontLeftCam.kSingleTagStdDevs));
    cameras.put(
        "frontright",
        new CameraConstants(
            FrontRightCam.kCameraName,
            FrontRightCam.kMultiTagStdDevs,
            FrontRightCam.kRobotToCam,
            FrontRightCam.kSingleTagStdDevs));
    cameras.put(
        "backleft",
        new CameraConstants(
            BackLeftCam.kCameraName,
            BackLeftCam.kMultiTagStdDevs,
            BackLeftCam.kRobotToCam,
            BackLeftCam.kSingleTagStdDevs));
    cameras.put(
        "backright",
        new CameraConstants(
            BackRightCam.kCameraName,
            BackRightCam.kMultiTagStdDevs,
            BackRightCam.kRobotToCam,
            BackRightCam.kSingleTagStdDevs));
  }

  public static class FrontLeftCam {
    public static final String kCameraName = "frontleft";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.27225625, 0.235, 0.259),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(-15)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }

  public static class FrontRightCam {
    public static final String kCameraName = "frontright";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.28654375, -0.222504, 0.257175),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(15)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }

  public static class BackLeftCam {
    public static final String kCameraName = "backleft";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(-0.28813125, 0.23495, 0.273812),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(15)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }

  public static class BackRightCam {
    public static final String kCameraName = "backright";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(-0.28829, -0.21351875, 0.2667),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(15)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }
}
