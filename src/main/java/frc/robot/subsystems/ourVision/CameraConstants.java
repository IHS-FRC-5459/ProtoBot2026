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
        "left",
        new CameraConstants(
            LeftCam.kCameraName,
            LeftCam.kMultiTagStdDevs,
            LeftCam.kRobotToCam,
            LeftCam.kSingleTagStdDevs));
    cameras.put(
        "right",
        new CameraConstants(
            RightCam.kCameraName,
            RightCam.kMultiTagStdDevs,
            RightCam.kRobotToCam,
            RightCam.kSingleTagStdDevs));
  }

  public static class LeftCam {
    public static final String kCameraName = "left";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.33655, 0.2587625, 0.2778125),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(-15)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }

  public static class RightCam {
    public static final String kCameraName = "right";
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.358775, -0.2682875, 0.27305),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(15)));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  }
}
