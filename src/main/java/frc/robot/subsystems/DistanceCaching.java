// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class DistanceCaching extends SubsystemBase {
  TimeOfFlight left, right;
  Queue<Double> leftQ = new LinkedList<>();
  Queue<Double> rightQ = new LinkedList<>();
  int badLeftNum = 0;
  int badRightNum = 0;
  private final int queueSize = 10;
  private final int maxBadMeasurements = 20;
  /** Creates a new DistanceCaching. */
  public DistanceCaching() {
    left = new TimeOfFlight(Constants.Sensors.Distance.leftId);
    right = new TimeOfFlight(Constants.Sensors.Distance.rightId);
  }

  public double getResult() {
    if (badLeftNum < maxBadMeasurements && badRightNum < maxBadMeasurements) {
      return (getRight() / 1000 + getLeft() / 1000) / 2;
    }
    return Math.max(getRight() / 1000, getLeft() / 1000);
  }

  public double getDifference() {
    if (badLeftNum < maxBadMeasurements && badRightNum < maxBadMeasurements) {
      return getLeft() / 1000 - getRight() / 1000;
    }
    return 0;
  }

  public double getLeft() {
    double total = 0;
    int numZeroes = 0;
    for (double d : leftQ) {
      total += d;
      if (d == 0) {
        badLeftNum++;
        numZeroes++;
      } else {
        badLeftNum = 0;
      }
    }
    if (total == 0) {
      return -1;
    }
    return total / (leftQ.size() - numZeroes);
  }

  public double getRight() {
    double total = 0;
    int numZeroes = 0;
    for (double d : rightQ) {
      total += d;
      if (d == 0) {
        badRightNum++;
        numZeroes++;
      } else {
        badRightNum = 0;
      }
    }
    if (total == 0) {
      return -1;
    }
    return total / (rightQ.size() - numZeroes);
  }

  public boolean leftValid() {
    return getLeft() != -1;
  }

  public boolean rightValid() {
    return getRight() != -1;
  }

  public boolean bothValid() {
    return leftValid() && rightValid();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftQ.add(left.getRange());
    if (leftQ.size() > queueSize) {
      leftQ.remove();
    }
    rightQ.add(right.getRange());
    if (rightQ.size() > queueSize) {
      rightQ.remove();
    }
    Logger.recordOutput("distance/left", getLeft());
    Logger.recordOutput("distance/right", getRight());
    Logger.recordOutput("distance/avg", getResult());
  }
}
