// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class DistanceCaching extends SubsystemBase {
  TimeOfFlight left, right;
  Queue<Double> leftDistancesQ = new LinkedList<>();
  Queue<Double> rightDistanceQ = new LinkedList<>();
  private final int queueSize = 5;
  int leftId, rightId;
  double offset;
  String direction;
  private String loggingPrefix = "";
  /** Creates a new DistanceCaching. */
  // offset is the always posotive x translation of the distance sensors to the centerline (x=0
  // line)
  public DistanceCaching(int leftId, int rightId, double offset, String direction) {
    loggingPrefix = "subsystems/distanceCaching/" + direction + "/";
    left = new TimeOfFlight(leftId);
    right = new TimeOfFlight(rightId);
    left.setRangingMode(RangingMode.Short, 24);
    right.setRangingMode(RangingMode.Short, 24);
    this.leftId = leftId;
    this.rightId = rightId;
    this.offset = offset;
    this.direction = direction;
  }

  public double getXDistance() {
    return ((getRightFiltered() + getLeftFiltered()) / 2 + offset);
  }

  public double getDifference() {
    return getRightFiltered() - getLeftFiltered();
  }

  public double getLeftFiltered() {
    double sumOfDistanceMeasurements = 0;
    int numZeroes = 0;
    for (double distanceMeasurement : leftDistancesQ) {
      sumOfDistanceMeasurements += distanceMeasurement;
      if (distanceMeasurement == 0) {
        numZeroes++;
      }
    }
    if (sumOfDistanceMeasurements == 0) {
      return -1;
    }
    return (sumOfDistanceMeasurements / (leftDistancesQ.size() - numZeroes)) / 1000;
  }

  public double getRightFiltered() {
    double sumOfDistancesMeasurements = 0;
    int numZeroes = 0;
    for (double distanceMeasurements : rightDistanceQ) {
      sumOfDistancesMeasurements += distanceMeasurements;
      if (distanceMeasurements <= 0) {
        numZeroes++;
      }
    }
    if (sumOfDistancesMeasurements == 0) {
      return -1;
    }
    return (sumOfDistancesMeasurements / (rightDistanceQ.size() - numZeroes)) / 1000;
  }

  public boolean leftMeasurementsValid() {
    return getLeftFiltered() != -1;
  }

  public boolean rightMeasurementsValid() {
    return getRightFiltered() != -1;
  }

  public boolean bothValid() {
    return leftMeasurementsValid() && rightMeasurementsValid();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    double leftRawDistance = left.getRange() - 30;
    leftDistancesQ.add(leftRawDistance);
    if (leftDistancesQ.size() > queueSize) {
      leftDistancesQ.remove();
    }
    double rightRawDistance = right.getRange() - 30;
    rightDistanceQ.add(rightRawDistance);
    if (rightDistanceQ.size() > queueSize) {
      rightDistanceQ.remove();
    }
    Logger.recordOutput(loggingPrefix + "rawdistancesleftdistsensor", leftRawDistance);
    Logger.recordOutput(loggingPrefix + "rawdistancesrightdistsensor", rightRawDistance);
    Logger.recordOutput(loggingPrefix + "left", getLeftFiltered());
    Logger.recordOutput(loggingPrefix + "right", getRightFiltered());
    Logger.recordOutput(loggingPrefix + "avg", getXDistance());
    Logger.recordOutput(loggingPrefix + "getSampletimeleft", left.getSampleTime());
    Logger.recordOutput(loggingPrefix + "isleftvalid", leftMeasurementsValid());
    Logger.recordOutput(loggingPrefix + "isrightvalid", rightMeasurementsValid());
  }
}
