package frc.robot.telemetry.types;

import edu.wpi.first.math.geometry.Pose2d;

public class Pose2dEntry implements TelemetryEntry {
  private final DoubleArrayTelemetryEntry poseEntry;

  public Pose2dEntry(String path, boolean shouldNT) {
    poseEntry = new DoubleArrayTelemetryEntry(path, shouldNT);
  }

  double[] pose = new double[3];

  public void append(Pose2d value) {
    pose[0] = value.getTranslation().getX();
    pose[1] = value.getTranslation().getY();
    pose[2] = value.getRotation().getRadians();

    poseEntry.append(pose);
  }

  @Override
  public void close() {
    poseEntry.close();
  }
}
