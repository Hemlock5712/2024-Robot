package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.Objects;

/**
 * The VisionHelpers class provides utility methods and record classes for vision-related
 * operations.
 */
public class VisionHelpers {

  /**
   * Represents a pose estimate with additional information.
   *
   * @param pose The pose (position and orientation) estimate.
   * @param timestampSeconds The timestamp in seconds when the pose estimate was recorded.
   * @param averageTagDistance The average distance to the detected tags.
   * @param tagIDs The IDs of the detected tags.
   */
  public record PoseEstimate(
      /** The pose (position and orientation) estimate. */
      Pose2d pose,
      /** The timestamp in seconds when the pose estimate was recorded. */
      double timestampSeconds,
      /** The average distance to the detected tags. */
      double averageTagDistance,
      /** The IDs of the detected tags. */
      int tagCount) {

    /**
     * Checks if this pose estimate is equal to another object.
     *
     * @param obj The object to compare.
     * @return True if the objects are equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null || getClass() != obj.getClass()) {
        return false;
      }
      PoseEstimate other = (PoseEstimate) obj;
      return Double.compare(tagCount, other.tagCount) == 0
          && Objects.equals(pose, other.pose)
          && Double.compare(timestampSeconds, other.timestampSeconds) == 0
          && Double.compare(averageTagDistance, other.averageTagDistance) == 0;
    }

    /**
     * Returns a string representation of this pose estimate.
     *
     * @return The string representation.
     */
    @Override
    public String toString() {
      return "PoseEstimate{"
          + "pose="
          + pose.toString()
          + ", timestampSeconds="
          + Double.toString(timestampSeconds)
          + ", averageTagDistance="
          + Double.toString(averageTagDistance)
          + ", tagIDs="
          + Double.toString(tagCount)
          + '}';
    }
  }

  /**
   * Converts a Pose3d object to an array of doubles.
   *
   * @param pose The Pose3d object to convert.
   * @return The array of doubles representing the pose.
   */
  public static double[] getPose3dToArray(Pose3d pose) {
    double[] result = new double[6];
    result[0] = pose.getTranslation().getX();
    result[1] = pose.getTranslation().getY();
    result[2] = pose.getTranslation().getZ();
    result[3] = Units.radiansToDegrees(pose.getRotation().getX());
    result[4] = Units.radiansToDegrees(pose.getRotation().getY());
    result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
    return result;
  }

  public static double[] getPose2dToArray(Pose2d pose) {
    double[] result = new double[6];
    result[0] = pose.getTranslation().getX();
    result[1] = pose.getTranslation().getY();
    result[2] = 0;
    result[3] = Units.radiansToDegrees(0);
    result[4] = Units.radiansToDegrees(0);
    result[5] = Units.radiansToDegrees(pose.getRotation().getRadians());
    return result;
  }

  /**
   * Represents a timestamped vision update with pose and standard deviations.
   *
   * @param timestamp The timestamp of the vision update.
   * @param pose The pose estimate.
   * @param stdDevs The standard deviations matrix.
   */
  public record TimestampedVisionUpdate(
      /** The timestamp of the vision update. */
      double timestamp,
      /** The pose estimate. */
      Pose2d pose,
      /** The standard deviations matrix. */
      Matrix<N3, N1> stdDevs) {

    /**
     * Returns a string representation of this vision update.
     *
     * @return The string representation.
     */
    @Override
    public String toString() {
      return "VisionUpdate{"
          + "timestamp="
          + Double.toString(timestamp)
          + ", pose="
          + pose.toString()
          + ", stdDevs="
          + stdDevs.toString()
          + '}';
    }
  }
}
