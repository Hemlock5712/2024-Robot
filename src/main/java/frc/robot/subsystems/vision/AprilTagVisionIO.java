// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.VisionHelpers;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  class AprilTagVisionIOInputs implements LoggableInputs {

    ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

    @Override
    public void toLog(LogTable table) {
      table.put("poseEstimates", poseEstimates.size());
      for (PoseEstimate poseEstimate : poseEstimates) {
        int posePosition = poseEstimates.indexOf(poseEstimate);
        table.put(
            "estimatedPose/" + Integer.toString(posePosition),
            VisionHelpers.getPose2dToArray(poseEstimate.pose()));
        table.put(
            "captureTimestamp/" + Double.toString(posePosition), poseEstimate.timestampSeconds());
        table.put("tagIDs/" + Double.toString(posePosition), poseEstimate.tagCount());
        table.put(
            "averageTagDistance/" + Double.toString(posePosition),
            poseEstimate.averageTagDistance());
      }
      table.put("valid", !poseEstimates.isEmpty());
    }

    @Override
    public void fromLog(LogTable table) {
      int estimatedPoseCount = table.get("poseEstimates", 0);
      for (int i = 0; i < estimatedPoseCount; i++) {
        Pose2d poseEstimation =
            LimelightHelpers.toPose2D(
                table.get("estimatedPose/" + Integer.toString(i), new double[] {}));
        double timestamp = table.get("captureTimestamp/" + Double.toString(i), 0.0);
        double averageTagDistance = table.get("averageTagDistance/" + Double.toString(i), 0.0);
        int tagCount = table.get("tagCount/" + Integer.toString(i), 0);
        poseEstimates.add(
            new PoseEstimate(poseEstimation, timestamp, averageTagDistance, tagCount));
      }
      table.get("valid", false);
    }
  }

  default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
