package frc.robot.util.visualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static FlightTrajectory trajectory;
  private static Supplier<Pose3d> poseSupplier;

  public static void shootNote(Pose3d startPosition, double angle, double velocity) {
    trajectory = new FlightTrajectory(startPosition, angle, velocity);
  }

  public static void setRobotPoseSupplier(Supplier<Pose3d> poseSupplier) {
    NoteVisualizer.poseSupplier = poseSupplier;
  }

  public static Pose3d updateNotePosition(double dt) {
    trajectory.update(dt);

    Pose3d position = trajectory.getCurrentPosition();

    // Once trajectory goes below the ground, stop simulating it
    if (trajectory.getCurrentPosition().getTranslation().getZ() < 0) {
      trajectory = null;
    }
    drawNote(position);
    return position;
  }

  private static void drawNote(Pose3d pose) {
    Logger.recordOutput("Visualization/NotePosition", pose);

    // Rotate note 90 degrees around x axis to make it oriented the right way
    Pose3d rotatedPose =
        new Pose3d(
            pose.getTranslation(),
            pose.getRotation()
                .rotateBy(new Rotation3d(Math.PI / 2, 0, 0)));

    // Pose in robot space
    Pose3d robotPose = poseSupplier.get();
    Pose3d notePose = rotatedPose.relativeTo(robotPose);

    Logger.recordOutput("Visualization/NotePositionRobotSpace", notePose);
  }

  public static boolean isTrajectoryActive() {
    return trajectory != null;
  }
}
