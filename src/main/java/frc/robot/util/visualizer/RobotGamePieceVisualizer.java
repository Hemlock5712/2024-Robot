package frc.robot.util.visualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotGamePieceVisualizer {

  private static Supplier<Pose3d> poseSupplier;
  private static Supplier<Transform3d> armTransformSupplier;
  private static DoubleSupplier shooterAngleSupplier;
  private static BooleanSupplier isMagazineLoadedSupplier;
  private static BooleanSupplier isShooterLoadedSupplier;

  public static void setRobotPoseSupplier(Supplier<Pose3d> poseSupplier) {
    RobotGamePieceVisualizer.poseSupplier = poseSupplier;
  }

  public static void setArmTransformSupplier(Supplier<Transform3d> armTransformSupplier) {
    RobotGamePieceVisualizer.armTransformSupplier = armTransformSupplier;
  }

  public static void setShooterAngleSupplier(DoubleSupplier shooterAngleSupplier) {
    RobotGamePieceVisualizer.shooterAngleSupplier = shooterAngleSupplier;
  }

  public static void setIsMagazineLoadedSupplier(BooleanSupplier isMagazineLoadedSupplier) {
    RobotGamePieceVisualizer.isMagazineLoadedSupplier = isMagazineLoadedSupplier;
  }

  public static void setIsShooterLoadedSupplier(BooleanSupplier isShooterLoadedSupplier) {
    RobotGamePieceVisualizer.isShooterLoadedSupplier = isShooterLoadedSupplier;
  }

  public static void drawGamePieces() {
    List<Pose3d> gamePiecePoses = new ArrayList<>();

    // Draw the game piece in the intake portion
    if (isMagazineLoadedSupplier.getAsBoolean()) {
      gamePiecePoses.add(
          poseSupplier
              .get()
              .plus(
                  new Transform3d(
                      new Pose3d() {},
                      new Pose3d(
                          -.234, 0, 0.212, new Rotation3d(0, Units.degreesToRadians(-13.7), 0)))));
    } else {
      gamePiecePoses.add(new Pose3d() {});
    }

    // // Draw the game piece in the shooter portion
    if (isShooterLoadedSupplier.getAsBoolean()) {
      Pose3d armPose = poseSupplier.get().transformBy(armTransformSupplier.get());

      gamePiecePoses.add(armPose);
    } else {
      gamePiecePoses.add(new Pose3d() {});
    }

    // Draw the game pieces
    Logger.recordOutput("Visualization/GamePiecePoses", gamePiecePoses.toArray(new Pose3d[0]));
  }
}
