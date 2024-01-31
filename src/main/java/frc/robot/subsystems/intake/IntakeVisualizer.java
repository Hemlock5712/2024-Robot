package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class IntakeVisualizer {

  private final String logKey;

  public IntakeVisualizer(String logKey) {
    this.logKey = logKey;
  }

  Translation2d intakeTransform = new Translation2d(-0.358, 0.223);

  public void update(double intakeAngle) {
    Pose3d intakePose =
        new Pose3d(
            new Translation3d(intakeTransform.getX(), 0, intakeTransform.getY()),
            new Rotation3d(0, intakeAngle, 0));
    Pose3d magazinePose = new Pose3d(new Translation3d(), new Rotation3d());

    Logger.recordOutput("Mechanism3d/" + logKey, intakePose, magazinePose);
    Logger.recordOutput("Mechanism3d/" + logKey, intakePose, magazinePose);
  }
}
