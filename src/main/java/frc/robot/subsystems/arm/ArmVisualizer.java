package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {
  private final String logKey;

  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechanismRoot;
  private final MechanismLigament2d fixedLigament;
  private final MechanismLigament2d armLigament;
  private final MechanismLigament2d wristLigament;
  private final MechanismLigament2d wristLigamentBack;

  double armLength = Units.inchesToMeters(18);

  public ArmVisualizer(String logKey, Color8Bit color) {
    this.logKey = logKey;
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("Arm", 1.31115, 1.13081);
    fixedLigament =
        mechanismRoot.append(
            new MechanismLigament2d("Fixed", 0.63881, 90, 6, new Color8Bit(Color.kBlack)));
    armLigament =
        fixedLigament.append(
            new MechanismLigament2d("Arm", armLength, 90, 4, new Color8Bit(Color.kDarkBlue)));
    wristLigament =
        armLigament.append(
            new MechanismLigament2d("Wrist", 0.2032, 0, 4, new Color8Bit(Color.kDarkGreen)));
    wristLigamentBack =
        armLigament.append(
            new MechanismLigament2d("WristBack", 0.2032, 180, 4, new Color8Bit(Color.kDarkRed)));
  }

  public void update(double armAngle, double wristAngle) {
    armLigament.setAngle(Units.radiansToDegrees(armAngle));
    wristLigament.setAngle(Units.radiansToDegrees(wristAngle) + 90);
    wristLigamentBack.setAngle(Units.radiansToDegrees(wristAngle) + 270);

    Logger.recordOutput("Mechanism2d/" + logKey, mechanism);
    Transform3d hardpoint =
        new Transform3d(
            Units.inchesToMeters(12.25), 0, Units.inchesToMeters(25.15), new Rotation3d(0, 0, 0));

    // Pose3d armPose = new Pose3d(Units.inchesToMeters(10), 0, 0, new Rotation3d(0,
    // armAngle, 0));
    Pose3d armPose = getArmPose(armAngle);
    Pose3d wristPose = getWristPose(armAngle, wristAngle);

    Logger.recordOutput("Mechanism3d/" + logKey, armPose, wristPose);
  }

  Translation2d armRoot = new Translation2d(-0.31, 0.64);

  private Pose3d getArmPose(double armAngle) {
    return new Pose3d(armRoot.getX(), 0, armRoot.getY(), new Rotation3d(0, -armAngle, 0));
  }

  /*
   * Get the pose of the wrist at the end of the arm
   */
  private Pose3d getWristPose(double armAngle, double wristAngle) {
    return getArmPose(armAngle)
        .transformBy(new Transform3d(armLength, 0, 0, new Rotation3d(0, wristAngle, 0)));
  }
}
