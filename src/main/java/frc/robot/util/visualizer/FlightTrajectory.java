package frc.robot.util.visualizer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;

public class FlightTrajectory {

  Pose3d start;
  double angle;
  // double velocity;

  double horizontalVelocity;
  double verticalVelocity;

  Pose3d current;

  double gravity = 9.81;

  /**
   * Creates a new FlightTrajectory
   *
   * @param startPosition Starting position
   * @param angle Angle in radians, where 0 is horizontal and pi/2 is vertical
   * @param velocity Velocity in meters per second
   */
  public FlightTrajectory(Pose3d startPosition, double angle, double velocity) {
    this.start = startPosition;
    this.angle = angle;
    // this.velocity = velocity;
    this.current = startPosition;

    this.horizontalVelocity = velocity * Math.cos(angle);
    this.verticalVelocity = velocity * Math.sin(angle);
  }

  public void update(double dt) {
    // Calculate force of gravity
    double gravityForce = gravity * dt;

    // Calculate force of drag
    // This is on a torus shaped object, where the direction of the flight is towards the flat end
    // of the torus
    double dragForce =
        0.5
            * 1.225
            * Math.pow(0.1, 2)
            * Math.PI
            * Math.pow(0.1, 2)
            * horizontalVelocity
            * horizontalVelocity;

    // Calculate horizontal velocity
    double horizontalVelocity = this.horizontalVelocity - dragForce * dt;

    // Calculate vertical velocity
    double verticalVelocity = this.verticalVelocity - gravityForce * dt;

    // Calculate distance traveled horizontally
    double horizontalDistance = horizontalVelocity * dt;

    // Calculate distance traveled vertically
    double verticalDistance = verticalVelocity * dt;

    Transform3d positionTransform =
        new Transform3d(horizontalDistance, 0, verticalDistance, new Rotation3d());
    Pose3d nextPosition = current.transformBy(positionTransform);

    current = nextPosition;
    this.horizontalVelocity = horizontalVelocity;
    this.verticalVelocity = verticalVelocity;

    Logger.recordOutput("Visualization/HorizontalVelocity", horizontalVelocity);
    Logger.recordOutput("Visualization/VerticalVelocity", verticalVelocity);
    Logger.recordOutput("Visualization/HorizontalDistance", horizontalDistance);
    Logger.recordOutput("Visualization/VerticalDistance", verticalDistance);
  }

  public Pose3d getCurrentPosition() {
    return current;
  }

  public double getShotAngle() {
    return angle;
  }
}
