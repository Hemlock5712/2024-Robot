// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.SmartArm;
import frc.robot.commands.SmartFlywheel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeActuatorIO;
import frc.robot.subsystems.intake.IntakeActuatorSim;
import frc.robot.subsystems.intake.IntakeWheelsIO;
import frc.robot.subsystems.intake.IntakeWheesIOSIM;
import frc.robot.subsystems.lineBreak.LineBreak;
import frc.robot.subsystems.lineBreak.LineBreakIO;
import frc.robot.subsystems.lineBreak.LineBreakIOSim;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.subsystems.magazine.MagazineIO;
import frc.robot.subsystems.magazine.MagazineIOSIM;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.util.visualizer.NoteVisualizer;
import frc.robot.util.visualizer.ShotVisualizer;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private AprilTagVision aprilTagVision;
  private Arm arm;
  private Intake intake;
  private LineBreak lineBreak;
  private Magazine magazine;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(moduleConfigs[0]),
                new ModuleIOTalonFX(moduleConfigs[1]),
                new ModuleIOTalonFX(moduleConfigs[2]),
                new ModuleIOTalonFX(moduleConfigs[3]));

        flywheel = new Flywheel(new FlywheelIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeActuatorIO() {}, new IntakeWheelsIO() {});
        magazine = new Magazine(new MagazineIO() {});
        lineBreak = new LineBreak(new LineBreakIO() {});
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOLimelight("limelight-fl"),
                new AprilTagVisionIOLimelight("limelight-fr"),
                new AprilTagVisionIOLimelight("limelight-bl"),
                new AprilTagVisionIOLimelight("limelight-br"));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        // aprilTagVision =
        //     new AprilTagVision(
        //         new AprilTagVisionIOPhotonVisionSIM(
        //             "photonCamera1",
        //             new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
        //             drive::getPose));
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        arm = new Arm(new ArmIOSim());
        intake = new Intake(new IntakeActuatorSim(), new IntakeWheesIOSIM());
        magazine = new Magazine(new MagazineIOSIM());
        lineBreak = new LineBreak(new LineBreakIOSim());
        break;
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeActuatorIO() {}, new IntakeWheelsIO() {});
        magazine = new Magazine(new MagazineIO() {});
        break;
    }

    NoteVisualizer.setRobotPoseSupplier(
        () ->
            new Pose3d(
                new Translation3d(
                    drive.getPose().getTranslation().getX(),
                    drive.getPose().getTranslation().getY(),
                    0),
                new Rotation3d(0, 0, drive.getPose().getRotation().getRadians())));

    NamedCommands.registerCommand(
        "Shoot",
        new SmartShoot(arm, flywheel, magazine, lineBreak, drive::getPose)
            .andThen(
                new ScheduleCommand(
                    Commands.defer(() -> new ShotVisualizer(drive, arm, flywheel), Set.of()))));
    // NamedCommands.registerCommand(
    //     "SIMGamePiecePickup", new ScheduleCommand(new SimulateGamePiecePickup(lineBreak, arm)));

    /*
    Register a command that calls the SimulateGamePiecePickup command without blocking.
    Currently broken, due to what seems like a race condition.
     */
    // NamedCommands.registerCommand(
    //     "SIMGamePiecePickup",
    //     new ScheduleCommand(
    //         Commands.defer(() -> new SimulateGamePiecePickup(lineBreak, arm), Set.of())));

    // Temporary workaround for the above line to prevent blocking at each pickup.
    NamedCommands.registerCommand(
        "SIMGamePiecePickup",
        new InstantCommand(() -> lineBreak.setGamePiece(false, false, false, false, true, false)));
    NamedCommands.registerCommand(
        "SmartControl",
        Commands.parallel(
            new SmartFlywheel(flywheel),
            new SmartArm(arm, lineBreak),
            new SmartIntake(intake, lineBreak, arm::isArmWristInIntakePosition)));
    NamedCommands.registerCommand(
        "IntakeDown", new InstantCommand(() -> intake.enableIntakeRequest()));
    NamedCommands.registerCommand(
        "Preload",
        new InstantCommand(() -> lineBreak.setGamePiece(false, false, false, false, true, false)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // autoChooser.addOption(
    // "Flywheel FF Characterization",
    // new FeedForwardCharacterization(
    // flywheel, flywheel::runCharacterizationVolts,
    // flywheel::getCharacterizationVelocity));
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
    //         .withTimeout(5.0));

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Flywheel SysId (Quasistatic Forward)",
    //     flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Flywheel SysId (Quasistatic Reverse)",
    //     flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Flywheel SysId (Dynamic Forward)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Flywheel SysId (Dynamic Reverse)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    SmartController.getInstance().disableSmartControl();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    arm.setDefaultCommand(new SmartArm(arm, lineBreak));
    flywheel.setDefaultCommand(new SmartFlywheel(flywheel));
    intake.setDefaultCommand(
        new SmartIntake(intake, lineBreak, () -> arm.isArmWristInIntakePosition()));
    magazine.setDefaultCommand(new SmartMagazine(magazine, lineBreak));

    controller
        .leftBumper()
        .whileTrue(Commands.runOnce(() -> SmartController.getInstance().toggleDriveMode()));
    controller
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> SmartController.getInstance().enableSmartControl(),
                () -> SmartController.getInstance().disableSmartControl()));

    controller.a().whileTrue(new PathFinderAndFollow());

    controller
        .b()
        .whileTrue(
            Commands.startEnd(
                () -> intake.enableIntakeRequest(), () -> intake.disableIntakeRequest()));

    controller.x().whileTrue(new SmartShoot(arm, flywheel, magazine, lineBreak, drive::getPose).andThen(
                new ScheduleCommand(
                    Commands.defer(() -> new ShotVisualizer(drive, arm, flywheel), Set.of()))));

    if (Constants.getMode() == Constants.Mode.SIM) {
      controller.pov(0).onTrue(new InstantCommand(() -> lineBreak.bumpGamePiece()));
      controller.pov(180).onTrue(new InstantCommand(() -> lineBreak.shootGamePiece()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
