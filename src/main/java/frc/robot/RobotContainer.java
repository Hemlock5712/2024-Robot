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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SmartController.DriveModeType;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeActuatorIO;
import frc.robot.subsystems.intake.IntakeActuatorIOSpark;
import frc.robot.subsystems.intake.IntakeActuatorSim;
import frc.robot.subsystems.intake.IntakeWheelsIO;
import frc.robot.subsystems.intake.IntakeWheelsIOSIM;
import frc.robot.subsystems.intake.IntakeWheelsIOTalonFX;
import frc.robot.subsystems.leds.LedController;
import frc.robot.subsystems.lineBreak.LineBreak;
import frc.robot.subsystems.lineBreak.LineBreakIO;
import frc.robot.subsystems.lineBreak.LineBreakIODigitalInput;
import frc.robot.subsystems.lineBreak.LineBreakIOSim;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.subsystems.magazine.MagazineIO;
import frc.robot.subsystems.magazine.MagazineIOSIM;
import frc.robot.subsystems.magazine.MagazineIOSpark;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.util.visualizer.NoteVisualizer;
import frc.robot.util.visualizer.RobotGamePieceVisualizer;
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
  private final AprilTagVision aprilTagVision;
  private final Arm arm;
  private final Intake intake;
  private final LineBreak lineBreak;
  private final Magazine magazine;
  private final LedController ledController;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

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

        flywheel = new Flywheel(new FlywheelIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        magazine = new Magazine(new MagazineIOSpark());
        lineBreak = new LineBreak(new LineBreakIODigitalInput());
        intake = new Intake(new IntakeActuatorIOSpark(), new IntakeWheelsIOTalonFX());
        // intake = new Intake(new IntakeActuatorIO() {}, new IntakeWheelsIOTalonFX());
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOLimelight("limelight-fl"),
                new AprilTagVisionIOLimelight("limelight-fr"),
                new AprilTagVisionIOLimelight("limelight-bl"),
                new AprilTagVisionIOLimelight("limelight-br"));
        ledController = new LedController(aprilTagVision);
        climber = new Climber(new ClimberIOTalonFX());
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
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        arm = new Arm(new ArmIOSim());
        intake = new Intake(new IntakeActuatorSim(), new IntakeWheelsIOSIM());
        magazine = new Magazine(new MagazineIOSIM());
        lineBreak = new LineBreak(new LineBreakIOSim());
        ledController = new LedController(aprilTagVision);
        climber = new Climber(new ClimberIO() {});
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
        lineBreak = new LineBreak(new LineBreakIO() {});
        ledController = new LedController(aprilTagVision);
        climber = new Climber(new ClimberIO() {});
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

    RobotGamePieceVisualizer.setRobotPoseSupplier(
        () ->
            new Pose3d(
                new Translation3d(
                    drive.getPose().getTranslation().getX(),
                    drive.getPose().getTranslation().getY(),
                    0),
                new Rotation3d(0, 0, drive.getPose().getRotation().getRadians())));

    RobotGamePieceVisualizer.setArmTransformSupplier(arm::getFlywheelPosition);
    RobotGamePieceVisualizer.setShooterAngleSupplier(arm::getWristAngleAbsolute);
    RobotGamePieceVisualizer.setIsMagazineLoadedSupplier(lineBreak::hasGamePieceIntake);
    RobotGamePieceVisualizer.setIsShooterLoadedSupplier(lineBreak::isShooterLoaded);

    NamedCommands.registerCommand(
        "Shoot",
        new SmartShoot(arm, flywheel, magazine, lineBreak, drive::getPose, 1.5)
            .deadlineWith(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0))
            .andThen(
                new ScheduleCommand(
                    Commands.defer(() -> new ShotVisualizer(drive, arm, flywheel), Set.of()))));
    NamedCommands.registerCommand(
        "QuickShoot",
        new SmartShoot(arm, flywheel, magazine, lineBreak, drive::getPose, 1.0)
            .deadlineWith(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0))
            .andThen(
                new ScheduleCommand(
                    Commands.defer(() -> new ShotVisualizer(drive, arm, flywheel), Set.of()))));

    NamedCommands.registerCommand(
        "EnableSmartControl", Commands.runOnce(SmartController.getInstance()::enableSmartControl));

    // Temporary workaround for the above line to prevent blocking at each pickup.
    // NamedCommands.registerCommand(
    //     "SIMGamePiecePickup",
    //     new ScheduleCommand(
    //         Commands.defer(
    //             () ->
    //                 new InstantCommand(
    //                         () -> lineBreak.setGamePiece(false, true, false, false, false,
    // false))
    //                     .andThen(Commands.waitSeconds(0.2))
    //                     .andThen(
    //                         new InstantCommand(
    //                             () ->
    //                                 lineBreak.setGamePiece(
    //                                     false, false, false, false, true, false))),
    //             Set.of())));

    NamedCommands.registerCommand("SIMGamePiecePickup", Commands.none());

    NamedCommands.registerCommand(
        "SmartControl",
        Commands.parallel(
            new SmartFlywheel(flywheel),
            new SmartArm(arm, lineBreak, climber),
            new SmartIntake(intake, lineBreak, arm::isArmWristInIntakePosition)));

    NamedCommands.registerCommand("IntakeDown", new InstantCommand(intake::enableIntakeRequest));
    NamedCommands.registerCommand("IntakeUp", new InstantCommand(intake::disableIntakeRequest));

    NamedCommands.registerCommand("Magazine", new ManualMagazine(magazine, lineBreak));

    NamedCommands.registerCommand(
        "Preload",
        new InstantCommand(() -> lineBreak.setGamePiece(false, false, false, false, true, false)));

    // Run SmartController updates in autonomous
    new Trigger(DriverStation::isAutonomousEnabled)
        .and(
            new Trigger(
                () -> SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER))
        .whileTrue(
            new InstantCommand(
                () -> {
                  SmartController.getInstance()
                      .calculateSpeaker(drive.getPose(), new Translation2d(0, 0));
                }));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    arm.setDefaultCommand(new SmartArm(arm, lineBreak, climber));
    flywheel.setDefaultCommand(new SmartFlywheel(flywheel));
    intake.setDefaultCommand(
        new SmartIntake(intake, lineBreak, arm::isArmWristInIntakePosition).ignoringDisable(true));
    magazine.setDefaultCommand(new SmartMagazine(magazine, intake, lineBreak));
    lineBreak.setDefaultCommand(
        new InstantCommand(RobotGamePieceVisualizer::drawGamePieces, lineBreak));
    ledController.setDefaultCommand(new HandleLEDs(ledController, lineBreak));
    climber.setDefaultCommand(new SmartClimb(climber));

    controller
        .start()
        .and(controller.back())
        .whileTrue(
            Commands.runOnce(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    controller
        .leftTrigger()
        .whileTrue(new SmartShoot(arm, flywheel, magazine, lineBreak, drive::getPose, 2.5));

    controller
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(intake::enableIntakeRequest, intake::disableIntakeRequest)
                .deadlineWith(new VibrateController(controller, lineBreak)));
    // Commands.either(
    //         new ManualArm(arm, flywheel, ArmConstants.frontAmp),
    //         () -> SmartController.getInstance().getEmergencyIntakeMode())

    controller.a().whileTrue(Commands.runOnce(SmartController.getInstance()::enableSmartControl));

    controller.b().whileTrue(Commands.runOnce(SmartController.getInstance()::disableSmartControl));

    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                Commands.run(intake::outtake, intake), Commands.run(magazine::backward, magazine)));

    controller.y().whileTrue(new ManualArm(arm, flywheel, ArmConstants.emergencyIntake, () -> -5));

    controller2
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.AMP),
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    controller2
        .y()
        .whileTrue(
            Commands.run(() -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    // controller2
    //     .leftTrigger(0.5)
    //     .and(controller2.b())
    //     .onTrue(new ManualClimber(climber, 5.2, 0))
    //     .onFalse(new ManualClimber(climber, 2.4, 1));

    controller2
        .pov(180)
        .toggleOnTrue(
            Commands.startEnd(
                () -> climber.setRequestingClimb(true), () -> climber.setRequestingClimb(false)));
    controller2
        .pov(0)
        .toggleOnTrue(
            Commands.runEnd(
                () -> SmartController.getInstance().setDriveMode(DriveModeType.CLIMBER),
                () -> SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER)));

    // controller2.leftTrigger(0.5).whileTrue(new MoveArmForClimbing(arm, climber));

    // controller2
    //     .leftTrigger(0.5)
    //     .and(controller2.pov(0))
    //     .toggleOnTrue(new ManualArm(arm, flywheel, ArmConstants.trap));

    // controller2.x().whileTrue(new CalibrateClimber(climber));

    controller.x().whileTrue(new ManualShoot(arm, flywheel, magazine, lineBreak, 1.5));

    // controller2
    //     .start()
    //     .and(controller2.back())
    //     .onTrue(Commands.runOnce(SmartController.getInstance()::toggleEmergencyIntakeMode));

    if (Constants.getMode() == Constants.Mode.SIM) {
      controller.pov(0).onTrue(new InstantCommand(lineBreak::bumpGamePiece));
      controller.pov(180).onTrue(new InstantCommand(lineBreak::shootGamePiece));
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

  public void intakeUp() {
    intake.disableIntakeRequest();
  }
}
