// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.pivotintake.PivotIntakeConstants.kPivotGroundPos;
import static frc.robot.subsystems.pivotshooter.PivotShooterConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.AzimuthAngles.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeatureFlags;
import frc.robot.autos.routines.AutoRoutines;
import frc.robot.helpers.XboxStalker;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.AmpBarIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotintake.PivotIntakeIOTalonFX;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import frc.robot.subsystems.pivotshooter.PivotShooterIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveTelemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int secondaryAxis = XboxController.Axis.kRightY.value;

  /* Subsystems */
  private boolean isRed;
  private double clockAngle;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  public static double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  public static double MaxAngularRate = 1.7 * Math.PI; // My drivetrain

  private double SlowMaxSpeed = 5.96 * 0.17;
  private double SlowMaxAngular = 1.3 * Math.PI * 0.2;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.stickDeadband * MaxSpeed)
          .withRotationalDeadband(
              Constants.rotationalDeadband * MaxAngularRate) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop
  private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(MaxSpeed);

  public Shooter shooter;
  public Intake intake;
  public AmpBar ampbar;
  public PivotIntake pivotIntake;
  public Climb climb;

  public Vision vision;

  public PivotShooter pivotShooter;
  public LED led;

  /* Auto */
  // private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Cancel any previous commands running
    CommandScheduler.getInstance().cancelAll();

    vision = new Vision(new VisionIOLimelight());

    // Setup subsystems & button-bindings
    configurePivotShooter();
    configureShooter();

    configurePivotIntake();
    configureIntake();
    configureSwerve();

    configureClimb();
    // If all the subsystems are enabled, configure "operator" autos
    if (FeatureFlags.kIntakeEnabled
        && FeatureFlags.kShooterEnabled
        && FeatureFlags.kPivotIntakeEnabled) {
      configureOperatorAutos();
    }
    if (FeatureFlags.kRumbleEnabled) {
      configureRumble();
    }

    // Named commands
    {
      // Auto named commands
      NamedCommands.registerCommand("test intake", intake.setIntakeVoltage(12).withTimeout(1));
      // NamedCommands.registerCommand( // intake ground note, stow to feeder chamber
      // "intake sequence new",
      // new SequentialCommandGroup(
      // new ParallelCommandGroup(
      // new PivotSetAngle(pivotIntake, PivotConstants.kPivotGroundAngleDeg)
      // .withTimeout(0.75),
      // new IntakeIn(intake)),
      // new ParallelCommandGroup(
      // new PivotSlamAndVoltage(pivotIntake).withTimeout(0.75),
      // new ScheduleCommand(new ShootSpeaker(shooter)))));

      NamedCommands.registerCommand(
          // shoot preloaded note to speaker, use at match start
          "preload speaker",
          new SequentialCommandGroup(
              // new PrintCommand("preload im outta blush"),
              pivotShooter.zero(),
              new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                      new WaitCommand(0.5),
                      intake
                          .setVoltage(
                              IntakeConstants.kIntakeIntakeVoltage,
                              IntakeConstants.kPassthroughIntakeVoltage)
                          .withTimeout(0.7)),
                  pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset),
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS))));

      NamedCommands.registerCommand(
          // shoot preloaded note to speaker, use at match start
          "preload speaker amp side",
          new SequentialCommandGroup(
              // new PrintCommand("preload im outta blush"),
              pivotShooter.zero(),
              new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                      new WaitCommand(0.8),
                      intake
                          .setVoltage(
                              IntakeConstants.kIntakeIntakeVoltage,
                              IntakeConstants.kPassthroughIntakeVoltage)
                          .withTimeout(0.7)),
                  pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset),
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS))
              // new PivotShooterSlamAndVoltage(pivotShooter)));
              ));
      NamedCommands.registerCommand(
          // intake ground note, stow to feeder chamber
          "intake sequence",
          new ParallelCommandGroup(
              pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos),
              intake.intakeIn(),
              // new PivotShooterSlamAndVoltage(pivotShooter),
              // new PivotShootSubwoofer(pivotShooter),
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS)));
      NamedCommands.registerCommand(
          // outtake note to feeder
          "outtake speaker",
          new SequentialCommandGroup(
              // new ScheduleCommand(new
              // PivotShootSubwoofer(pivotShooter)).asProxy(),
              new ParallelCommandGroup(
                  intake
                      .setVoltage(
                          IntakeConstants.kIntakeIntakeVoltage,
                          IntakeConstants.kPassthroughIntakeVoltage)
                      .withTimeout(2),
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS))));
      NamedCommands.registerCommand(
          "aim subwoofer", pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset));
      NamedCommands.registerCommand("shooter off", shooter.off());

      NamedCommands.registerCommand(
          // outtake note to feeder
          "safety",
          new ParallelCommandGroup(
              intake.intakeIn().withTimeout(1),
              shooter.setVelocity(
                  ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS)));
      NamedCommands.registerCommand(
          "aim wing center",
          pivotShooter.setPosition(PivotShooterConstants.kWingNoteCenterPreset)); // wing note
      // center
      NamedCommands.registerCommand(
          "aim wing side",
          pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)); // wing
      // note
      // side
      NamedCommands.registerCommand(
          "aim wing far side",
          pivotShooter.setPosition(PivotShooterConstants.kWingNoteFarSidePreset)); // wing note far
      // side
      NamedCommands.registerCommand(
          "aim truss",
          pivotShooter.setPosition(PivotShooterConstants.kTrussSourceSidePreset)); // truss source
      // sid
      NamedCommands.registerCommand(
          "aim half truss wing",
          pivotShooter.setPosition(PivotShooterConstants.kHalfWingPodiumPreset)); // half wing
      // podium
      NamedCommands.registerCommand("zero pivot shooter", pivotShooter.slamAndPID());

      NamedCommands.registerCommand(
          // rev shooter to speaker presets
          "rev speaker",
          shooter.setVelocity(
              ShooterConstants.kShooterSubwooferRPS,
              ShooterConstants.kShooterFollowerSubwooferRPS));
      NamedCommands.registerCommand(
          // rev shooter to amp presets
          "rev amp",
          shooter.setVelocity(
              ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS));
      NamedCommands.registerCommand(
          // modular pivot down, use for sabotage
          "pivot down", pivotIntake.setPosition(kPivotGroundPos).withTimeout(0.75));
      NamedCommands.registerCommand("stow", pivotIntake.slamAndPID().withTimeout(0.75));
      NamedCommands.registerCommand(
          // intake with no stow, use for sabotage
          "intake", intake.intakeIn());
      NamedCommands.registerCommand(
          // shoot preloaded note to amp, use at match start
          "preload amp",
          new SequentialCommandGroup(
              pivotIntake.zero(),
              new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                      new WaitCommand(0.8), intake.intakeIn().withTimeout(1.5)),
                  shooter.setVelocity(
                      ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS))));
      NamedCommands.registerCommand(
          "scheduled shoot speaker",
          new ScheduleCommand(
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS)));
      NamedCommands.registerCommand(
          "lmao",
          new RepeatCommand(
              new SequentialCommandGroup(
                  pivotIntake.setPosition(kPivotGroundPos).withTimeout(0.75),
                  pivotIntake.slamAndPID())));
    }
    /* Run checks */
    configureCheeks();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption(
        "Source auto",
        AutoRoutines.sourceMobility(drivetrain, intake, shooter, pivotShooter, pivotIntake));
    autoChooser.addOption(
        "Amp Mobility",
        AutoRoutines.ampMobility(drivetrain, intake, shooter, pivotShooter, pivotIntake));
    autoChooser.addOption(
        "5 Note",
        AutoRoutines.center5Note(drivetrain, intake, shooter, pivotShooter, pivotIntake, vision));
    autoChooser.addOption(
        "5 Note but center first",
        AutoRoutines.center5Note2(drivetrain, intake, shooter, pivotShooter, pivotIntake, vision));
    autoChooser.addOption(
        "Source Rush Shreya Auto",
        AutoRoutines.sourceCenter2(drivetrain, intake, shooter, pivotShooter, pivotIntake, vision));
    autoChooser.addOption(
        "Note Detection",
        AutoRoutines.noteDetectionRush(
            drivetrain, intake, pivotIntake, pivotShooter, shooter, vision));
    autoChooser.addOption(
        "1 Feed 1 Score Preload Amp",
        AutoRoutines.ampFeed1Sub1Pre1(drivetrain, intake, pivotIntake, pivotShooter, shooter));
    autoChooser.addOption("Box path", AutoRoutines.boxAuto(drivetrain));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureRumble() {
    // if (FeatureFlags.kShooterEnabled) {
    // // When shooter is within 5 RPM of target, rumble operator controller
    // new Trigger(() -> shooter.isAtTargetVelocity())
    // .onTrue(
    // Commands.run(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 100);
    // }))
    // .onFalse(
    // Commands.run(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    // }));
    // }
    if (FeatureFlags.kIntakeEnabled) {

      // XXX: Or maybe I should re-use the debouncedBeamBreak trigger inside this
      // object
      new Trigger(() -> intake.isBeamBroken())
          .onTrue(
              Commands.run(
                  () -> {
                    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                  }))
          .onFalse(
              Commands.run(
                  () -> {
                    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                  }));
    }
  }

  private void configurePivotShooter() {
    pivotShooter = new PivotShooter(FeatureFlags.kPivotShooterEnabled, new PivotShooterIOTalonFX());
    operator
        .x()
        .onTrue(
            new SequentialCommandGroup(
                pivotShooter.setPosition(
                    PivotShooterConstants.kSubWooferPreset * kPivotMotorGearing)));
    operator
        .b()
        .onTrue(
            new SequentialCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)));
  }

  private void configureIntake() {
    intake = new Intake(FeatureFlags.kIntakeEnabled, new IntakeIOTalonFX());
    // intake.setDefaultCommand(new IntakeSetVoltage(intake, 0));
    // operator.rightBumper().whileTrue(new IntakeInOverride(intake));
    // We assume intake is already enabled, so if pivot is enabled as
    // use IntakeOutWithArm
    operator
        .rightBumper()
        .whileTrue(
            intake.setVoltage(
                IntakeConstants.kIntakeIntakeVoltage, IntakeConstants.kPassthroughIntakeVoltage));

    operator
        .leftBumper()
        .whileTrue(
            intake.setVoltage(
                -IntakeConstants.kIntakeIntakeVoltage, -IntakeConstants.kPassthroughIntakeVoltage));
    // operator.povDown().onTrue(new IntakeOff(intake));
  }

  private void configurePivotIntake() {
    pivotIntake = new PivotIntake(FeatureFlags.kPivotIntakeEnabled, new PivotIntakeIOTalonFX());
    operator
        .povRight()
        .onTrue(pivotIntake.setPosition(kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing));
    operator.povLeft().onTrue(pivotIntake.slamAndPID());
  }

  private void configureClimb() {
    climb = new Climb(FeatureFlags.kClimbEnabled, new ClimbIOTalonFX());
    operator.povDown().onTrue(climb.zero());

    new Trigger(() -> operator.getRawAxis(translationAxis) > 0.5).onTrue(climb.retractClimber());
    if (this.ampbar != null && this.pivotShooter != null) {
      new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5)
          .onTrue(
              Commands.sequence(
                  new ParallelCommandGroup(
                          ampbar.setAmpPosition(),
                          pivotShooter.setPosition(12 / 138.33 * kPivotMotorGearing))
                      .withTimeout(1),
                  climb.extendClimber()));
    } else {
      new Trigger(() -> Math.abs(operator.getRawAxis(secondaryAxis)) > 0.5)
          .onTrue(new PrintCommand("u suck")); // old
      // command
      // waws
      // dehook
      // climb
    }
  }

  public void setAllianceCol(boolean col) {
    isRed = col;
  }

  public double getY() {
    return driver.getRightY();
  }

  public double getX() {
    return driver.getRightX();
  }

  public void setClockAngle(double angle) {
    clockAngle = angle;
  }

  public void configureSwerve() {
    // default command
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(driver.getLeftY() * MaxSpeed) // Drive -y is forward
                    .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive -x is left
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
    // TODO: maybe make left stick axes negative, test first

    // TODO: if clock angle is 0, robot will not adjust heading
    driver
        .rightTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                drivetrain.getPigeon2().getAngle(),
                                clockAngle,
                                Timer.getFPGATimestamp()))));

    driver
        .leftTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(-driver.getRightX() * SlowMaxAngular)));

    // Reset robot heading on button press
    driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    driver
        .povUp()
        .onTrue(
            Commands.sequence(
                drivetrain.applyRequest(
                    () ->
                        drive
                            .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                            .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                            .withRotationalRate(
                                SwerveConstants.azimuthController.calculate(
                                    drivetrain.getPigeon2().getAngle(),
                                    aziCleanUp,
                                    Timer.getFPGATimestamp()))),
                drivetrain.runOnce(drivetrain::seedFieldRelative)));

    // Azimuth angle bindings. isRed == true for red alliance presets. isRed != true
    // for blue.
    if (isRed) {
      driver
          .rightBumper()
          .whileTrue(
              drivetrain.applyRequest(
                  () ->
                      drive
                          .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                          .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  drivetrain.getPigeon2().getAngle(),
                                  aziSourceRed,
                                  Timer.getFPGATimestamp()))));
      driver
          .a()
          .whileTrue(
              drivetrain.applyRequest(
                  () ->
                      drive
                          .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                          .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  drivetrain.getPigeon2().getAngle(),
                                  aziAmpRed,
                                  Timer.getFPGATimestamp()))));
      driver
          .povRight()
          .whileTrue(
              drivetrain.applyRequest(
                  () ->
                      drive
                          .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                          .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  drivetrain.getPigeon2().getAngle(),
                                  aziFeederRed,
                                  Timer.getFPGATimestamp()))));
    } else {
      driver
          .rightBumper()
          .whileTrue(
              drivetrain.applyRequest(
                  () ->
                      drive
                          .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                          .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  drivetrain.getPigeon2().getAngle(),
                                  aziSourceBlue,
                                  Timer.getFPGATimestamp()))));
      driver
          .a()
          .whileTrue(
              drivetrain.applyRequest(
                  () ->
                      drive
                          .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                          .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  drivetrain.getPigeon2().getAngle(),
                                  aziAmpBlue,
                                  Timer.getFPGATimestamp()))));
      driver
          .povRight()
          .whileTrue(
              drivetrain.applyRequest(
                  () ->
                      drive
                          .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                          .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  drivetrain.getPigeon2().getAngle(),
                                  aziFeederBlue,
                                  Timer.getFPGATimestamp()))));
    }

    // Universal azimuth bindings

    driver
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                drivetrain.getPigeon2().getAngle(),
                                aziSubwooferFront,
                                Timer.getFPGATimestamp()))));
    driver
        .x()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                drivetrain.getPigeon2().getAngle(),
                                aziSubwooferLeft,
                                Timer.getFPGATimestamp()))));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                drivetrain.getPigeon2().getAngle(),
                                aziSubwooferRight,
                                Timer.getFPGATimestamp()))));
    driver
        .povDown()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(driver.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(driver.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                drivetrain.getPigeon2().getAngle(),
                                aziCleanUp,
                                Timer.getFPGATimestamp()))));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(swerveTelemetry::telemeterize);
    // drivetrain.registerTelemetry(logger::telemeterize);
    // drivetrain.registerTelemetry(swerveTelemetry::telemeterize);
  }

  private void configureShooter() {
    shooter = new Shooter(FeatureFlags.kShooterEnabled, new ShooterIOTalonFX());
    if (FeatureFlags.kAmpBarEnabled) {
      ampbar = new AmpBar(FeatureFlags.kAmpBarEnabled, new AmpBarIOTalonFX());
      operator
          .rightTrigger()
          .onTrue(
              Commands.parallel(
                  shooter.setVelocity(
                      ShooterConstants.kShooterSubwooferRPS,
                      ShooterConstants.kShooterFollowerSubwooferRPS),
                  ampbar.setStowPosition()));
      operator
          .leftTrigger()
          .onTrue(
              new ParallelCommandGroup(
                  shooter.setVelocity(
                      ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS),
                  ampbar.setAmpPosition(),
                  pivotShooter.setPosition(kAmpPreset * kPivotMotorGearing)));
      operator
          .y()
          .onTrue(
              new ParallelCommandGroup(
                  shooter.off(), ampbar.setStowPosition(), pivotShooter.slamAndPID()));
    } else {
      operator
          .rightTrigger()
          .onTrue(
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS));
      operator
          .leftTrigger()
          .onTrue(
              shooter.setVelocity(
                  ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS));
      operator.y().onTrue(new ParallelCommandGroup(shooter.off(), pivotShooter.slamAndPID()));
    }
  }

  private void configureOperatorAutos() {
    operator.a().whileTrue(drivetrain.pidToNote(vision));
    operator
        .povUp()
        .onTrue(
            new ParallelCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kFeederPreset * kPivotMotorGearing),
                shooter.setVelocity(
                    ShooterConstants.kShooterFeederRPS,
                    ShooterConstants.kShooterFollowerFeederRPS)));
  }

  // operator.x().onTrue(new AutoScoreAmp(swerveDrive, shooter, intake));
  // operator.b().onTrue(new AutoScoreSpeaker(swerveDrive, shooter, intake));
  // }

  public void disableRumble() {
    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  private void configureCheeks() {
    // Here, put checks for the configuration of the robot
    if (DriverStation.isFMSAttached()) {
      // FMS-attached checks
      if (!FeatureFlags.kSwerveEnabled) {
        // Swerve is disabled, but the robot is FMS-attached. this *probably* shouldn't
        // happen!
        System.out.println(
            "Swerve is disabled, but the robot is FMS-attached. This probably shouldn't happen!");

        DriverStation.reportError(
            "Swerve is disabled, but the robot is FMS-attached. This probably shouldn't happen!",
            false);
        // You can forcibly disable the robot by throwing an exception here, or by
        // calling
        // DriverStationJNI.observeUserProgramDisabled();
      }
      if (FeatureFlags.kDebugEnabled) {
        System.err.println(
            "Robot is FMS-attached & debug mode is enabled. This is not recommended!");
        DriverStation.reportWarning(
            "Robot is FMS-attached & debug mode is enabled. This is not recommended.", false);
      }
    } else {
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void periodic(double dt) {
    XboxStalker.stalk(driver, operator);
  }
}
