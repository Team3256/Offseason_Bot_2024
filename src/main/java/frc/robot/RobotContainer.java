// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.pivotintake.PivotIntakeConstants.kPivotGroundPos;
import static frc.robot.subsystems.pivotshooter.PivotShooterConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FeatureFlags;
import frc.robot.autos.commands.IntakeSequence;
import frc.robot.helpers.XboxStalker;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.AmpBarIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.commands.*;
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
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.utils.CommandQueue;
import frc.robot.utils.SwerveFieldCentricFacingAngle;
import io.github.oblarg.oblog.annotations.Config;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController test = new CommandXboxController(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int secondaryAxis = XboxController.Axis.kRightY.value;

  /* Subsystems */
  private boolean isRed;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // My drivetrain

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
  private SwerveFieldCentricFacingAngle azi =
          new SwerveFieldCentricFacingAngle()
                  .withDeadband(MaxSpeed*.1)
                  .withRotationalDeadband(MaxAngularRate*.1)
                  .withHeadingController(SwerveConstants.azimuthController)
                  .withDriveRequestType(
                          SwerveModule.DriveRequestType.OpenLoopVoltage);
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public Shooter shooter;
  public Intake intake;
  public AmpBar ampbar;
  public PivotIntake pivotIntake;
  public Climb climb;
  public CommandQueue commandQueue;

  public Vision vision;

  public PivotShooter pivotShooter;
  public LED led;

  @Config.Command(name = "Climb Zero")
  private Command zeroClimb;

  /* Auto */
  //  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Cancel any previous commands running
    CommandScheduler.getInstance().cancelAll();

    commandQueue = new CommandQueue();
    vision = new Vision(new VisionIOLimelight());

    // Setup subsystems & button-bindings
    if (FeatureFlags.kPivotShooterEnabled) {
      configurePivotShooter();
    }
    if (FeatureFlags.kShooterEnabled) {
      configureShooter();
    }

    if (FeatureFlags.kPivotIntakeEnabled) {
      configurePivotIntake();
      test.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
      test.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
      test.y().whileTrue(pivotIntake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      test.a().whileTrue(pivotIntake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      test.b().whileTrue(pivotIntake.sysIdDynamic(SysIdRoutine.Direction.kForward));
      test.x().whileTrue(pivotIntake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
    if (FeatureFlags.kIntakeEnabled) {
      configureIntake();
    }
    if (FeatureFlags.kSwerveEnabled) {
      configureSwerve();
    }
    if (FeatureFlags.kClimbEnabled) {
      configureClimb();
    }
    // If all the subsystems are enabled, configure "operator" autos
    if (FeatureFlags.kIntakeEnabled
        && FeatureFlags.kShooterEnabled
        && FeatureFlags.kPivotIntakeEnabled) {
      configureOperatorAutos();
    }

    if (FeatureFlags.kLEDEnabled) {
      configureLED();
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

      NamedCommands.registerCommand( // shoot preloaded note to speaker, use at match start
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

      NamedCommands.registerCommand( // shoot preloaded note to speaker, use at match start
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
      NamedCommands.registerCommand( // intake ground note, stow to feeder chamber
          "intake sequence",
          new ParallelCommandGroup(
              pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos),
              intake.intakeIn(),
              // new PivotShooterSlamAndVoltage(pivotShooter),
              // new PivotShootSubwoofer(pivotShooter),
              shooter.setVelocity(
                  ShooterConstants.kShooterSubwooferRPS,
                  ShooterConstants.kShooterFollowerSubwooferRPS)));
      NamedCommands.registerCommand( // outtake note to feeder
          "outtake speaker",
          new SequentialCommandGroup(
              // new ScheduleCommand(new PivotShootSubwoofer(pivotShooter)).asProxy(),
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

      NamedCommands.registerCommand( // outtake note to feeder
          "safety",
          new ParallelCommandGroup(
              intake.intakeIn().withTimeout(1),
              shooter.setVelocity(
                  ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS)));
      NamedCommands.registerCommand(
          "aim wing center",
          pivotShooter.setPosition(
              PivotShooterConstants.kWingNoteCenterPreset)); // wing note center
      NamedCommands.registerCommand(
          "aim wing side",
          pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)); // wing note side
      NamedCommands.registerCommand(
          "aim wing far side",
          pivotShooter.setPosition(
              PivotShooterConstants.kWingNoteFarSidePreset)); // wing note far side
      NamedCommands.registerCommand(
          "aim truss",
          pivotShooter.setPosition(
              PivotShooterConstants.kTrussSourceSidePreset)); // truss source sid
      NamedCommands.registerCommand(
          "aim half truss wing",
          pivotShooter.setPosition(
              PivotShooterConstants.kHalfWingPodiumPreset)); // half wing podium
      NamedCommands.registerCommand("zero pivot shooter", pivotShooter.slamAndPID());

      NamedCommands.registerCommand( // rev shooter to speaker presets
          "rev speaker",
          shooter.setVelocity(
              ShooterConstants.kShooterSubwooferRPS,
              ShooterConstants.kShooterFollowerSubwooferRPS));
      NamedCommands.registerCommand( // rev shooter to amp presets
          "rev amp",
          shooter.setVelocity(
              ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS));
      NamedCommands.registerCommand( // modular pivot down, use for sabotage
          "pivot down", pivotIntake.setPosition(kPivotGroundPos).withTimeout(0.75));
      NamedCommands.registerCommand("stow", pivotIntake.slamAndPID().withTimeout(0.75));
      NamedCommands.registerCommand( // intake with no stow, use for sabotage
          "intake", intake.intakeIn());
      NamedCommands.registerCommand( // shoot preloaded note to amp, use at match start
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

    // operator.povLeft().onTrue(cancelCommand);
    // Configure the auto
    //    if (FeatureFlags.kSwerveEnabled) {
    //      autoChooser = AutoBuilder.buildAutoChooser();
    //    } else {
    //      autoChooser = new SendableChooser<>();
    //    }
    //    // Autos
    //    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configurePivotShooter() {
    pivotShooter = new PivotShooter(new PivotShooterIOTalonFX());
    // operator.b().onTrue(new bruh(pivotShooter));
    // operator.x().onTrue(new SequentialCommandGroup(new
    // PivotShootSubwoofer(pivotShooter)));
    operator
        .x()
        .onTrue(
            new SequentialCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset)));
    operator
        .b()
        .onTrue(
            new SequentialCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)));
  }

  private void configureIntake() {
    intake = new Intake(new IntakeIOTalonFX());
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
    driver.rightTrigger().whileTrue(intake.intakeIn());

    // operator.povDown().onTrue(new IntakeOff(intake));
  }

  private void configurePivotIntake() {
    pivotIntake = new PivotIntake(new PivotIntakeIOTalonFX());
    operator.povRight().onTrue(pivotIntake.setPosition(kPivotGroundPos));
    operator.povLeft().onTrue(pivotIntake.slamAndPID());
  }

  private void configureClimb() {
    climb = new Climb(new ClimbIOTalonFX());
    // zeroClimb = new ZeroClimb(climb); // NEED FOR SHUFFLEBOARD

    operator.povDown().onTrue(climb.zero());
    // new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5).onTrue(new
    // UpClimb(climb));
    new Trigger(() -> operator.getRawAxis(translationAxis) > 0.5).onTrue(climb.retractClimber());
    if (this.ampbar != null && this.pivotShooter != null) {
      new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5)
          .onTrue(
              Commands.sequence(
                  new ParallelCommandGroup(
                          ampbar.setAmpPosition(), pivotShooter.setPosition(12 / 138.33))
                      .withTimeout(1),
                  climb.extendClimber()));
    } else {
      new Trigger(() -> Math.abs(operator.getRawAxis(secondaryAxis)) > 0.5)
          .onTrue(new PrintCommand("u suck")); // old command waws dehook climb
    }
    // Josh: HangSequence is broken and Rhea does not want to use it; we should
    // rmove this
    // later.
    // new ScheduleCommand(new HangSequence(climb, operator)).schedule();
    // operator.povDownLeft().onTrue(new TestClimbFlip(climb));
  }

  public void setAllianceCol(boolean col) {
    isRed = col;
  }

  public void configureSwerve() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driver.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureShooter() {
    shooter = new Shooter(new ShooterIOTalonFX());
    // new Trigger(() -> Math.abs(shooter.getShooterRps() - 100) <= 5)
    // .onTrue(
    // new InstantCommand(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 100);
    // }))
    // .onFalse(
    // new InstantCommand(
    // () -> {
    // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    // }));
    if (FeatureFlags.kAmpBarEnabled) {
      ampbar = new AmpBar(new AmpBarIOTalonFX());
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
                  pivotShooter.setPosition(kAmpPreset)));
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
    operator.a().onTrue(new IntakeSequence(intake, pivotIntake, pivotShooter, shooter, ampbar));
    operator
        .povUp()
        .onTrue(
            new ParallelCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kFeederPreset),
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

  public void configureLED() {
    int[][] ledList = new int[][] {new int[] {2, 3}, new int[] {1, 1}};

    led = new LED();
    led.setDefaultCommand(new CoordinatesButItsMultiple(led, ledList, 100, 0, 0, 10));
    // led.setDefaultCommand(new SetLEDsFromBinaryString(led, LEDConstants.based,
    // 100, 0, 0, 5));

    /*
     * Intake LED, flashes RED while intake is down and running,
     * flashes GREEN on successful intake
     */
    if (FeatureFlags.kIntakeEnabled) {
      // Trigger intakeDetectedNote = new Trigger(intake::isBeamBroken);
      // // intakeDetectedNote.whileTrue(new SetSuccessfulIntake(led));

      // intakeDetectedNote
      // .onTrue(
      // new InstantCommand(
      // () -> {
      // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
      // driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
      // }))
      // .onFalse(
      // new InstantCommand(
      // () -> {
      // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      // driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      // }));

      // This boolean is true when velocity is LESS than 0.
      // Trigger intakeRunning = new Trigger(intake::isMotorSpinning);
      // intakeRunning.whileTrue(new SetGroundIntakeRunning(led));
    }

    /*
     * Shooter LED, solid ORANGE while shooter is running, flashes ORANGE if note
     * fed
     */
    // if (FeatureFlags.kShooterEnabled) {

    // Trigger shooterRunning = new Trigger(
    // () -> (shooter.getShooterFollowerRps() > 75 || shooter.getShooterRps() >
    // 75));
    // shooterRunning.whileTrue(new SetSpeakerScore(led));
    // }
    // if (FeatureFlags.kSwerveEnabled) {
    // if (DriverStation.isAutonomousEnabled() ||

    // FeatureFlags.kSwerveUseVisionForPoseEst) {
    // Trigger swerveSpeakerAligned = new Trigger(swerveDrive::isAlignedToSpeaker);
    // swerveSpeakerAligned.whileTrue(new SetRobotAligned(led));
    // }
    // if (DriverStation.isTeleopEnabled()) {
    // // if (DriverStation.isTeleopEnabled()) {
    // // Trigger azimuthRan =
    // // new Trigger(
    // // () ->
    // // (driver.leftBumper().getAsBoolean() ||
    // driver.rightBumper().getAsBoolean())
    // // || driver.x().getAsBoolean()
    // // || driver.b().getAsBoolean()
    // // || driver.a().getAsBoolean()
    // // || driver.povUp().getAsBoolean());
    // // azimuthRan.whileTrue(new SetAzimuthRan(led));
    // // }
    // }

    // if (FeatureFlags.kClimbEnabled) {
    // Trigger climbRunning =
    // new Trigger(
    // () -> ((climb.getLeftCurrent() > 0) || (operator.povDown().getAsBoolean())));
    // }
    // }
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

  //  public Command getAutonomousCommand() {
  //    return autoChooser.getSelected();
  //  }

  /* Test Routines */

  public void runPitTestRoutine() {
    // Command pitRoutine = new PitRoutine(swerveDrive, climb, intake, pivotIntake,
    // shooter);
    // pitRoutine.schedule();
  }

  public void periodic(double dt) {
    XboxStalker.stalk(driver, operator);
    // System.out.println(Limelight.getBotpose("limelight").length);
    // //
    // double ty = Limelight.getTY("limelight");
    // //
    // // // how many degrees back is your limelight rotated from perfectly
    // vertical?
    // //
    // double limelightMountAngleDegrees = 21.936;

    // // distance from the center of the Limelight lens to the floor
    // double limelightLensHeightInches = 15.601;

    // // distance from the target to the floor
    // double goalHeightInches = 56.375;

    // double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // // calculate distance
    // double distanceFromLimelightToGoalInches =
    // (goalHeightInches - limelightLensHeightInches) /
    // Math.tan(angleToGoalRadians);
    // LimelightHelpers.setPriorityTagID("limelight", 7);
    // System.out.println("Distance: " + ty);
    // System.out.println("Distance: " + distanceFromLimelightToGoalInches);
    // Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    // if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
    // System.out.println("red");
    // } else if (ally.isPresent()) {
    // System.out.println("blue");
    // } else {
    // System.out.println("red");
    // }
  }
}
