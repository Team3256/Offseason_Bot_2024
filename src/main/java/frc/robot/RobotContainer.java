// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.RobotBase.isReal;
import static frc.robot.subsystems.pivotintake.PivotIntakeConstants.kPivotGroundPos;
import static frc.robot.subsystems.pivotshooter.PivotShooterConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.AzimuthConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeatureFlags;
import frc.robot.autos.commands.AutoScoreAmp;
import frc.robot.autos.commands.AutoScoreSpeaker;
import frc.robot.autos.commands.IntakeSequence;
import frc.robot.autos.commands.MoveToNote;
import frc.robot.autos.commands.RotateToNote;
import frc.robot.helpers.XboxStalker;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.commands.AmpPosition;
import frc.robot.subsystems.ampbar.commands.StowPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.commands.DehookClimb;
import frc.robot.subsystems.climb.commands.DownClimb;
import frc.robot.subsystems.climb.commands.UpClimb;
import frc.robot.subsystems.climb.commands.ZeroClimb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.*;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.commands.*;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotintake.PivotIntakeIOTalonFX;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import frc.robot.subsystems.pivotshooter.PivotShooterIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.subsystems.shooter.commands.ShootAmp;
import frc.robot.subsystems.shooter.commands.ShootFeed;
import frc.robot.subsystems.shooter.commands.ShootSpeaker;
import frc.robot.subsystems.shooter.commands.ShootSubwoofer;
import frc.robot.subsystems.shooter.commands.ShooterOff;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.commands.*;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

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
  // private final CommandXboxController tester = new CommandXboxController(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int secondaryAxis = XboxController.Axis.kRightY.value;

  /* Swerve Helpers */
  private boolean isRed = true;

  /* Subsystems */
  @Log @Config public SwerveDrive swerveDrive;
  @Log @Config public Shooter shooter;
  @Log @Config public Intake intake;
  @Log @Config public AmpBar ampbar;
  @Log @Config public PivotIntake pivotIntake;
  @Log @Config public Climb climb;

  @Log @Config public PivotShooter pivotShooter;
  @Log @Config public LED led;

  @Config.Command(name = "Auto Score Speaker")
  private Command autoScoreSpeaker;

  @Config.Command(name = "Auto Score Amp")
  private Command autoScoreAmp;

  @Config.Command(name = "Climb Zero")
  private Command zeroClimb;

  /* Auto */
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Cancel any previous commands running
    CommandScheduler.getInstance().cancelAll();

    // Setup subsystems & button-bindings
    if (FeatureFlags.kPivotShooterEnabled) {
      configurePivotShooter();
    }
    if (FeatureFlags.kShooterEnabled) {
      configureShooter();
    }

    if (FeatureFlags.kPivotIntakeEnabled) {
      configurePivotIntake();
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

    if (FeatureFlags.kSwerveEnabled
        && FeatureFlags.kIntakeEnabled
        && FeatureFlags.kShooterEnabled
        && FeatureFlags.kPivotIntakeEnabled) {
      autoScoreSpeaker = new AutoScoreSpeaker(swerveDrive, shooter, intake);
      autoScoreAmp = new AutoScoreAmp(swerveDrive, shooter, intake);
    }
    // Named commands
    {
      // Auto named commands
      NamedCommands.registerCommand("test intake", new IntakeSetVoltage(intake, 12).withTimeout(1));
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
                      new WaitCommand(0.5), // TODO: maybe need to tune this too
                      new IntakeInOverride(intake)
                          .withTimeout(0.7)), // TODO: tune time in withTimeout
                  pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset),
                  new ShootSubwoofer(shooter))));

      NamedCommands.registerCommand( // shoot preloaded note to speaker, use at match start
          "preload speaker amp side",
          new SequentialCommandGroup(
              // new PrintCommand("preload im outta blush"),
              pivotShooter.zero(),
              new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                      new WaitCommand(0.8), // TODO: maybe need to tune this too
                      new IntakeInOverride(intake)
                          .withTimeout(0.7)), // TODO: tune time in withTimeout
                  pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset),
                  new ShootSubwoofer(shooter))
              // new PivotShooterSlamAndVoltage(pivotShooter)));
              ));
      NamedCommands.registerCommand( // intake ground note, stow to feeder chamber
          "intake sequence",
          new ParallelCommandGroup(
              pivotIntake.setPosition(PivotIntakeConstants.kPivotGroundPos),
              new IntakeIn(intake),
              // new PivotShooterSlamAndVoltage(pivotShooter),
              // new PivotShootSubwoofer(pivotShooter),
              new ShootSubwoofer(shooter)));
      NamedCommands.registerCommand( // outtake note to feeder
          "outtake speaker",
          new SequentialCommandGroup(
              // new ScheduleCommand(new PivotShootSubwoofer(pivotShooter)).asProxy(),
              new ParallelCommandGroup(
                  new IntakeInOverride(intake).withTimeout(2),
                  new ShootSubwoofer(shooter)))); // TODO: tune time in withTimeout
      NamedCommands.registerCommand("aim subwoofer", pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset));
      NamedCommands.registerCommand("shooter off", new ShooterOff(shooter));

      NamedCommands.registerCommand( // outtake note to feeder
          "safety",
          new ParallelCommandGroup(
              new IntakeIn(intake).withTimeout(1),
              new ShootAmp(shooter))); // TODO: tune time in withTimeout
      NamedCommands.registerCommand(
          "aim wing center",pivotShooter.setPosition(PivotShooterConstants.kWingNoteCenterPreset)); // wing note center
      NamedCommands.registerCommand(
          "aim wing side", pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)); // wing note side
      NamedCommands.registerCommand(
          "aim wing far side", pivotShooter.setPosition(PivotShooterConstants.kWingNoteFarSidePreset)); // wing note far side
      NamedCommands.registerCommand(
          "aim truss", pivotShooter.setPosition(PivotShooterConstants.kTrussSourceSidePreset)); // truss source sid
      NamedCommands.registerCommand(
          "aim half truss wing", pivotShooter.setPosition(PivotShooterConstants.kHalfWingPodiumPreset)); // half wing podium
      NamedCommands.registerCommand(
          "zero pivot shooter", pivotShooter.slamAndPID());

      NamedCommands.registerCommand( // rev shooter to speaker presets
          "rev speaker", new ShootSpeaker(shooter));
      NamedCommands.registerCommand( // rev shooter to amp presets
          "rev amp", new ShootAmp(shooter));
      NamedCommands.registerCommand( // modular pivot down, use for sabotage
          "pivot down", pivotIntake.setPosition(kPivotGroundPos).withTimeout(0.75));
      NamedCommands.registerCommand("stow", pivotIntake.slamAndPID().withTimeout(0.75));
      NamedCommands.registerCommand( // intake with no stow, use for sabotage
          "intake", new IntakeIn(intake));
      NamedCommands.registerCommand( // shoot preloaded note to amp, use at match start
          "preload amp",
          new SequentialCommandGroup(
              pivotIntake.zero(),
              new ParallelDeadlineGroup(
                  new SequentialCommandGroup(
                      new WaitCommand(0.8), // TODO: maybe need to tune this too
                      new IntakeOut(intake).withTimeout(1.5)), // TODO: tune time in withTimeout
                  new ShootAmp(shooter))));
      NamedCommands.registerCommand(
          "scheduled shoot speaker", new ScheduleCommand(new ShootSpeaker(shooter)));
      NamedCommands.registerCommand(
          "align to note",
          new SequentialCommandGroup(
              new RotateToNote(swerveDrive), new MoveToNote(swerveDrive, intake)));
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
    if (FeatureFlags.kSwerveEnabled) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
    }
    // Autos
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configurePivotShooter() {
    pivotShooter = new PivotShooter(new PivotShooterIOTalonFX());
    // operator.b().onTrue(new bruh(pivotShooter));
    // operator.x().onTrue(new SequentialCommandGroup(new
    // PivotShootSubwoofer(pivotShooter)));
    operator.x().onTrue(new SequentialCommandGroup(pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset)));
    operator
        .b()
        .onTrue(
            new SequentialCommandGroup(pivotShooter.setPosition(PivotShooterConstants.kWingNoteSidePreset)));
  }

  private void configureIntake() {
    intake = new Intake();
    // intake.setDefaultCommand(new IntakeSetVoltage(intake, 0));
    // operator.rightBumper().whileTrue(new IntakeInOverride(intake));
    // We assume intake is already enabled, so if pivot is enabled as
    // use IntakeOutWithArm
    operator.rightBumper().whileTrue(new IntakeAndPassthrough(intake));

    operator
        .leftBumper()
        .whileTrue(
            new SequentialCommandGroup(
                pivotShooter.setPosition(7),
                new ParallelCommandGroup(new GetRidOfNote(intake), new Shoot(shooter, 100, 100))));
    driver.rightTrigger().whileTrue(new IntakeOut(intake));

    // operator.povDown().onTrue(new IntakeOff(intake));
  }

  private void configurePivotIntake() {
    pivotIntake = new PivotIntake(new PivotIntakeIOTalonFX());
    operator.povRight().onTrue(pivotIntake.setPosition(kPivotGroundPos));
    operator.povLeft().onTrue(pivotIntake.slamAndPID());
  }

  private void configureClimb() {
    climb = new Climb();
    // zeroClimb = new ZeroClimb(climb); // NEED FOR SHUFFLEBOARD

    operator.povDown().onTrue(new ZeroClimb(climb));
    // new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5).onTrue(new
    // UpClimb(climb));
    new Trigger(() -> operator.getRawAxis(translationAxis) > 0.5).onTrue(new DownClimb(climb));
    if (this.ampbar != null && this.pivotShooter != null) {
      new Trigger(() -> operator.getRawAxis(translationAxis) < -0.5)
          .onTrue(
              Commands.sequence(
                  new ParallelCommandGroup(
                          new AmpPosition(ampbar),
                          pivotShooter.setPosition(12/138.33))
                      .withTimeout(1),
                  new UpClimb(climb)));
    } else {
      new Trigger(() -> Math.abs(operator.getRawAxis(secondaryAxis)) > 0.5)
          .onTrue(new DehookClimb(climb));
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
    swerveDrive = new SwerveDrive();
    swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            swerveDrive,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis)));

    driver
        .leftTrigger()
        .whileTrue(
            new TeleopSwerveLimited(
                swerveDrive,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis)));

    driver
        .rightTrigger()
        .whileTrue(
            new NoMoreRotation(
                swerveDrive,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                true,
                true));

    /* full reset */
    driver.y().onTrue(new ZeroGyro(swerveDrive));
    // driver.y().onTrue(new ForceResetModulePositions(swerveDrive));

    if (isRed) /* RED ALLIANCE PRESETS */ {

      /* AMP */
      driver
          .a()
          .onTrue(
              new Azimuth(
                      swerveDrive,
                      driver::getLeftY,
                      driver::getLeftX,
                      () -> aziAmpRed,
                      () -> true,
                      true,
                      true)
                  .withTimeout(aziCommandTimeOut));

      /* SOURCE */
      driver
          .rightBumper()
          .onTrue(
              new Azimuth(
                      swerveDrive,
                      driver::getLeftY,
                      driver::getLeftX,
                      () -> aziSourceRed,
                      () -> true,
                      true,
                      true)
                  .withTimeout(aziCommandTimeOut));

      /* FEEDER */
      driver
          .povRight()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> feederRed,
                  () -> true,
                  true,
                  true));

    } else /* BLUE ALLIANCE PRESETS */ {

      /* AMP */
      driver
          .a()
          .onTrue(
              new Azimuth(
                      swerveDrive,
                      driver::getLeftY,
                      driver::getLeftX,
                      () -> aziAmpBlue,
                      () -> true,
                      true,
                      true)
                  .withTimeout(aziCommandTimeOut));

      /* SOURCE */
      driver
          .rightBumper()
          .onTrue(
              new Azimuth(
                      swerveDrive,
                      driver::getLeftY,
                      driver::getLeftX,
                      () -> aziSourceBlue,
                      () -> true,
                      true,
                      true)
                  .withTimeout(aziCommandTimeOut));

      /* FEEDER */
      driver
          .povRight()
          .onTrue(
              new Azimuth(
                  swerveDrive,
                  driver::getLeftY,
                  driver::getLeftX,
                  () -> feederBlue,
                  () -> true,
                  true,
                  true));
    }

    /* SUBWOOFER FRONT */
    driver
        .leftBumper()
        .onTrue(
            new Azimuth(
                    swerveDrive,
                    driver::getLeftY,
                    driver::getLeftX,
                    () -> aziSubwooferFront,
                    () -> true,
                    true,
                    true)
                .withTimeout(aziCommandTimeOut));

    /* SUBWOOFER RIGHT */
    driver
        .b()
        .onTrue(
            new Azimuth(
                    swerveDrive,
                    driver::getLeftY,
                    driver::getLeftX,
                    () -> aziSubwooferRight,
                    () -> true,
                    true,
                    true)
                .withTimeout(aziCommandTimeOut));

    /* SUBWOOFER LEFT */
    driver
        .x()
        .onTrue(
            new Azimuth(
                    swerveDrive,
                    driver::getLeftY,
                    driver::getLeftX,
                    () -> aziSubwooferLeft,
                    () -> true,
                    true,
                    true)
                .withTimeout(aziCommandTimeOut));

    /* CLEANUP */
    driver
        .povDown()
        .onTrue(
            new Azimuth(
                    swerveDrive,
                    driver::getLeftY,
                    driver::getLeftX,
                    () -> cleanUp,
                    () -> true,
                    true,
                    true)
                .withTimeout(aziCommandTimeOut));
  }

  private void configureShooter() {
    if (isReal()) {
      shooter = new Shooter();
    } else {
      shooter = new Shooter();
    }
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
      ampbar = new AmpBar();
      operator
          .rightTrigger()
          .onTrue(Commands.parallel(new ShootSpeaker(shooter), new StowPosition(ampbar)));
      operator
          .leftTrigger()
          .onTrue(
              new ParallelCommandGroup(
                  new ShootAmp(shooter),
                  new AmpPosition(ampbar),
                  pivotShooter.setPosition(kAmpPreset)));
      operator
          .y()
          .onTrue(
              new ParallelCommandGroup(
                  new ShooterOff(shooter),
                  new StowPosition(ampbar),
                  pivotShooter.slamAndPID()));
    } else {
      operator.rightTrigger().onTrue(new ShootSpeaker(shooter));
      operator.leftTrigger().onTrue(new ShootAmp(shooter));
      operator
          .y()
          .onTrue(
              new ParallelCommandGroup(
                  new ShooterOff(shooter), pivotShooter.slamAndPID()));
    }
  }

  private void configureOperatorAutos() {
    operator.a().onTrue(new IntakeSequence(intake, pivotIntake, pivotShooter, shooter, ampbar));
    operator
        .povUp()
        .onTrue(
            new ParallelCommandGroup(
                pivotShooter.setPosition(PivotShooterConstants.kFeederPreset), new ShootFeed(shooter)));
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
      Trigger intakeDetectedNote = new Trigger(intake::isBeamBroken);
      // intakeDetectedNote.whileTrue(new SetSuccessfulIntake(led));

      intakeDetectedNote
          .onTrue(
              new InstantCommand(
                  () -> {
                    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                  }))
          .onFalse(
              new InstantCommand(
                  () -> {
                    operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                  }));

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

  private void systemCheeks() {
    if (FeatureFlags.kSwerveEnabled && swerveDrive != null) {
      swerveDrive.CANTest();
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /* Test Routines */
  public boolean CANTest() {
    return swerveDrive.CANTest();
  }

  public void runPitTestRoutine() {
    // Command pitRoutine = new PitRoutine(swerveDrive, climb, intake, pivotIntake,
    // shooter);
    // pitRoutine.schedule();
  }

  public void ccccccc() {
    Command shoot = new ShootSpeaker(shooter);
    shoot.schedule();
  }

  public void shootSpeaker() {
    Command shoot = new ScheduleCommand(new ShootSpeaker(shooter));
    shoot.schedule();
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
