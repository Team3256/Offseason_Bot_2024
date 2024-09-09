// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ampbar.AmpBar;
import frc.robot.subsystems.ampbar.AmpBarConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivotintake.PivotIntake;
import frc.robot.subsystems.pivotintake.PivotIntakeConstants;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum StructureState {
    IDLE,
    HOMED,
    PREINTAKE,
    INTAKE,
    PRECLIMB,
    CLIMB,
    PRESUB,
    PREPODIUM,
    PREFEED,
    SHOOT,
    PREAMP,
    OUTTAKE,
  }

  private final AmpBar ampBar;
  private final Climb climb;
  private final Intake intake;
  private final PivotIntake pivotIntake;
  private final PivotShooter pivotShooter;
  private final Shooter shooter;
  private final Vision vision;

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Timer stateTimer = new Timer();

  public Superstructure(
      AmpBar ampBar,
      Climb climb,
      Intake intake,
      PivotIntake pivotIntake,
      PivotShooter pivotShooter,
      Shooter shooter,
      Vision vision) {
    this.ampBar = ampBar;
    this.climb = climb;
    this.intake = intake;
    this.pivotIntake = pivotIntake;
    this.pivotShooter = pivotShooter;
    this.shooter = shooter;
    this.vision = vision;

    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }

    configStateTransitions();
  }

  public void configStateTransitions() {
    stateTriggers
        .get(StructureState.IDLE)
        .onTrue(ampBar.off())
        .onTrue(climb.off())
        .onTrue(intake.off())
        .onTrue(pivotIntake.off())
        .onTrue(pivotShooter.off())
        .onTrue(shooter.off());
    stateTriggers
        .get(StructureState.HOMED)
        .onTrue(ampBar.setStowPosition())
        .onTrue(climb.retractClimber())
        .onTrue(intake.off())
        .onTrue(pivotIntake.homePosition())
        .onTrue(pivotShooter.homePosition())
        .onTrue(shooter.off());
    stateTriggers
        .get(StructureState.PREINTAKE)
        .onTrue(pivotIntake.intakePosition())
        .onTrue(pivotShooter.homePosition())
        .and(() -> !intake.isBeamBroken())
        .onTrue(this.setState(StructureState.INTAKE));
    stateTriggers
        .get(StructureState.INTAKE)
        .whileTrue(intake.intakeIn())
        .and(() -> intake.isBeamBroken())
        .onTrue(this.setState(StructureState.PRESUB));
    stateTriggers
        .get(StructureState.PRESUB)
        .onTrue(pivotIntake.homePosition())
        .onTrue(
            pivotShooter.setPosition(
                PivotShooterConstants.kSubWooferPreset * PivotShooterConstants.kPivotMotorGearing))
        .onTrue(ampBar.setStowPosition())
        .onTrue(intake.off())
        .onTrue(
            shooter.setVelocity(
                ShooterConstants.kShooterSubwooferRPS,
                ShooterConstants.kShooterFollowerSubwooferRPS));

    stateTriggers
        .get(StructureState.PREPODIUM)
        .onTrue(pivotIntake.homePosition())
        .onTrue(
            pivotShooter.setPosition(
                PivotShooterConstants.kPodiumRPreset * PivotShooterConstants.kPivotMotorGearing))
        .onTrue(ampBar.setStowPosition())
        .onTrue(intake.off())
        .onTrue(
            shooter.setVelocity(
                ShooterConstants.kShooterSpeakerRPS, ShooterConstants.kShooterFollowerSpeakerRPS));

    stateTriggers
        .get(StructureState.PREFEED)
        .onTrue(pivotIntake.homePosition())
        .onTrue(
            pivotShooter.setPosition(
                PivotShooterConstants.kFeederPreset * PivotShooterConstants.kPivotMotorGearing))
        .onTrue(ampBar.setStowPosition())
        .onTrue(intake.off())
        .onTrue(
            shooter.setVelocity(
                ShooterConstants.kShooterFeederRPS, ShooterConstants.kShooterFollowerFeederRPS));

    stateTriggers
        .get(StructureState.SHOOT)
        .and(pivotShooter::isAtGoal)
        .and(shooter::isAtGoal)
        .and(ampBar::isAtGoal)
        .onTrue(intake.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage))
        .and(() -> !intake.isBeamBroken())
        .debounce(1)
        .onTrue(this.setState(StructureState.HOMED));
    stateTriggers
        .get(StructureState.PRECLIMB)
        .onTrue(pivotIntake.homePosition())
        .onTrue(
            pivotShooter.setPosition(
                PivotShooterConstants.kClimbPreset * PivotShooterConstants.kPivotMotorGearing))
        .onTrue(ampBar.setAmpPosition())
        .onTrue(intake.off())
        .onTrue(shooter.off())
        .and(
            () ->
                MathUtil.isNear(
                    pivotShooter.getPosition(),
                    PivotShooterConstants.kClimbPreset * PivotShooterConstants.kPivotMotorGearing,
                    0.5))
        .and(() -> MathUtil.isNear(pivotIntake.getPosition(), 0, 0.5))
        .and(
            () ->
                MathUtil.isNear(
                    ampBar.getPosition(),
                    AmpBarConstants.kAmpBarAmpPosition * AmpBarConstants.kAmpBarGearing,
                    0.5))
        .onTrue(climb.extendClimber());

    stateTriggers.get(StructureState.CLIMB).and(climb::isExtended).onTrue(climb.retractClimber());

    stateTriggers
        .get(StructureState.PREAMP)
        .onTrue(pivotIntake.homePosition())
        .onTrue(
            pivotShooter.setPosition(
                PivotShooterConstants.kAmpPreset * PivotShooterConstants.kPivotMotorGearing))
        .onTrue(ampBar.setAmpPosition())
        .onTrue(intake.off())
        .onTrue(
            shooter.setVelocity(
                ShooterConstants.kShooterAmpRPS, ShooterConstants.kShooterFollowerAmpRPS));

    stateTriggers
        .get(StructureState.OUTTAKE)
        .onTrue(
            pivotIntake.setPosition(
                PivotIntakeConstants.kPivotGroundPos * PivotIntakeConstants.kPivotMotorGearing))
        .whileTrue(
            intake.setVoltage(
                -IntakeConstants.kIntakeIntakeVoltage, -IntakeConstants.kPassthroughIntakeVoltage));
  }
  // call manually
  public void periodic() {
    Logger.recordOutput(this.getClass().getSimpleName() + "/State", this.state.toString());
    Logger.recordOutput(this.getClass().getSimpleName() + "/PrevState", this.prevState.toString());
    Logger.recordOutput(this.getClass().getSimpleName() + "/StateTime", this.stateTimer.get());
  }

  public Command setState(StructureState state) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state;
          this.state = state;
          this.stateTimer.restart();
        });
  }
}
