// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends DisableSubsystem {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeIOAutoLogged = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine intake_sysIdRoutine;
  private final SysIdRoutine passthrough_sysIdRoutine;

  private final Trigger debouncedBeamBreak = new Trigger(this::isBeamBroken).debounce(0.1);
  ;

  public Intake(boolean disabled, IntakeIO intakeIO) {
    super(disabled);

    this.intakeIO = intakeIO;
    intake_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1)), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    intakeIO
                        .getIntakeMotor()
                        .setControl(intakeIO.getIntakeVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
    passthrough_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1)), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    intakeIO
                        .getPassthroughMotor()
                        .setControl(
                            intakeIO.getPassthroughVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    super.periodic();
    intakeIO.updateInputs(intakeIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), intakeIOAutoLogged);
  }

  public Command setVoltage(double voltage, double passthroughVoltage) {
    return this.run(
            () -> {
              intakeIO.setIntakeVoltage(voltage);
              intakeIO.setPassthroughVoltage(passthroughVoltage);
            })
        .finallyDo(intakeIO::off);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return this.run(
            () -> {
              intakeIO.setIntakeVelocity(velocity);
              intakeIO.setPassthroughVelocity(passthroughVelocity);
            })
        .finallyDo(intakeIO::off);
  }

  public Command setIntakeVoltage(double voltage) {
    return this.run(() -> intakeIO.setIntakeVoltage(voltage)).finallyDo(intakeIO::off);
  }

  public Command setIntakeVelocity(double velocity) {
    return this.run(() -> intakeIO.setIntakeVelocity(velocity)).finallyDo(intakeIO::off);
  }

  public Command setPassthroughVoltage(double voltage) {
    return this.run(() -> intakeIO.setPassthroughVoltage(voltage)).finallyDo(intakeIO::off);
  }

  public Command setPassthroughVelocity(double velocity) {
    return this.run(() -> intakeIO.setPassthroughVelocity(velocity)).finallyDo(intakeIO::off);
  }

  public Command off() {
    return this.runOnce(intakeIO::off);
  }

  public Command intakeIn() {
    return this.run(
            () -> {
              intakeIO.setIntakeVoltage(IntakeConstants.kIntakeIntakeVoltage);
              intakeIO.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage);
            })
        .until(debouncedBeamBreak)
        .andThen(this.off());
  }

  public Command intakeSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intake_sysIdRoutine.quasistatic(direction);
  }

  public Command intakeSysIdDynamic(SysIdRoutine.Direction direction) {
    return intake_sysIdRoutine.dynamic(direction);
  }

  public Command passthroughSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return passthrough_sysIdRoutine.quasistatic(direction);
  }

  public Command passthroughSysIdDynamic(SysIdRoutine.Direction direction) {
    return passthrough_sysIdRoutine.dynamic(direction);
  }

  public boolean isBeamBroken() {
    return intakeIO.isBeamBroken();
  }
}
