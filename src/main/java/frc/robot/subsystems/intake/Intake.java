// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.helpers.TimedBoolean;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeIOAutoLogged = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine intake_sysIdRoutine;
  private final SysIdRoutine passthrough_sysIdRoutine;
  public Intake(IntakeIO intakeIO) {

    this.intakeIO = intakeIO;
    intake_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Seconds.of(1)),        // Use default ramp rate (1 V/s)
                    Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                    null,        // Use default timeout (10 s)
                    // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> intakeIO.getIntakeMotor().setControl(intakeIO.getIntakeVoltageRequest().withOutput(volts.in(Volts))),
                    null,
                    this
            )
    );
    passthrough_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Seconds.of(1)),        // Use default ramp rate (1 V/s)
                    Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                    null,        // Use default timeout (10 s)
                    // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> intakeIO.getPassthroughMotor().setControl(intakeIO.getPassthroughVoltageRequest().withOutput(volts.in(Volts))),
                    null,
                    this
            )
    );
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), intakeIOAutoLogged);
  }

  public Command setVoltage(double voltage, double passthroughVoltage) {
    return new StartEndCommand(
        () -> {
          intakeIO.setIntakeVoltage(voltage);
          intakeIO.setPassthroughVoltage(passthroughVoltage);
        },
        () -> intakeIO.off(),
        this);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return new StartEndCommand(
        () -> {
          intakeIO.setIntakeVelocity(velocity);
          intakeIO.setPassthroughVelocity(passthroughVelocity);
        },
        () -> intakeIO.off(),
        this);
  }

  public Command setIntakeVoltage(double voltage) {
    return new StartEndCommand(
        () -> intakeIO.setIntakeVoltage(voltage), () -> intakeIO.off(), this);
  }

  public Command setIntakeVelocity(double velocity) {
    return new StartEndCommand(
        () -> intakeIO.setIntakeVelocity(velocity), () -> intakeIO.off(), this);
  }

  public Command setPassthroughVoltage(double voltage) {
    return new StartEndCommand(
        () -> intakeIO.setPassthroughVoltage(voltage), () -> intakeIO.off(), this);
  }

  public Command setPassthroughVelocity(double velocity) {
    return new StartEndCommand(
        () -> intakeIO.setPassthroughVelocity(velocity), () -> intakeIO.off(), this);
  }

  public Command off() {
    return new StartEndCommand(() -> intakeIO.off(), () -> {}, this);
  }

  public Command intakeIn() {
    return new Command() {
      TimedBoolean beamBreak =
          new TimedBoolean(intakeIO::isBeamBroken, IntakeConstants.kBeamBreakDelayTime);

      @Override
      public void initialize() {
        intakeIO.setIntakeVoltage(IntakeConstants.kIntakeIntakeVoltage);
        intakeIO.setPassthroughVoltage(IntakeConstants.kPassthroughIntakeVoltage);
      }

      @Override
      public void execute() {
        beamBreak.update();
      }

      @Override
      public boolean isFinished() {
        return beamBreak.hasBeenTrueForThreshold();
      }
    };
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
