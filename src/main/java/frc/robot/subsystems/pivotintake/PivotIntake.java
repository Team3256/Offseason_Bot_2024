// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class PivotIntake extends DisableSubsystem {

  private final PivotIntakeIO pivotIntakeIO;
  private final PivotIntakeIOInputsAutoLogged pivotIntakeIOAutoLogged =
      new PivotIntakeIOInputsAutoLogged();
  private final SysIdRoutine m_sysIdRoutine;

  public PivotIntake(boolean disabled, PivotIntakeIO pivotIntakeIO) {
    super(disabled);

    this.pivotIntakeIO = pivotIntakeIO;
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds.of(0.5)), // Use default ramp rate (1 V/s)
                Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    pivotIntakeIO
                        .getMotor()
                        .setControl(pivotIntakeIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    super.periodic();
    pivotIntakeIO.updateInputs(pivotIntakeIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), pivotIntakeIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(() -> pivotIntakeIO.setPosition(position));
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> pivotIntakeIO.setVoltage(voltage)).finallyDo(pivotIntakeIO::off);
  }

  public Command off() {
    return this.runOnce(pivotIntakeIO::off);
  }

  public Command slamZero() {
    return this.run(() -> pivotIntakeIO.setVoltage(PivotIntakeConstants.kPivotSlamShooterVoltage))
        .until(
            () ->
                pivotIntakeIOAutoLogged.pivotIntakeMotorStatorCurrent
                    > PivotIntakeConstants.kPivotSlamStallCurrent)
        .andThen(this.zero());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command slamAndPID() {

    return Commands.sequence(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return this.runOnce(pivotIntakeIO::zero);
  }
}
