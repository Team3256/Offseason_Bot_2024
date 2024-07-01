// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class PivotIntake extends SubsystemBase {

  private final PivotIntakeIO pivotIntakeIO;
  private final PivotIntakeIOInputsAutoLogged pivotIntakeIOAutoLogged =
      new PivotIntakeIOInputsAutoLogged();
  private final SysIdRoutine m_sysIdRoutine;

  public PivotIntake(PivotIntakeIO pivotIntakeIO) {

    this.pivotIntakeIO = pivotIntakeIO;
    m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Seconds.of(1)),        // Use default ramp rate (1 V/s)
                    Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                    null,        // Use default timeout (10 s)
                    // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> pivotIntakeIO.getMotor().setControl(pivotIntakeIO.getVoltageRequest().withOutput(volts.in(Volts))),
                    null,
                    this
            )
    );
  }

  @Override
  public void periodic() {
    pivotIntakeIO.updateInputs(pivotIntakeIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), pivotIntakeIOAutoLogged);
  }

  public Command setPosition(double position) {
    return new StartEndCommand(
        () -> pivotIntakeIO.setPosition(position * PivotIntakeConstants.kPivotMotorGearing),
        () -> {},
        this);
  }

  public Command setVoltage(double voltage) {
    return new StartEndCommand(
        () -> pivotIntakeIO.setVoltage(voltage), () -> pivotIntakeIO.setVoltage(0), this);
  }

  public Command off() {
    return new StartEndCommand(() -> pivotIntakeIO.off(), () -> {}, this);
  }

  public Command slamZero() {
    return new Command() {
      @Override
      public void initialize() {
        pivotIntakeIO.setVoltage(PivotIntakeConstants.kPivotSlamShooterVoltage);
      }

      @Override
      public void end(boolean interrupted) {
        pivotIntakeIO.off();
        if (!interrupted) {
          pivotIntakeIO.zero();
        }
      }

      @Override
      public boolean isFinished() {
        return pivotIntakeIOAutoLogged.pivotIntakeMotorStatorCurrent
            > PivotIntakeConstants.kPivotSlamStallCurrent;
      }
    };
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command slamAndPID() {
    return new SequentialCommandGroup(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return new StartEndCommand(() -> pivotIntakeIO.zero(), () -> {}, this);
  }
}
