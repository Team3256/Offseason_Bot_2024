package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.utils.DualVelocitySubsystem;

public class Intake extends DualVelocitySubsystem {
    private DigitalInput beamBreakInput;
    public Intake() {
        super(IntakeConstants.kUseIntakeMotionMagic, IntakeConstants.kUsePassthroughMotionMagic, IntakeConstants.kIntakeCurrentThreshold, IntakeConstants.kIntakeCurrentThreshold, IntakeConstants.kIntakeVelocitySpiking, IntakeConstants.kPassthroughVelocitySpiking);
        beamBreakInput = new DigitalInput(IntakeConstants.kIntakeBeamBreakDIO);
        super.configureRealHardware(IntakeConstants.kIntakeMotorID, IntakeConstants.kPassthroughMotorID, NeutralModeValue.Coast, NeutralModeValue.Brake, IntakeConstants.kIntakeKS, IntakeConstants.kIntakeKV, IntakeConstants.kIntakeKP, IntakeConstants.kIntakeKI, IntakeConstants.kIntakeKD, IntakeConstants.kPassthroughkS, IntakeConstants.kPassthroughkV, IntakeConstants.kPassthroughkP, IntakeConstants.kPassthroughkI, IntakeConstants.kPassthroughkD, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive);
        if(!RobotBase.isReal()) {
            super.configureSimHardware();
        }
    }

    public void setOutputVoltageIntake(double voltage) {
        super.setOutputVoltageUno(voltage);
      }
    
      public void setOutputVoltagePassthrough(double voltage) {
        super.setOutputVoltageDos(voltage);
      }
    
    
      public void setIntakeVelocity(double velocity) {
        super.setUnoVelocity(velocity);
        if (Constants.FeatureFlags.kDebugEnabled) {
          System.out.println("Intake velocity set to: " + velocity);
        }
      }

      public void setPassthroughVelocity(double velocity) {
        super.setDosVelocity(velocity);
        if (Constants.FeatureFlags.kDebugEnabled) {
          System.out.println("Intake velocity set to: " + velocity);
        }
      }

    
      @AutoLogOutput
      public boolean isIntakeMotorCurrentSpiking() {
        return super.isUnoMotorCurrentSpiking();
      }
    
      @AutoLogOutput
      public boolean isPassthroughMotorCurrentSpiking() {
        return super.isDosMotorCurrentSpiking();
      }
    
      @AutoLogOutput(key = "intake velocity")
      public double getIntakeVelocity() {
       return super.getUnoVelocity();
      }
    
      @AutoLogOutput(key = "passthrough velocity")
      public double getPassthroughVelocity() {
        return super.getDosVelocity();
      }
    
      @AutoLogOutput(key = "intake velocity spiking")
      public boolean isIntakeMotorVelocitySpiking() {
        return super.isUnoMotorVelocitySpiking();
      }
    
      @AutoLogOutput(key = "passthrough velocity spiking")
      public boolean isPassthroughMotorVelocitySpiking() {
        return super.isDosMotorVelocitySpiking();
      }

    
    
      @AutoLogOutput
      public double getIntakeMotorVoltage() {
        return super.getUnoMotorVoltage();
      }
    
      @AutoLogOutput
      public double getIntakeCurrent() {
        return super.getUnoCurrent();
      }
    
    
      @AutoLogOutput
      public double getPassthroughMotorVoltage() {
        return super.getDosMotorVoltage();
      }
    
      @AutoLogOutput
      public double getPassthroughCurrent() {
        return super.getDosCurrent();
      }
    
      public void off() {
        super.off();
        if (Constants.FeatureFlags.kDebugEnabled) {
          System.out.println("Intake off");
        }
      }
    
      @AutoLogOutput
      public boolean isBeamBroken() {
        return !beamBreakInput.get();
      }
    
      @Override
      public void periodic() {
        // System.out.println("TOF DISTANCE: " + getTOFDistance());
      }
    
      public void intakeIn() {
        setIntakeVelocity(IntakeConstants.kIntakeNoteRPM / 60);
      }
    
      public void intakeOut() {
        setIntakeVelocity(IntakeConstants.kOuttakeNoteRPM / 60);
      }
    
    
}
