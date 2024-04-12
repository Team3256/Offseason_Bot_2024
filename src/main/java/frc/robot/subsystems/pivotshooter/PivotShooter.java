// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.utils.SinglePositionSubsystem;

public class PivotShooter extends SinglePositionSubsystem {

  public PivotShooter() {
    super(PivotShooterConstants.kUseMotionMagic, PivotShooterConstants.kCurrentThreshold, PivotShooterConstants.kStallVelocityThreshold);
    super.configureRealHardware(PivotShooterConstants.kPivotMotorID, NeutralModeValue.Brake, PivotShooterConstants.kS, PivotShooterConstants.kV, PivotShooterConstants.kP, PivotShooterConstants.kI, PivotShooterConstants.kD, PivotShooterConstants.motionMagicVelocity, PivotShooterConstants.motionMagicAcceleration, PivotShooterConstants.motionMagicJerk, PivotShooterConstants.enableStatorLimit, PivotShooterConstants.statorLimit);
    if(!RobotBase.isReal()) {
      super.configureSimHardware(PivotShooterConstants.kNumPivotMotors, PivotShooterConstants.kPivotMotorGearing, PivotShooterConstants.jKgMetersSquared,PivotShooterConstants.kPivotLength, PivotShooterConstants.kPivotMinAngleDeg, PivotShooterConstants.kPivotMaxAngleDeg, false);
    }
  }

  @Override
   public void off() {
       super.off();
       if (Constants.FeatureFlags.kDebugEnabled) {
           System.out.println("Pivot off");
         }
   }
   @Override
   public void setOutputVoltage(double voltage) {
       super.setOutputVoltage(voltage);
       if (Constants.FeatureFlags.kDebugEnabled) {
           System.out.println("Pivot Shooter voltage set to: " + voltage);
       }
   }


   @Override
   public void setDegrees(double degrees) {
       super.setDegrees(degrees);
       if (Constants.FeatureFlags.kDebugEnabled) {
           System.out.println("Pivot Shooter set to: " + degrees);
       }
   }
   @Override
   public void zero() {
       super.zero();
       if (Constants.FeatureFlags.kDebugEnabled) {
           System.out.println("[PivotShooter] Setting zero position to: " + getDegrees());
       }
   }
}