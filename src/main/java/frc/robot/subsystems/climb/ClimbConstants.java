// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;

public final class ClimbConstants {

  public static final int kLeftClimbMotorID = 18;
  public static final double gearRatio = 20; // needs to be tuned

  public static final double kClimbUpPosition = 150 / 20;

  public static final double kClimbDownPosition = 0;
  public static final double wheelRadius = 1;

  public static double kCurrentThreshold = 4.5;

  // // What about the tension from the spring?
  // public static double gyroRollStableThreshold = 1; // 1 degree of error is
  // tolerated
  public static double updateFrequency;
  public static boolean kUseMotionMagic = false;

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKP(1).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(100))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));

  public static final String kBotPoseLimelightId = "todo";
  public static final ArrayList<Pose2d> kStagePosesBlue =
      new ArrayList<>() {
        {
          // Podium
          add(new Pose2d(2.657515287399292, 4.105274677276611, new Rotation2d(0)));
          // Amp side
          add(new Pose2d(5.990447521209717, 6.171302795410156, new Rotation2d(-2.015216124571914)));
          // Etc
          add(new Pose2d(6.10739278793335, 2.1172094345092773, new Rotation2d(2.118189174278151)));
        }
      };
  public static final ArrayList<Pose2d> kStagePosesRed =
      new ArrayList<>() {
        {
          // Podium
          add(new Pose2d(13.981689453125, 4.105274677276611, new Rotation2d(3.1415926536)));
          // Amp side
          add(
              new Pose2d(
                  10.512321472167969, 6.11283016204834, new Rotation2d(-1.0445000982232164)));
          // Etc
          add(
              new Pose2d(
                  10.492830276489258, 2.0977187156677246, new Rotation2d(1.0214219124306612)));
        }
      };
}
