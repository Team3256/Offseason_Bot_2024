package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public double noteLimelightX = 0.0;
        public double noteLimelightY = 0.0;

        public double centerLimelightX = 0.0;
        public double centerLimelightY = 0.0;

    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
