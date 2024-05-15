package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {

      private final VisionIO visionIO;
      private final VisionIOInputsAutoLogged visionIOAutoLogged = new VisionIOInputsAutoLogged();

      private double lastLastCenterLimelightX = 0.0;
      private double lastLastCenterLimelightY = 0.0;
      private double lastCenterLimelightX = 0.0;
      private double lastCenterLimelightY = 0.0;
      private double centerLimelightX = 0.0;
        private double centerLimelightY = 0.0;

      public Vision(VisionIO visionIO) {
        this.visionIO = visionIO;
      }

      @Override
      public void periodic() {
        visionIO.updateInputs(visionIOAutoLogged);

        lastLastCenterLimelightX = lastCenterLimelightX;
        lastLastCenterLimelightY = lastCenterLimelightY;
        lastCenterLimelightX = centerLimelightX;
        lastCenterLimelightY = centerLimelightY;
        centerLimelightX = visionIOAutoLogged.centerLimelightX;
        centerLimelightY = visionIOAutoLogged.centerLimelightY;

      }

        public double getCenterLimelightX() {
            return centerLimelightX;
        }

        public double getCenterLimelightY() {
            return centerLimelightY;
        }

        public double getLastCenterLimelightX() {
            return lastCenterLimelightX;
        }

        public double getLastCenterLimelightY() {
            return lastCenterLimelightY;
        }

        public double getLastLastCenterLimelightX() {
            return lastLastCenterLimelightX;
        }

        public double getLastLastCenterLimelightY() {
            return lastLastCenterLimelightY;
        }
        @AutoLogOutput
        public double getCompensatedCenterLimelightX() {
            return centerLimelightX + (lastCenterLimelightX - lastLastCenterLimelightX);
        }
        @AutoLogOutput
        public double getCompensatedCenterLimelightY() {
            return centerLimelightY + (lastCenterLimelightY - lastLastCenterLimelightY);
        }

}
