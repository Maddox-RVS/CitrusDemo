package frc.robot.subsystems.super_structure.pivot;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.super_structure.Component;

public interface Pivot extends Component {

    public static class PivotInputs implements LoggableInputs {
        public double degrees, targetDegrees;
        public double degreesPerSec = 0.0;
        public double leftAmps = 0.0, rightAmps = 0.0;
        public double volts = 0.0;
        public double leftTemp = 0.0, rightTemp = 0.0;
        public double gyroPitchDegrees;
        public boolean isHomed = false;

        public PivotInputs(Double startingDegrees) {
            degrees = startingDegrees;
            targetDegrees = startingDegrees;
            gyroPitchDegrees = startingDegrees;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("Degrees", degrees);
            table.put("DegreesPerSec", degreesPerSec);
            table.put("LeftAmps", leftAmps);
            table.put("RightAmps", rightAmps);
            table.put("Volts", volts);
            table.put("LeftTemp", leftTemp);
            table.put("RightTemp", rightTemp);
            table.put("TargetDegrees", targetDegrees);
            table.put("GyroPitchDegrees", gyroPitchDegrees);
        }

        @Override
        public void fromLog(LogTable table) {
            degrees = table.get("Degrees", degrees);
            degreesPerSec = table.get("DegreesPerSec", degreesPerSec);
            leftAmps = table.get("LeftAmps", leftAmps);
            rightAmps = table.get("RightAmps", rightAmps);
            volts = table.get("Volts", volts);
            leftTemp = table.get("LeftTemp", leftTemp);
            rightTemp = table.get("RightTemp", rightTemp);
            targetDegrees = table.get("TargetDegrees", targetDegrees);
            gyroPitchDegrees = table.get("GyroPitchDegrees", gyroPitchDegrees);
        }
    }

    /**
     * Abstract from motors, will set the wrist degrees.
     * Parallel to the elevator is 0 degrees
     * 
     * @return true if degrees has been reached
     */
    public boolean setPivotDegrees(Double degrees);

    /**
     * @return the current angle of the mechanism
     */
    public Double getPivotDegrees();
}
