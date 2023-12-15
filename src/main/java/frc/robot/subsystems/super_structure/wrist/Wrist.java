package frc.robot.subsystems.super_structure.wrist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.super_structure.Component;

public interface Wrist extends Component {

    public static class WristInputs implements LoggableInputs {
        public double degrees, targetDegrees;
        public double degreesPerSec = 0.0;
        public double amps = 0.0;
        public double volts = 0.0;
        public double temp = 0.0;
        public boolean isHomed = false;

        public WristInputs(Double startingDegrees) {
            degrees = startingDegrees;
            targetDegrees = startingDegrees;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("WristDegrees", degrees);
            table.put("WristDegreesPerSec", degreesPerSec);
            table.put("WristAmps", amps);
            table.put("WristVolts", volts);
            table.put("WristTemp", temp);
            table.put("WristTargetDegrees", targetDegrees);
            table.put("WristIsHomed", isHomed);
        }

        @Override
        public void fromLog(LogTable table) {
            degrees = table.get("WristDegrees", degrees);
            degreesPerSec = table.get("WristDegreesPerSec", degreesPerSec);
            amps = table.get("WristAmps", amps);
            volts = table.get("WristVolts", volts);
            temp = table.get("WristTemp", temp);
            targetDegrees = table.get("WristTargetDegrees", targetDegrees);
            isHomed = table.get("WristIsHomed", isHomed);
        }
    }

    /**
     * Abstract from motors, will set the wrist degrees.
     * Parallel to the elevator is 0 degrees
     * 
     * @return true if degrees has been reached
     */
    public boolean setWristDegrees(Double degrees);

    /**
     * @return the current angle of the mechanism
     */
    public Double getWristDegrees();
}
