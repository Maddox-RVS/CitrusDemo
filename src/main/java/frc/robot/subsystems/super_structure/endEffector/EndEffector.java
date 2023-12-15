package frc.robot.subsystems.super_structure.endEffector;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.super_structure.Component;

public interface EndEffector extends Component {

    public static class EndEffectorInputs implements LoggableInputs {
        public double amps = 0.0;
        public double volts = 0.0;
        public double temp = 0.0;
        public double currentLimit = 0.0;
        public boolean isInverted = false;

        @Override
        public void toLog(LogTable table) {
            table.put("Amps", amps);
            table.put("Volts", volts);
            table.put("Temp", temp);
            table.put("CurrentLimit", currentLimit);
            table.put("IsInverted", isInverted);
        }

        @Override
        public void fromLog(LogTable table) {
            amps = table.get("Amps", amps);
            volts = table.get("Volts", volts);
            temp = table.get("Temp", temp);
            currentLimit = table.get("CurrentLimit", currentLimit);
            isInverted = table.get("IsInverted", isInverted);
        }
    }

    /**
     * Runs the end-effector at a given percent output
     * 
     * @param volts of the end-effector motor
     */
    public void runEndEffector(Double volts);

    /**
     * @return the voltage of the end-effector motor
     */
    public Double getEndEffectorVoltage();

    /**
     * @param amps true to enable current limits, false to disable
     */
    public void setEndEffectorCurrentLimits(Double amps);

    @Override
    default public void manualDriveMechanism(Double percentOut) {
        this.runEndEffector(percentOut * 12.0);
    }

    @Override
    default public boolean homeMechanism(boolean force) {
        return true;
    }

    @Override
    default void stopMechanism() {
        this.runEndEffector(0.0);
    }
}
