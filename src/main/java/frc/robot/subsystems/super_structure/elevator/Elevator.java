package frc.robot.subsystems.super_structure.elevator;

import frc.robot.subsystems.super_structure.Component;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Constants.kSuperStructure.kElevator;

public interface Elevator extends Component {

    public static class ElevatorInputs implements LoggableInputs{
        public double meters, targetMeters;
        public double metersPerSec;
        public double volts = 0.0;
        public double leftAmps = 0.0, rightAmps = 0.0;
        public double leftTemp = 0.0, rightTemp = 0.0;
        public boolean isLimitSwitchHit = false;
        public boolean isHomed = false;

        public ElevatorInputs(Double startingMeters) {
            this.meters = startingMeters;
            this.targetMeters = startingMeters;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("ElevatorMeters", meters);
            table.put("ElevatorTargetMeters", targetMeters);
            table.put("ElevatorMetersPerSec", metersPerSec);
            table.put("ElevatorVolts", volts);
            table.put("ElevatorLeftAmps", leftAmps);
            table.put("ElevatorRightAmps", rightAmps);
            table.put("ElevatorLeftTemp", leftTemp);
            table.put("ElevatorRightTemp", rightTemp);
            table.put("ElevatorLimitSwitch Hit", isLimitSwitchHit);
            table.put("ElevatorHomed", isHomed);
        }

        @Override
        public void fromLog(LogTable table) {
            meters = table.get("ElevatorMeters", meters);
            targetMeters = table.get("ElevatorTargetMeters", targetMeters);
            metersPerSec = table.get("ElevatorMetersPerSec", metersPerSec);
            volts = table.get("ElevatorVolts", volts);
            leftAmps = table.get("ElevatorLeftAmps", leftAmps);
            rightAmps = table.get("ElevatorRightAmps", rightAmps);
            leftTemp = table.get("ElevatorLeftTemp", leftTemp);
            rightTemp = table.get("ElevatorRightTemp", rightTemp);
            isLimitSwitchHit = table.get("ElevatorLimitSwitch Hit", isLimitSwitchHit);
            isHomed = table.get("ElevatorHomed", isHomed);
        }
    }

    /**
     * Abstract from motors, will set the elevator meters.
     * 
     * @param meters from pivot
     *               (min: {@link kElevator#ELEVATOR_MIN_METERS},
     *               max: {@link kElevator#ELEVATOR_MAX_METERS})
     * @return true if meters has been reached
     */
    public boolean setElevatorMeters(Double meters);

    public Double getElevatorMeters();

    /**
     * @return if the reverse limit switch is activated
     */
    public boolean isLimitSwitchHit();
}