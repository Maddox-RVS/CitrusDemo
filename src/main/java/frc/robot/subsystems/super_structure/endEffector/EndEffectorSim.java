package frc.robot.subsystems.super_structure.endEffector;

import org.littletonrobotics.junction.Logger;

public class EndEffectorSim implements EndEffector {
    private Double eeVolts = 0.0, eeCurentLimit = 0.0;

    private final EndEffectorInputs inputs;

    public EndEffectorSim() {
        inputs = new EndEffectorInputs();
    }

    @Override
    public Double getEndEffectorVoltage() {
        return eeVolts;
    }

    @Override
    public void runEndEffector(Double volts) {
        eeVolts = volts;
    }

    @Override
    public void setEndEffectorCurrentLimits(Double amps) {
        eeCurentLimit = amps;
    }

    @Override
    public void periodic() {
        inputs.currentLimit = eeCurentLimit;
        inputs.volts = eeVolts;

        Logger.processInputs("SuperStructure/EndEffector", inputs);
    }
}
