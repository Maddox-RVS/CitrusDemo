package frc.robot.subsystems.super_structure.endEffector;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.kSuperStructure;
import frc.robot.Constants.kSuperStructure.kEndEffector;
import frc.robot.util.ShuffleboardApi.ShuffleEntryContainer;

public class EndEffectorReal implements EndEffector {

    private final TalonFX endEffectorMotor;

    private final StatusSignal<Double> motorAmps, motorVolts, motorTemp;

    private final EndEffectorInputs inputs;

    public EndEffectorReal() {
        endEffectorMotor = new TalonFX(kEndEffector.MOTOR_ID, kSuperStructure.CANBUS);
        endEffectorMotor.getConfigurator().apply(getEndEffectorMotorConfig());

        motorAmps = endEffectorMotor.getStatorCurrent();
        motorVolts = endEffectorMotor.getSupplyVoltage();
        motorTemp = endEffectorMotor.getDeviceTemp();
        motorTemp.setUpdateFrequency(4);

        inputs = new EndEffectorInputs();
    }

    /**
     * Constructs a TalonFXConfiguration object only from values
     * from {@link frc.robot.Constants.kEndEffector}
     * 
     * @return the TalonFXConfiguration object
     */
    private TalonFXConfiguration getEndEffectorMotorConfig() {
        TalonFXConfiguration eeMotorCfg = new TalonFXConfiguration();

        eeMotorCfg.MotorOutput.Inverted = kEndEffector.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return eeMotorCfg;
    }

    @Override
    public void runEndEffector(Double volts) {
        var voltageControlRequest = new VoltageOut(volts);
        this.endEffectorMotor.setControl(voltageControlRequest);
    }

    @Override
    public Double getEndEffectorVoltage() {
        return inputs.volts;
    }

    @Override
    public void setEndEffectorCurrentLimits(Double limit) {
        if (limit != inputs.currentLimit) {
            var cfg = new CurrentLimitsConfigs();
            cfg.SupplyCurrentLimitEnable = true;
            cfg.SupplyCurrentLimit = limit;
            cfg.SupplyCurrentThreshold = limit;
            cfg.SupplyTimeThreshold = 0.2;
            cfg.StatorCurrentLimit = limit;
            cfg.StatorCurrentLimitEnable = true;
            endEffectorMotor.getConfigurator().apply(cfg);
            inputs.currentLimit = limit;
        }
    }

    @Override
    public void setupShuffleboard(ShuffleEntryContainer tab) {}

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorAmps, motorVolts);

        inputs.amps = motorAmps.getValue();
        inputs.volts = motorVolts.getValue();
        inputs.temp = motorTemp.getValue();

        Logger.processInputs("/SuperStructure/EndEffector", inputs);
    }
}
