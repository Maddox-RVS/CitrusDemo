package frc.robot.subsystems.super_structure.wrist;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.kSuperStructure;
import frc.robot.Constants.kSuperStructure.kWrist;

public class WristReal implements Wrist {

    private final TalonFX motor;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts, motorTemp;

    private final WristInputs inputs;
    private boolean isHomed = false;
    private Double setPointDegrees = kWrist.HOME_DEGREES;

    public WristReal(Double startingDegrees) {
        motor = new TalonFX(kWrist.MOTOR_ID, kSuperStructure.CANBUS);
        motor.getConfigurator().apply(getWristMotorConfig());

        motorRots = motor.getRotorPosition();
        motorVelo = motor.getRotorVelocity();
        motorAmps = motor.getStatorCurrent();
        motorVolts = motor.getSupplyVoltage();
        motorTemp = motor.getDeviceTemp();

        motorTemp.setUpdateFrequency(4);

        motor.setPosition(mechDegreesToMotorRots(startingDegrees));
        inputs = new WristInputs(startingDegrees);
    }

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    private Double motorRotsToMechDegrees(Double motorRots) {
        return motorRots * 360.0 * kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    /**
     * Constructs a TalonFXConfiguration object only from values
     * from {@link frc.robot.Constants.kWrist}
     * 
     * @return the TalonFXConfiguration object
     */
    private TalonFXConfiguration getWristMotorConfig() {
        TalonFXConfiguration wristMotorCfg = new TalonFXConfiguration();
        wristMotorCfg.Slot0.kP = kWrist.MOTOR_kP;
        wristMotorCfg.Slot0.kI = kWrist.MOTOR_kI;
        wristMotorCfg.Slot0.kD = kWrist.MOTOR_kD;
        // wristMotorCfg.Slot0.kS = kWrist.MOTOR_kS;
        // wristMotorCfg.Slot0.kV = kWrist.MOTOR_kV;

        wristMotorCfg.MotionMagic.MotionMagicCruiseVelocity = kWrist.MAX_VELOCITY;
        wristMotorCfg.MotionMagic.MotionMagicAcceleration = kWrist.MAX_ACCELERATION;
        wristMotorCfg.MotionMagic.MotionMagicJerk = kWrist.MAX_JERK;

        wristMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristMotorCfg.MotorOutput.Inverted = kWrist.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        wristMotorCfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;

        return wristMotorCfg;
    }

    @Override
    public boolean setWristDegrees(Double degrees) {
        isHomed = false;
        setPointDegrees = degrees;
        var posControlRequest = new MotionMagicDutyCycle(mechDegreesToMotorRots(degrees));
        this.motor.setControl(posControlRequest);
        return Math.abs(degrees - getWristDegrees()) < kWrist.TOLERANCE;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        isHomed = false;
        var percentControlRequest = new DutyCycleOut(percentOut);
        this.motor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.motor.setVoltage(0.0);
    }

    @Override
    public Double getWristDegrees() {
        return inputs.degrees;
    }

    @Override
    public boolean homeMechanism(boolean force) {
        if (force) {
            isHomed = false;
        }
        if (isHomed) {
            return true;
        }

        if (getWristDegrees() < kWrist.HOME_DEGREES - 10.0) {
            setWristDegrees(kWrist.HOME_DEGREES);
        } else {
            manualDriveMechanism(0.2);
        }
        if (inputs.amps > kWrist.CURRENT_PEAK_FOR_ZERO) {
            this.stopMechanism();
            this.motor.setPosition(mechDegreesToMotorRots(kWrist.HOME_DEGREES + kWrist.HARD_OFFSET));
            isHomed = true;
        }

        return isHomed;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            motorAmps, motorVolts,
            motorRots, motorVelo,
            motorTemp
        );

        inputs.degrees = motorRotsToMechDegrees(motorRots.getValue());
        inputs.degreesPerSec = motorRotsToMechDegrees(motorVelo.getValue());
        inputs.amps = motorAmps.getValue();
        inputs.volts = motorVolts.getValue();
        inputs.temp = motorTemp.getValue();
        inputs.isHomed = isHomed;
        inputs.targetDegrees = setPointDegrees;

        Logger.processInputs("SuperStructure/Wrist", inputs);

        isHomed = inputs.isHomed;
    }
}
