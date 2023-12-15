package frc.robot.subsystems.super_structure.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.robot.Constants.kSuperStructure;
import frc.robot.Constants.kSuperStructure.kElevator;
import frc.robot.util.ShuffleboardApi.ShuffleEntryContainer;

public class ElevatorReal implements Elevator {
    /** Right */
    private final TalonFX leaderMotor;
    /** Left */
    private final TalonFX followerMotor;

    private final StatusSignal<Double> motorRots, motorVelo, motorVolts;
    private final StatusSignal<Double> leftMotorAmps, rightMotorAmps, leftMotorTemp, rightMotorTemp;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final ElevatorInputs inputs;

    private boolean isHomed = false;

    private Double mechMetersToMotorRots(Double meters) {
        return ((meters - kElevator.HOME_METERS)
                / (kElevator.MECHANISM_DIAMETER_METERS * Math.PI))
                / kElevator.MOTOR_TO_MECHANISM_RATIO;
    }

    private Double motorRotsToMechMeters(Double motorRots) {
        return (motorRots * kElevator.MOTOR_TO_MECHANISM_RATIO)
                * (kElevator.MECHANISM_DIAMETER_METERS * Math.PI)
                + kElevator.HOME_METERS;
    }

    public ElevatorReal(Double startingMeters) {
        // Right
        leaderMotor = new TalonFX(kElevator.ELEVATOR_RIGHT_MOTOR_ID, kSuperStructure.CANBUS);
        leaderMotor.getConfigurator().apply(getMotorConfiguration());

        // Left
        followerMotor = new TalonFX(kElevator.ELEVATOR_LEFT_MOTOR_ID, kSuperStructure.CANBUS);
        followerMotor.getConfigurator().apply(getMotorConfiguration());
        followerMotor.setControl(new Follower(kElevator.ELEVATOR_RIGHT_MOTOR_ID, true));

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        motorVolts = leaderMotor.getSupplyVoltage();
        leftMotorAmps = followerMotor.getStatorCurrent();
        rightMotorAmps = leaderMotor.getStatorCurrent();
        leftMotorTemp = followerMotor.getDeviceTemp();
        rightMotorTemp = leaderMotor.getDeviceTemp();
        rightMotorTemp.setUpdateFrequency(4);
        leftMotorTemp.setUpdateFrequency(4);
        reverseLimitSwitch = leaderMotor.getReverseLimit();

        leaderMotor.setPosition(mechMetersToMotorRots(startingMeters));
        inputs = new ElevatorInputs(startingMeters);
    }

    private TalonFXConfiguration getMotorConfiguration() {
        var motorCfg = new TalonFXConfiguration();
        motorCfg.Slot0.kP = kElevator.MOTOR_kP;
        motorCfg.Slot0.kI = kElevator.MOTOR_kI;
        motorCfg.Slot0.kD = kElevator.MOTOR_kD;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kElevator.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kElevator.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kElevator.MAX_JERK;

        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        motorCfg.HardwareLimitSwitch.ReverseLimitEnable = true;

        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorCfg.MotorOutput.Inverted = kElevator.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return motorCfg;
    }

    @Override
    public boolean setElevatorMeters(Double meters) {
        this.isHomed = false;
        var posControlRequest = new MotionMagicDutyCycle(mechMetersToMotorRots(meters));
        this.leaderMotor.setControl(posControlRequest);
        return Math.abs(meters - getElevatorMeters()) < kElevator.TOLERANCE;
    }

    @Override
    public Double getElevatorMeters() {
        return inputs.meters;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false, false, false);
        this.leaderMotor.setControl(percentControlRequest);
        this.isHomed = false;
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);
    }

    @Override
    public boolean isLimitSwitchHit() {
        return inputs.isLimitSwitchHit;
    }

    @Override
    public boolean homeMechanism(boolean force) {
        if (force) {
            isHomed = false;
        }
        if (this.isHomed) {
            return true;
        }
        this.manualDriveMechanism(-0.2);
        if (this.isLimitSwitchHit()) {
            this.stopMechanism();
            this.leaderMotor.setPosition(0.0);
            this.isHomed = true;
        }
        return this.isLimitSwitchHit();
    }

    @Override
    public void setupShuffleboard(ShuffleEntryContainer tab) {
        // tab.addDouble("Elevator Motor Rots", motorRots::getValue);
        // tab.addDouble("Elevator Motor Velo", motorVelo::getValue);
        // tab.addDouble("Elevator Motor Amps", motorAmps::getValue);
        // tab.addDouble("Elevator Motor Volts", motorVolts::getValue);
        // tab.addBoolean("Elevator LimitSwitch", this::isLimitSwitchHit);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            motorRots, motorVelo,
            motorVolts, reverseLimitSwitch,
            leftMotorAmps, rightMotorAmps,
            leftMotorTemp, rightMotorTemp
        );

        inputs.meters = motorRotsToMechMeters(motorRots.getValue());
        inputs.metersPerSec = motorRotsToMechMeters(motorVelo.getValue());
        inputs.isLimitSwitchHit = reverseLimitSwitch.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.volts = motorVolts.getValue();
        inputs.leftAmps = leftMotorAmps.getValue();
        inputs.rightAmps = rightMotorAmps.getValue();
        inputs.leftTemp = leftMotorTemp.getValue();
        inputs.rightTemp = rightMotorTemp.getValue();
        inputs.isHomed = isHomed;

        Logger.processInputs("SuperStructure/Elevator", inputs);

        isHomed = inputs.isHomed;
    }
}
