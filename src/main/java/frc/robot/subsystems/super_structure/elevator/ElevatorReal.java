package frc.robot.subsystems.super_structure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants.kSuperStructure.Specs;
import frc.robot.Constants.kSuperStructure.kElevator;
import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ShuffleboardApi.ShuffleLayout;

public class ElevatorReal implements Elevator {
    /** Right */
    private final TalonFX leaderMotor;
    /** Left */
    private final TalonFX followerMotor;
    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts;

    private final LinearFilter ampWindow = LinearFilter.movingAverage(25);
    private Double ampWindowVal = 0.0;

    private Boolean isStowed = false;
    private Double cachedElevatorMeters;

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
        leaderMotor = new TalonFX(kElevator.ELEVATOR_RIGHT_MOTOR_ID);
        leaderMotor.getConfigurator().apply(getMotorConfiguration());

        // Left
        followerMotor = new TalonFX(kElevator.ELEVATOR_LEFT_MOTOR_ID);
        followerMotor.getConfigurator().apply(getMotorConfiguration());
        followerMotor.setControl(new Follower(kElevator.ELEVATOR_RIGHT_MOTOR_ID, true));

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        motorAmps = leaderMotor.getStatorCurrent();
        motorVolts = leaderMotor.getSupplyVoltage();

        leaderMotor.setRotorPosition(mechMetersToMotorRots(startingMeters));
        cachedElevatorMeters = startingMeters;
    }

    private TalonFXConfiguration getMotorConfiguration() {
        var motorCfg = new TalonFXConfiguration();
        motorCfg.Slot0.kP = kElevator.MOTOR_kP;
        motorCfg.Slot0.kI = kElevator.MOTOR_kI;
        motorCfg.Slot0.kD = kElevator.MOTOR_kD;
        // motorConfig.Slot0.kS = kElevator.MOTOR_kS;
        // motorConfig.Slot0.kV = kElevator.MOTOR_kV;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kElevator.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kElevator.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kElevator.MAX_JERK;

        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        motorCfg.HardwareLimitSwitch.ReverseLimitEnable = true;
        // motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        // motorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;

        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorCfg.MotorOutput.Inverted = kElevator.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return motorCfg;
    }

    @Override
    public Boolean setElevatorMeters(Double meters) {
        if (meters < Specs.ELEVATOR_MIN_METERS) {
            new SetpointTooLow(Specs.ELEVATOR_MIN_METERS, meters).log();
            return false;
        } else if (meters > Specs.ELEVATOR_MAX_METERS) {
            new SetpointTooHigh(Specs.ELEVATOR_MAX_METERS, meters).log();
            return false;
        }
        this.isStowed = false;
        var posControlRequest = new MotionMagicDutyCycle(mechMetersToMotorRots(meters));
        this.leaderMotor.setControl(posControlRequest);
        return Math.abs(meters - getElevatorMeters()) < kElevator.TOLERANCE;
    }

    @Override
    public Double getElevatorMeters() {
        return cachedElevatorMeters;
    }

    @Override
    public void manualDriveWrist(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.leaderMotor.setControl(percentControlRequest);
        this.isStowed = false;
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);
        ;
    }

    @Override
    public Boolean isLimitSwitchHit() {
        if (leaderMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround
                || leaderMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround) {
            return true;
        }
        return false;
    }

    @Override
    public Boolean homeMechanism(boolean force) {
        if (force) {
            isStowed = false;
        }
        if (this.isStowed) {
            return true;
        }
        this.manualDriveWrist(-0.2);
        if (this.isLimitSwitchHit()) {
            this.stopMechanism();
            this.leaderMotor.setRotorPosition(0.0);
            this.isStowed = true;
        }
        return this.isLimitSwitchHit();
    }

    @Override
    public Double getRecentCurrent() {
        return ampWindowVal;
    }

    @Override
    public void brake(Boolean toBrake) {
        var motorOutputCfg = new MotorOutputConfigs();
        motorOutputCfg.NeutralMode = toBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motorOutputCfg.Inverted = kElevator.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        leaderMotor.getConfigurator().apply(motorOutputCfg);
        if (toBrake) {
            leaderMotor.setControl(new StaticBrake());
        } else {
            leaderMotor.setControl(new CoastOut());
        }
    }

    @Override
    public void setupShuffleboard(ShuffleLayout tab) {
        tab.addDouble("Elevator Motor Rots", () -> motorRots.refresh().getValue());
        tab.addDouble("Elevator Motor Velo", () -> motorVelo.refresh().getValue());
        tab.addDouble("Elevator Motor Amps", () -> motorAmps.refresh().getValue());
        tab.addDouble("Elevator Motor Volts", () -> motorVolts.refresh().getValue());
        tab.addBoolean("Elevator LimitSwitch", this::isLimitSwitchHit);
    }

    @Override
    public void periodic() {
        cachedElevatorMeters = motorRotsToMechMeters(
                BaseStatusSignal.getLatencyCompensatedValue(motorRots, motorVelo));

        ampWindowVal = ampWindow.calculate(motorAmps.getValue());
    }
}
