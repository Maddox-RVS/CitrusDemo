package frc.robot.subsystems.super_structure.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.kSuperStructure.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.subsystems.super_structure.Errors.*;

public class ElevatorReal implements Elevator {
    /**Right */
    private final TalonFX leaderMotor;
    /**Left */
    private final TalonFX followerMotor;
    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts;

    private Boolean isHomed = false;

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
    }

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

    private TalonFXConfiguration getMotorConfiguration() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = kElevator.MOTOR_kP;
        motorConfig.Slot0.kP = kElevator.MOTOR_kI;
        motorConfig.Slot0.kP = kElevator.MOTOR_kD;
        // motorConfig.Slot0.kS = kElevator.MOTOR_kS;
        // motorConfig.Slot0.kV = kElevator.MOTOR_kV;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = kElevator.MAX_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = kElevator.MAX_ACCELERATION;
        motorConfig.MotionMagic.MotionMagicJerk = kElevator.MAX_JERK;

        // motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = kElevator.ENABLE_SOFTLIMITS;
        // motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mechMetersToMotorRots(Specs.ELEVATOR_MAX_METERS);

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.2;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.2;

        motorConfig.MotorOutput.Inverted = kElevator.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return motorConfig;
    }

    @Override
    public Boolean setMechanismMeters(Double meters) {
        if (meters < Specs.ELEVATOR_MIN_METERS) {
            new SetpointTooLow(Specs.ELEVATOR_MIN_METERS, meters).log();
            return false;
        } else if (meters > Specs.ELEVATOR_MAX_METERS) {
            new SetpointTooHigh(Specs.ELEVATOR_MAX_METERS, meters).log();
            return false;
        }
        this.isHomed = true;
        var posControlRequest = new PositionVoltage(mechMetersToMotorRots(meters));
        this.leaderMotor.setControl(posControlRequest);
        return Math.abs(meters - getMechanismMeters()) < kPivot.TOLERANCE;
    }

    @Override
    public Double getMechanismMeters() {
        return motorRotsToMechMeters(motorRots.refresh().getValue());
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.leaderMotor.setControl(percentControlRequest);
        this.isHomed = false;
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);;
    }

    @Override
    public Boolean isLimitSwitchHit() {
        if (leaderMotor.getReverseLimit().refresh().getValue() == ReverseLimitValue.ClosedToGround) {
            return true;
        }
        return false;
    }

    @Override
    public Boolean homeMechanism() {
        if (this.isHomed) {
            return true;
        }
        this.manualDriveMechanism(-0.15);
        if (this.isLimitSwitchHit()) {
            this.stopMechanism();
            this.leaderMotor.setRotorPosition(0.0);
            this.isHomed = true;
        }
        return this.isLimitSwitchHit();
    }

    @Override
    public void periodic() {}

    @Override
    public void setupShuffleboard(ShuffleboardContainer tab) {
        tab.addNumber("Elevator Motor Rots", () -> motorRots.refresh().getValue());
        tab.addNumber("Elevator Motor Velo", () -> motorVelo.refresh().getValue());
        tab.addNumber("Elevator Motor Amps", () -> motorAmps.refresh().getValue());
        tab.addNumber("Elevator Motor Volts", () -> motorVolts.refresh().getValue());
        tab.addBoolean("Elevator LimitSwitch", this::isLimitSwitchHit);
    }
}
