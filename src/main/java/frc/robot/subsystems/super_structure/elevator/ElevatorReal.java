package frc.robot.subsystems.super_structure.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.kSuperStructure.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ErrorHelper.*;

public class ElevatorReal implements Elevator{
    private final TalonFX leaderMotor, followerMotor;
    private final StatusSignal<Double> motorRots, motorVelo;

    public ElevatorReal() {
        //Left
        leaderMotor = new TalonFX(kElevator.ELEVATOR_LEFT_MOTOR_ID);
        leaderMotor.getConfigurator().apply(getMotorConfiguration());

        //Right
        followerMotor = new TalonFX(kElevator.ELEVATOR_RIGHT_MOTOR_ID);
        followerMotor.getConfigurator().apply(getMotorConfiguration());
        followerMotor.setControl(new Follower(kElevator.ELEVATOR_RIGHT_MOTOR_ID, true));

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
    }

    private Double mechMetersToMotorRots(Double meters) {
        return ((180.0 * meters) / (kElevator.MECHANISM_RADIUS_METERS * Math.PI)) / 360.0;
    }
    private Double motorRotsToMechMeters(Double motorRots) {
        return (kElevator.MECHANISM_RADIUS_METERS * Math.PI * (motorRots * 360.0)) / 180.0;
    }

    private TalonFXConfiguration getMotorConfiguration() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = kElevator.MOTOR_kP;
        motorConfig.Slot0.kP = kElevator.MOTOR_kI;
        motorConfig.Slot0.kP = kElevator.MOTOR_kD;
        motorConfig.Slot0.kS = kElevator.MOTOR_kS;
        motorConfig.Slot0.kV = kElevator.MOTOR_kV;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = kElevator.MAX_VELOCITY;
        motorConfig.MotionMagic.MotionMagicAcceleration = kElevator.MAX_ACCELERATION;
        motorConfig.MotionMagic.MotionMagicJerk = kElevator.MAX_JERK;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = kElevator.ENABLE_SOFTLIMITS;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = kElevator.ENABLE_SOFTLIMITS;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mechMetersToMotorRots(Specs.ELEVATOR_MAX_METERS);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mechMetersToMotorRots(Specs.ELEVATOR_MIN_METERS);

        motorConfig.MotorOutput.Inverted = kElevator.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        return motorConfig;
    }

    @Override
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismMeters(Double meters) {
        if (meters < Specs.ELEVATOR_MIN_METERS) {
            return Result.err(new SetpointTooLow(Specs.ELEVATOR_MIN_METERS, meters));
        }
        else if (meters > Specs.ELEVATOR_MAX_METERS) {
            return Result.err(new SetpointTooHigh(Specs.ELEVATOR_MAX_METERS, meters));
        }

        var posControlRequest = new MotionMagicTorqueCurrentFOC(mechMetersToMotorRots(meters));
        this.leaderMotor.setControl(posControlRequest);
        return Result.ok(new Ok());
    }

    @Override
    public Double getMechanismMeters() {
        BaseStatusSignal.waitForAll(0, motorRots, motorVelo);
        return motorRotsToMechMeters(BaseStatusSignal.getLatencyCompensatedValue(motorRots, motorVelo));
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.leaderMotor.setControl(percentControlRequest);
    }

    @Override
    public void periodic() {

    }
}
