package frc.robot.subsystems.super_structure.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.kSuperStructure.*;
import frc.robot.subsystems.super_structure.Errors.*;
import frc.robot.util.ErrorHelper.*;

public class PivotReal implements Pivot {

    /** Left */
    private final TalonFX leaderMotor;
    /** Right */
    private final TalonFX followerMotor;

    private final StatusSignal<Double> motorRots, motorVelo;

    public PivotReal() {
        leaderMotor = new TalonFX(kPivot.LEFT_MOTOR_ID);
        followerMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID);
        leaderMotor.getConfigurator().apply(getMotorConfig());
        followerMotor.getConfigurator().apply(getMotorConfig());

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();

        followerMotor.setControl(
            new Follower(kPivot.LEFT_MOTOR_ID, true)
        );
    }

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kWrist.MOTOR_TO_MECHANISM_RATIO;
    }

    private TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Slot0.kP = kPivot.MOTOR_kP;
        motorCfg.Slot0.kI = kPivot.MOTOR_kI;
        motorCfg.Slot0.kD = kPivot.MOTOR_kD;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = kPivot.ENABLE_SOFTLIMITS;
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = kPivot.ENABLE_SOFTLIMITS;
        motorCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mechDegreesToMotorRots(
                kPivot.MAX_DEGREES);
        motorCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mechDegreesToMotorRots(
                kPivot.MIN_DEGREES);

        motorCfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        motorCfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        motorCfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        return motorCfg;
    }

    @Override
    public Result<Ok, GroupError<SuperStructureErrors>> setMechanismDegrees(Double degrees) {
        if (degrees > kPivot.MAX_DEGREES) {
            return Result.err(new SetpointTooHigh(kPivot.MAX_DEGREES, degrees));
        } else if (degrees < kPivot.MIN_DEGREES) {
            return Result.err(new SetpointTooLow(kPivot.MIN_DEGREES, degrees));
        }
        var posControlRequest = new MotionMagicTorqueCurrentFOC(mechDegreesToMotorRots(degrees));
        this.leaderMotor.setControl(posControlRequest);
        return Result.ok(new Ok());
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        var percentControlRequest = new DutyCycleOut(percentOut, true, false);
        this.leaderMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);
    }

    @Override
    public Double getMechanismDegrees() {
        BaseStatusSignal.waitForAll(0, motorRots, motorVelo);
        return BaseStatusSignal.getLatencyCompensatedValue(motorRots, motorVelo)
                * 360.0
                * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    @Override
    public void zeroMechanism() {

    }

    @Override
    public void playErrorTone() {}

    @Override
    public void periodic() {}
}