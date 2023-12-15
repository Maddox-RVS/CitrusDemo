package frc.robot.subsystems.super_structure.pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.kSuperStructure;
import frc.robot.Constants.kSuperStructure.kPivot;
import frc.robot.util.ShuffleboardApi.ShuffleEntryContainer;

public class PivotReal implements Pivot {

    /** Left */
    private final TalonFX leaderMotor;
    /** Right */
    private final TalonFX followerMotor;

    private final Pigeon2 gyro;

    private final StatusSignal<Double> motorRots, motorVelo, motorVolts;
    private final StatusSignal<Double> leftMotorAmps, rightMotorAmps, leftMotorTemp, rightMotorTemp;
    private final StatusSignal<Double> gyroPitch;

    private final PivotInputs inputs;

    private boolean isHomed = false;

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private Double motorRotsToMechDegrees(Double motorRots) {
        return motorRots * 360.0 * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    public PivotReal() {
        gyro = new Pigeon2(kPivot.PIGEON_ID, kSuperStructure.CANBUS);
        gyroPitch = gyro.getPitch();

        leaderMotor = new TalonFX(kPivot.LEFT_MOTOR_ID, kSuperStructure.CANBUS);
        followerMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID, kSuperStructure.CANBUS);
        leaderMotor.getConfigurator().apply(getMotorConfig());
        followerMotor.getConfigurator().apply(getMotorConfig());

        followerMotor.setControl(
                new Follower(kPivot.LEFT_MOTOR_ID, true));

        inputs = new PivotInputs(gyroPitch.getValue() - kPivot.PIGEON_OFFSET);
        leaderMotor.setPosition(mechDegreesToMotorRots(getPivotDegreesPigeon()));

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        rightMotorAmps = leaderMotor.getStatorCurrent();
        leftMotorAmps = followerMotor.getStatorCurrent();
        motorVolts = leaderMotor.getSupplyVoltage();

        rightMotorTemp = leaderMotor.getDeviceTemp();
        leftMotorTemp = followerMotor.getDeviceTemp();
        rightMotorTemp.setUpdateFrequency(4);
        leftMotorTemp.setUpdateFrequency(4);
    }

    private TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Slot0.kP = kPivot.MOTOR_kP;
        motorCfg.Slot0.kI = kPivot.MOTOR_kI;
        motorCfg.Slot0.kD = kPivot.MOTOR_kD;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorCfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        motorCfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        motorCfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        return motorCfg;
    }

    @Override
    public boolean setPivotDegrees(Double degrees) {
        isHomed = false;
        var posControlRequest = new MotionMagicDutyCycle(mechDegreesToMotorRots(degrees));
        this.leaderMotor.setControl(posControlRequest);
        return Math.abs(degrees - getPivotDegrees()) < kPivot.TOLERANCE;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        isHomed = false;
        var percentControlRequest = new DutyCycleOut(percentOut);
        this.leaderMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);
    }

    @Override
    public Double getPivotDegrees() {
        return inputs.degrees;
    }

    private Double getPivotDegreesPigeon() {
        return inputs.gyroPitchDegrees - kPivot.PIGEON_OFFSET;
    }

    private void seedPivot() {
        var pigeonDegrees = mechDegreesToMotorRots(getPivotDegreesPigeon());
        leaderMotor.setPosition(pigeonDegrees);
    }

    @Override
    public boolean homeMechanism(boolean force) {
        if (force) {
            isHomed = false;
        }
        if (isHomed) {
            this.stopMechanism();
            return true;
        }

        var reached = this.setPivotDegrees(kPivot.HOME_DEGREES);
        if (reached) {
            this.stopMechanism();
            seedPivot();
            isHomed = true;
            return isHomed;
        }
        if (
            inputs.leftAmps > kPivot.CURRENT_PEAK_FOR_HOME
            || inputs.rightAmps > kPivot.CURRENT_PEAK_FOR_HOME
        ) {
            this.stopMechanism();
            seedPivot();
            isHomed = true;
        }

        return isHomed;
    }

    @Override
    public void setupShuffleboard(ShuffleEntryContainer tab) {
        // tab.addDouble("Pivot Motor Rots", motorRots::getValue);
        // tab.addDouble("Pivot Motor Velo", motorVelo::getValue);
        // tab.addDouble("Pivot Motor Amps", motorAmps::getValue);
        // tab.addDouble("Pivot Motor Volts", motorVolts::getValue);
        // tab.addDouble("Pivot Degrees Gyro", this::getPivotDegreesPigeon);
        // tab.addBoolean("Pivot Homed", () -> isHomed);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            motorRots, motorVelo,
            motorVolts, gyroPitch,
            leftMotorAmps, rightMotorAmps,
            leftMotorTemp, rightMotorTemp
        );

        inputs.degrees = motorRotsToMechDegrees(motorRots.getValue());
        inputs.degreesPerSec = motorRotsToMechDegrees(motorVelo.getValue());
        inputs.gyroPitchDegrees = gyroPitch.getValue();
        inputs.volts = motorVolts.getValue();
        inputs.leftAmps = leftMotorAmps.getValue();
        inputs.rightAmps = rightMotorAmps.getValue();
        inputs.leftTemp = leftMotorTemp.getValue();
        inputs.rightTemp = rightMotorTemp.getValue();
        inputs.isHomed = isHomed;

        Logger.processInputs("SuperStructure/Pivot", inputs);

        isHomed = inputs.isHomed;
    }
}
