package frc.robot.subsystems.super_structure.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.kSuperStructure.*;

public class WristSim implements Wrist {

    private final SingleJointedArmSim sim;
    private final PIDController pidController = new PIDController(
        kWrist.MOTOR_kP, kWrist.MOTOR_kI, kWrist.MOTOR_kD, 0.2
    );
    private Double setDegrees = kWrist.HOME_DEGREES, AppliedVolts = 0.0;
    private boolean isHomed = false;

    private final WristInputs inputs;

    public WristSim(Double startingDegrees) {
        sim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            1.0 / kWrist.MOTOR_TO_MECHANISM_RATIO,
            0.07, //TODO: get real value
            0.3,
            Units.degreesToRadians(Specs.WRIST_MIN_ANGLE),
            Units.degreesToRadians(Specs.WRIST_MAX_ANGLE),
            false,
            Units.degreesToRadians(kWrist.HOME_DEGREES)
        );
        sim.setState(Units.degreesToRadians(setDegrees), 0);
        inputs = new WristInputs(startingDegrees);
    }

    @Override
    public boolean setWristDegrees(Double degrees) {
        isHomed = false;
        setDegrees = degrees;
        Double wristVoltageFeedback = pidController.calculate(
            sim.getAngleRads(), Units.degreesToRadians(degrees));
        sim.setInputVoltage(wristVoltageFeedback);
        AppliedVolts = wristVoltageFeedback;
        return Math.abs(degrees - getWristDegrees()) < kWrist.TOLERANCE;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        isHomed = false;
        sim.setInputVoltage(12.0*percentOut);
    }

    @Override
    public void stopMechanism() {
        sim.setInputVoltage(0.0);
    }

    @Override
    public Double getWristDegrees() {
        return inputs.degrees;
    }

    @Override
    public boolean homeMechanism(boolean force) {
        isHomed = true;
        sim.setState(Units.degreesToRadians(kWrist.HOME_DEGREES), 0);
        return true;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            sim.setInputVoltage(0);
            AppliedVolts = 0.0;
        }

        sim.update(0.2);

        inputs.degrees = Units.radiansToDegrees(sim.getAngleRads());
        inputs.degreesPerSec = Units.radiansToDegrees(sim.getVelocityRadPerSec());
        inputs.volts = AppliedVolts;
        inputs.amps = sim.getCurrentDrawAmps();
        inputs.isHomed = isHomed;
        inputs.temp = 0.0;
        inputs.targetDegrees = setDegrees;

        Logger.processInputs("SuperStructure/Wrist", inputs);
    }
}
