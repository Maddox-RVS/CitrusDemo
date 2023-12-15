package frc.robot.subsystems.super_structure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.ShuffleboardApi;
import frc.robot.subsystems.super_structure.States.SuperStructurePosition;
import frc.robot.subsystems.super_structure.pivot.*;
import frc.robot.subsystems.super_structure.wrist.*;
import frc.robot.subsystems.super_structure.endEffector.*;
import frc.robot.subsystems.super_structure.elevator.*;

public class SuperStructure extends SubsystemBase {
    private final Wrist wrist;
    private final Elevator elevator;
    private final Pivot pivot;
    private final EndEffector endEffector;

    private final Visualizer visualizer = new Visualizer();

    private SuperStructurePosition setpoint = SuperStructurePosition.fromState(States.HOME);

    private boolean isHomed = false;

    public SuperStructure() {
        if (Robot.isReal()) {
            this.wrist = new WristReal(setpoint.wristDegrees);
            this.pivot = new PivotReal(/* uses pigeon */);
            this.elevator = new ElevatorReal(setpoint.elevatorMeters);
            this.endEffector = new EndEffectorReal();
        } else {
            this.wrist = new WristSim(setpoint.wristDegrees);
            this.pivot = new PivotSim(setpoint.pivotDegrees);
            this.elevator = new ElevatorSim(setpoint.elevatorMeters);
            this.endEffector = new EndEffectorSim();
        }
        setupShuffleboard();
        visualizer.updateSetpoint(setpoint);
        visualizer.updateCurrent(setpoint);
    }

    public static enum SuperStructureMoveOrder {
        SELF_RESOLVE,
        PIVOT_FIRST,
        PIVOT_LAST,
        ALL_AT_ONCE
    }

    /** @returns true of the setpoint has been reached */
    public boolean setSetpoint(SuperStructurePosition to, SuperStructureMoveOrder order) {
        if (!checkSuperstructureEnabled()) return false;

        this.visualizer.updateSetpoint(to);
        this.setpoint = to;
        this.isHomed = false;

        // only pivot or wrist+elevator should run at a time
        BooleanSupplier runWristElevatorParallel = () -> {
            var wrist = this.wrist.setWristDegrees(to.wristDegrees);
            var elev = this.elevator.setElevatorMeters(to.elevatorMeters);
            return elev && wrist;
        };
        BooleanSupplier runPivot = () -> {
            return this.pivot.setPivotDegrees(to.pivotDegrees);
        };

        if (order == SuperStructureMoveOrder.SELF_RESOLVE) {
            // We need to make sure the pivot and elevator do not move at the same time.
            // We need to also try and run the pivot when the elevator is in the least
            // extended between the two states(pose and current).

            // i.e. if the elevator is currently not extended but the pose wants it all the
            // way out,
            // we will move the pivot first to reduce stress, but if the current elevator
            // extension
            // is greater than set pose we move elevator first

            if (this.elevator.getElevatorMeters() > to.elevatorMeters) {
                // check if wrist and elevator have reached their setpoints
                // if they have, run pivot
                if (runWristElevatorParallel.getAsBoolean()) {
                    return runPivot.getAsBoolean();
                }
            } else {
                if (runPivot.getAsBoolean()) {
                    return runWristElevatorParallel.getAsBoolean();
                }
            }
            return false;
        } else if (order == SuperStructureMoveOrder.PIVOT_FIRST) {
            return runPivot.getAsBoolean() && runWristElevatorParallel.getAsBoolean();
        } else if (order == SuperStructureMoveOrder.PIVOT_LAST) {
            return runWristElevatorParallel.getAsBoolean() && runPivot.getAsBoolean();
        } else if (order == SuperStructureMoveOrder.ALL_AT_ONCE) {
            var eagerEvalPivot = runPivot.getAsBoolean();
            var eagerEvalWristElevator = runWristElevatorParallel.getAsBoolean();
            return eagerEvalWristElevator && eagerEvalPivot;
        }
        return false;
    }

    /** @returns true of the setpoint has been reached */
    public boolean setSetpoint(SuperStructurePosition to) {
        return setSetpoint(to, SuperStructureMoveOrder.SELF_RESOLVE);
    }

    /**
     * Will move the mechanism to the home position,
     * they will follow the order of wrist -> elevator -> pivot
     * 
     * @param force if true, will ignore if the mechanism is already at the home
     *              position
     * @return true if all mechanisms have reached their home position
     */
    public boolean home(boolean force) {
        this.setpoint = SuperStructurePosition.fromState(States.HOME);
        this.visualizer.updateSetpoint(this.setpoint);
        // this will do wrist -> elevator -> pivot
        if (this.wrist.homeMechanism(force)
                && this.elevator.homeMechanism(force)
                && this.pivot.homeMechanism(force)) {
            this.isHomed = true;
            return true;
        }
        return false;
    }

    /**
     * @return true if the last {@link SuperStructure#home(boolean)} returned true
     *         and
     *         a {@link SuperStructure#setSetpoint} hasn't been called since.
     */
    public boolean isHomed() {
        return this.isHomed;
    }

    public void runEndEffector(Double volts, Double currentLimit) {
        if (!checkSuperstructureEnabled()) return;

        // when outtaking this should be false
        this.endEffector.setEndEffectorCurrentLimits(currentLimit);
        // assume max voltage is 12.0
        this.endEffector.runEndEffector(volts / 12.0);
    }

    public boolean checkSuperstructureEnabled() {
        return NetworkTableInstance.getDefault()
        .getEntry("/Shuffleboard/Driver/Enable Superstructure")
        .getBoolean(false);
    }

    public void stopAll() {
        this.wrist.stopMechanism();
        this.pivot.stopMechanism();
        this.elevator.stopMechanism();
        this.endEffector.runEndEffector(0.0);
    }

    public boolean reachedSetpoint(Double toleranceMult) {
        return this.setpoint.reachedState(this.getPose(), toleranceMult);
    }

    /**
     * @return the current position on the superstructure
     */
    public SuperStructurePosition getPose() {
        return new SuperStructurePosition(
                this.wrist.getWristDegrees(),
                this.pivot.getPivotDegrees(),
                this.elevator.getElevatorMeters(),
                this.endEffector.getEndEffectorVoltage());
    }

    /**
     * To be used for debugging, not guranteed to have all
     * safety features
     * 
     * @param wristPercent       of the wrist mechanisms motors
     * @param pivotPercent       of the pivot mechanisms motors
     * @param elevatorPercent    of the elevator mechanisms motors
     * @param endEffectorPercent of the end-effector mechanisms motors
     */
    public void manualControl(Double wristPercent, Double pivotPercent, Double elevatorPercent, Double endEffectorPercent) {
        if (!checkSuperstructureEnabled()) return;
        
        this.wrist.manualDriveMechanism(wristPercent);
        this.pivot.manualDriveMechanism(pivotPercent);
        this.elevator.manualDriveMechanism(elevatorPercent);
        this.endEffector.manualDriveMechanism(endEffectorPercent);
    }

    /** once moved over to TEMPLATE this can be removed */
    private void setupShuffleboard() {
        var tab = ShuffleboardApi.getTab("SuperStructure");
        tab.addDouble("Wrist Setpoint Degrees", () -> this.setpoint.wristDegrees)
                .withSize(2, 1);
        tab.addDouble("Pivot Setpoint Degrees", () -> this.setpoint.pivotDegrees)
                .withSize(2, 1);
        tab.addDouble("Elevator Setpoint Meters", () -> this.setpoint.elevatorMeters)
                .withSize(2, 1);

        tab.addDouble("Wrist Current Degrees", () -> this.wrist.getWristDegrees())
                .withSize(2, 1);
        tab.addDouble("Pivot Current Degrees", () -> this.pivot.getPivotDegrees())
                .withSize(2, 1);
        tab.addDouble("Elevator Current Meters", () -> this.elevator.getElevatorMeters())
                .withSize(2, 1);

        tab.addString("Current Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "None")
                .withSize(2, 1);

        wrist.setupShuffleboard(tab.getLayout("Wrist"));
        pivot.setupShuffleboard(tab.getLayout("Pivot"));
        elevator.setupShuffleboard(tab.getLayout("Elevator"));
        endEffector.setupShuffleboard(tab.getLayout("End Effector"));

        visualizer.setShuffleboardTab(tab);
    }

    @Override
    public void periodic() {
        this.wrist.periodic();
        this.elevator.periodic();
        this.pivot.periodic();
        this.endEffector.periodic();

        visualizer.updateCurrent(getPose());

        if (DriverStation.isDisabled() && this.getCurrentCommand() != null) {
            this.getCurrentCommand().cancel();
        }
    }
}
