package frc.robot.subsystems.super_structure;

import frc.robot.Constants.kSuperStructure.*;
import frc.robot.GamepieceMode;

public enum States {
    HOME(kPivot.HOME_DEGREES, kWrist.HOME_DEGREES, kElevator.HOME_METERS, EERequest.IDLE, EEBehavior.RUN_WHOLE_TIME, true),
    STOW(kPivot.HOME_DEGREES, kWrist.HOME_DEGREES-7.0, kElevator.HOME_METERS, EERequest.HOLD_TIGHT, EEBehavior.RUN_ON_START, true),
    STANDBY(kPivot.SCORE_DEGREES, kWrist.HOME_DEGREES, kElevator.HOME_METERS, EERequest.HOLD, EEBehavior.RUN_WHOLE_TIME, true),
    PLACE_HIGH(kPivot.SCORE_DEGREES, -19.0, kElevator.MAX_METERS, EERequest.OUTTAKING, EEBehavior.RUN_ON_TRANSITION, true),
    PLACE_MID(kPivot.SCORE_DEGREES, -26.0, 1.04, EERequest.OUTTAKING, EEBehavior.RUN_ON_TRANSITION, true),
    PLACE_LOW_BACK(60.0, kWrist.HOME_DEGREES-7.0, kElevator.HOME_METERS, EERequest.OUTTAKING, EEBehavior.RUN_ON_TRANSITION, true, 1.3),
    PLACE_LOW_FRONT(13.0, 15.0, kElevator.HOME_METERS, EERequest.SPIT, EEBehavior.RUN_ON_TRANSITION, true, 1.3),
    PICKUP_GROUND(kPivot.HOME_DEGREES, 13.0, kElevator.HOME_METERS, EERequest.INTAKING, EEBehavior.RUN_WHOLE_TIME, false, 1.3),
    PICKUP_STATION(63.1, -63.0, 1.08, EERequest.INTAKING, EEBehavior.RUN_ON_REACH, false),
    PICKUP_CHUTE(48.0, -10.0, kElevator.HOME_METERS, EERequest.INTAKING, EEBehavior.RUN_WHOLE_TIME, false),
    TREBUCHET(85.0, 90.0, kElevator.HOME_METERS, EERequest.OUTTAKING, EEBehavior.RUN_ON_REACH, true, 15.0);

    public final Double pivotDegrees;
    public final Double wristDegrees;
    public final Double elevatorMeters;
    public final EERequest eeRequest;
    public final EEBehavior eeBehavior;
    public final boolean useHeldGamepiece;
    public final Double toleranceMult;

    States(
            Double pivotDegrees, Double wristDegrees, Double elevatorMeters,
            EERequest eeRequest, EEBehavior eeBehavior,
            boolean useHeldGamepiece) {
        this.pivotDegrees = pivotDegrees;
        this.wristDegrees = wristDegrees;
        this.elevatorMeters = elevatorMeters;
        this.eeRequest = eeRequest;
        this.eeBehavior = eeBehavior;
        this.useHeldGamepiece = useHeldGamepiece;
        this.toleranceMult = 1.0;
    }

    States(
            Double pivotDegrees, Double wristDegrees, Double elevatorMeters,
            EERequest eeRequest, EEBehavior eeBehavior, boolean useHeldGamepiece,
            Double toleranceMult) {
        this.pivotDegrees = pivotDegrees;
        this.wristDegrees = wristDegrees;
        this.elevatorMeters = elevatorMeters;
        this.eeRequest = eeRequest;
        this.eeBehavior = eeBehavior;
        this.useHeldGamepiece = useHeldGamepiece;
        this.toleranceMult = toleranceMult;
    }

    public enum EERequest {
        IDLE(0.0, 0.0, 0.0, false),
        INTAKING(12.0, -12.0, 120.0, false),
        OUTTAKING(-12.0, 12.0, 120.0, true),
        SPIT(-8.0, 8.0, 40.0, true),
        HOLD_TIGHT(2.5, -2.5, 15.0, false),
        HOLD(1.2, -1.2, 7.5, false);

        public final double voltageCone, voltageCube, maxCurrent;
        public final boolean expelling;

        EERequest(double voltageCone, double voltageCube, double maxCurrent, boolean expelling) {
            this.voltageCone = voltageCone;
            this.voltageCube = voltageCube;
            this.maxCurrent = maxCurrent;
            this.expelling = expelling;
        }

        public double getVoltage(GamepieceMode mode) {
            if (mode == GamepieceMode.CONE) {
                return voltageCone;
            } else {
                return voltageCube;
            }
        }

        public double getCurrentLimit() {
            return this.maxCurrent;
        }
    }

    public enum EEBehavior {
        /** The entire time the state is active the end-effector will run the request */
        RUN_WHOLE_TIME,
        /** Runs at the start of the state until its setpoint is reached */
        RUN_ON_START,
        /** Runs once the setpoint is reached until a new state transition */
        RUN_ON_REACH,
        /** Run when the state is transitioning for ~0.4 seconds */
        RUN_ON_TRANSITION;
    }

    /**
     * A compact way of passing around data about the super structures current poses
     */
    public static class SuperStructurePosition {
        public final Double wristDegrees;
        public final Double pivotDegrees;
        public final Double elevatorMeters;
        public final Double endEffectorVoltage;

        public SuperStructurePosition(
                Double wristDegrees, Double pivotDegrees,
                Double elevatorMeters, Double endEffectorVoltage) {
            this.wristDegrees = wristDegrees;
            this.pivotDegrees = pivotDegrees;
            this.elevatorMeters = elevatorMeters;
            this.endEffectorVoltage = endEffectorVoltage;
        }

        /** Does not check end-effector */
        public boolean reachedState(SuperStructurePosition pose, Double toleranceMult) {
            return Math.abs(pose.pivotDegrees - pivotDegrees) < (kPivot.TOLERANCE * toleranceMult) &&
                    Math.abs(pose.wristDegrees - wristDegrees) < (kWrist.TOLERANCE * toleranceMult) &&
                    Math.abs(pose.elevatorMeters - elevatorMeters) < (kElevator.TOLERANCE * toleranceMult);
        }

        /** Does not set end-effector */
        public static SuperStructurePosition fromState(States state) {
            return new SuperStructurePosition(
                    state.wristDegrees, state.pivotDegrees,
                    state.elevatorMeters, 0.0);
        }

        public static SuperStructurePosition fromState(States state, Double endEffector) {
            return new SuperStructurePosition(
                    state.wristDegrees, state.pivotDegrees,
                    state.elevatorMeters, endEffector);
        }
    }
}
