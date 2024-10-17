package org.firstinspires.ftc.teamcode;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;

@SuppressWarnings({"unused"})
public class StateMachines {
    public enum SpecimenPickup {
        IDLE,
        EXTEND,
        REBOUND,
        LIFT,
        REVERSE,
        DONE
    }
    public static StateMachine getSpecimenPickupMachine(hwMecanumFtclib robot) {
        return new StateMachineBuilder()
                /* Make sure we stopped driving */
                .state(SpecimenPickup.IDLE)
                //.onEnter()
                .transitionTimed(0.5)
                /* Extend the elevator to the grab position */
                .state(SpecimenPickup.EXTEND)
                //.onEnter()
                .transitionTimed(0.5)
                /* Done with pickup operation */
                .state(SpecimenPickup.DONE)
                //.onEnter()
                .build();
    }

    public enum SamplePickup {
        IDLE,
        LEVEL,
        INTAKE,
        EXTEND,
        LIFT,
        DONE
    }
    public static StateMachine getSamplePickupMachine(hwMecanumFtclib robot) {
        return new StateMachineBuilder()
                /* Make sure we stopped driving */
                .state(SamplePickup.IDLE)
                //.onEnter()
                .transitionTimed(0.5)
                /* Move the manipulator to the intake position */
                .state(SamplePickup.LEVEL)
                //.onEnter()
                .transitionTimed(0.5)
                /* Extend the elevator */
                .state(SamplePickup.EXTEND)
                //.onEnter()
                .transitionTimed(0.5)
                /* Done with pickup operation */
                .state(SamplePickup.DONE)
                //.onEnter()
                .build();
    }

    public enum Climb {
        IDLE,
        RAISE,
        LIFT,
        DONE
    }
    public static StateMachine getClimbMachine(hwMecanumFtclib robot) {
        return new StateMachineBuilder()
            .state(Climb.IDLE)
            .build();
    }
}
