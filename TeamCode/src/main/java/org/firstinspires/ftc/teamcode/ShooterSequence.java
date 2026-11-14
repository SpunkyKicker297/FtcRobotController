/*
 * ShooterSequence.java
 *
 * Runs the shooter startup / shutdown sequence WITHOUT blocking, so the driver
 * can keep driving while the flywheel spins up. Call toggle() when the shooter
 * button is pressed, and call update() once every loop.
 *
 * STARTUP (button pressed while OFF):
 *   1. Spin the flywheel up toward TARGET_RPM.
 *   2. Watch the encoder RPM. Once it is within RPM_TOLERANCE of the target
 *      (and stays there briefly), open the control gate.
 *   3. Run the secondary intake to feed pollen into the shooter.
 *
 * SHUTDOWN (button pressed while running) -- the same steps in reverse:
 *   1. Stop the secondary intake.
 *   2. Close the control gate.
 *   3. Stop the flywheel.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSequence {

    public enum State { OFF, SPINNING_UP, FEEDING }

    // ---------------------------------------------------------------------
    //  TUNE THESE
    // ---------------------------------------------------------------------
    /** Requested flywheel speed (X-rpm). Set this to the speed that shoots well. */
    public static final double TARGET_RPM = 2500.0;
    /** How close to the target counts as "up to speed". */
    public static final double RPM_TOLERANCE = 100.0;
    /** RPM must stay within tolerance this long (seconds) before the gate opens,
     *  so a momentary pass through the target doesn't trigger it. */
    public static final double STABLE_SECONDS = 0.5;

    private final RobotHardware robot;
    private final ElapsedTime stableTimer = new ElapsedTime();
    private State state = State.OFF;

    public ShooterSequence(RobotHardware robot) {
        this.robot = robot;
    }

    /** Call when the shooter button is pressed: starts or stops the sequence. */
    public void toggle() {
        if (state == State.OFF) {
            // Startup step 1: spin up the flywheel.
            robot.setShooterRpm(TARGET_RPM);
            stableTimer.reset();
            state = State.SPINNING_UP;
        } else {
            shutdown();
        }
    }

    /** Call once per loop. Advances the sequence as conditions are met. */
    public void update() {
        if (state == State.SPINNING_UP) {
            // Startup step 2: wait for the RPM to reach the request.
            boolean atSpeed = Math.abs(robot.getShooterRpm() - TARGET_RPM) <= RPM_TOLERANCE;
            if (!atSpeed) {
                stableTimer.reset();   // restart the "stable" clock
            } else if (stableTimer.seconds() >= STABLE_SECONDS) {
                robot.openGate();      // gate opens...
                robot.feedOn();        // ...startup step 3: feed pollen in
                state = State.FEEDING;
            }
        }
    }

    /** Shutdown in reverse order: feed off, gate closed, flywheel off. */
    private void shutdown() {
        robot.feedOff();
        robot.closeGate();
        robot.shooterOff();
        state = State.OFF;
    }

    /** Immediately stop everything (used when the OpMode ends). */
    public void forceStop() {
        shutdown();
    }

    public State getState() {
        return state;
    }

    public boolean isRunning() {
        return state != State.OFF;
    }
}
