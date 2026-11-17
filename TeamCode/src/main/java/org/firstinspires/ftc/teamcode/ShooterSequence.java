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
 *   3. Start the PRIMARY intake with the gate, then after GATE_SETTLE_SECONDS
 *      (0.5 s) start the SECONDARY intake, so pollen isn't forced into the gate
 *      before it's fully open.
 *
 * SHUTDOWN (button pressed while running) -- the same steps in reverse:
 *   1. Stop both intakes.
 *   2. Close the control gate.
 *   3. Stop the flywheel.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSequence {

    public enum State { OFF, SPINNING_UP, OPENING, FEEDING }

    // ---------------------------------------------------------------------
    //  TUNE THESE
    // ---------------------------------------------------------------------
    /** Requested flywheel speed (X-rpm). Set this to the speed that shoots well. */
    public static final double TARGET_RPM = 2600.0;
    /** How close to the target counts as "up to speed". */
    public static final double RPM_TOLERANCE = 100.0;
    /** RPM must stay within tolerance this long (seconds) before the gate opens,
     *  so a momentary pass through the target doesn't trigger it. */
    public static final double STABLE_SECONDS = .5;
    /** After the gate opens, wait this long (seconds) before the SECONDARY intake
     *  starts, so pollen isn't pushed into a gate that's still moving.
     *  Shared with TeleOp (ContinuousShooter). */
    public static final double GATE_SETTLE_SECONDS = 0.5;

    private final RobotHardware robot;
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
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
                openGateAndStartPrimary();   // startup step 3a
            }
        } else if (state == State.OPENING
                && gateTimer.seconds() >= GATE_SETTLE_SECONDS) {
            robot.feedOn();                  // startup step 3b: gate is open, feed in
            state = State.FEEDING;
        }
    }

    /** Gate up + primary intake on now; the secondary waits for GATE_SETTLE_SECONDS. */
    private void openGateAndStartPrimary() {
        robot.openGate();
        robot.intakeIn();
        gateTimer.reset();
        state = State.OPENING;
    }

    /** True once the gate has been told to open (OPENING or FEEDING). */
    public boolean isGateOpen() {
        return state == State.OPENING || state == State.FEEDING;
    }

    /**
     * Open the gate and start feeding now, even if the RPM check hasn't passed.
     * Used by autonomous so a slow spin-up doesn't skip the shot entirely.
     */
    public void forceFeed() {
        if (state == State.OFF) {
            robot.setShooterRpm(TARGET_RPM);
        }
        if (!isGateOpen()) {
            openGateAndStartPrimary();   // update() starts the secondary after the delay
        }
    }

    /** Shutdown in reverse order: feed off, gate closed, flywheel off. */
    private void shutdown() {
        robot.shootFeedOff();
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
