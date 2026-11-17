/*
 * ContinuousShooter.java
 *
 * TeleOp shooter control where the FLYWHEEL RUNS THE WHOLE TIME and the button
 * only controls the gate and the intakes (primary AND secondary run together
 * while firing, so pollen stuck in the primary gets pushed through). Because the flywheel is
 * already at speed, shots start almost immediately after the button press.
 *
 * Call start() once after PLAY, toggleFire() when the fire button is pressed,
 * update() once every loop, and stop() when the OpMode ends.
 *
 * FIRE (button pressed while the gate is closed):
 *   - If the flywheel is at speed: open the gate and start the primary intake
 *     now; the secondary intake starts GATE_SETTLE_SECONDS (0.5 s) later.
 *   - If not (still spinning up or recovering): the shot is ARMED and fires
 *     automatically the moment the flywheel reaches speed.
 *
 * CEASE FIRE (button pressed while firing or armed):
 *   - Feed intake off, gate closed. The flywheel KEEPS spinning.
 *
 * While firing, the RPM dips each time pollen passes through. That's expected,
 * so feeding continues; the speed check only gates the START of a shot.
 *
 * Speed settings (TARGET_RPM, RPM_TOLERANCE, STABLE_SECONDS) are shared with the
 * autonomous and live in ShooterSequence.java.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ContinuousShooter {

    public enum State { OFF, SPINNING, ARMED, OPENING, FEEDING }

    private final RobotHardware robot;
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private State state = State.OFF;

    public ContinuousShooter(RobotHardware robot) {
        this.robot = robot;
    }

    /** Start the flywheel spinning. Call once, right after PLAY. */
    public void start() {
        robot.closeGate();
        robot.shootFeedOff();
        robot.setShooterRpm(ShooterSequence.TARGET_RPM);
        stableTimer.reset();
        state = State.SPINNING;
    }

    /** Call when the fire button is pressed. */
    public void toggleFire() {
        switch (state) {
            case SPINNING:
                if (isAtSpeed()) {
                    fire();
                } else {
                    state = State.ARMED;   // fire as soon as we're at speed
                }
                break;
            case ARMED:
            case OPENING:
            case FEEDING:
                ceaseFire();
                break;
            case OFF:
                break;                     // start() hasn't been called
        }
    }

    /** Call once per loop. Tracks flywheel speed and fires an armed shot. */
    public void update() {
        if (state == State.OFF) return;

        // Keep the "stable at speed" clock running continuously.
        if (Math.abs(robot.getShooterRpm() - ShooterSequence.TARGET_RPM)
                > ShooterSequence.RPM_TOLERANCE) {
            stableTimer.reset();
        }

        if (state == State.ARMED && isAtSpeed()) {
            fire();
        } else if (state == State.OPENING
                && gateTimer.seconds() >= ShooterSequence.GATE_SETTLE_SECONDS) {
            robot.feedOn();                // gate is open: start the secondary
            state = State.FEEDING;
        }
    }

    /** True while a shot is in progress (gate up), so the primary intake runs. */
    public boolean isFiring() {
        return state == State.OPENING || state == State.FEEDING;
    }

    /** True once the RPM has stayed within tolerance for STABLE_SECONDS. */
    public boolean isAtSpeed() {
        return state != State.OFF
                && stableTimer.seconds() >= ShooterSequence.STABLE_SECONDS;
    }

    /** Stop everything: feed off, gate closed, flywheel off. */
    public void stop() {
        robot.shootFeedOff();
        robot.closeGate();
        robot.shooterOff();
        state = State.OFF;
    }

    public State getState() {
        return state;
    }

    /** Gate up + primary intake now; update() starts the secondary after the delay. */
    private void fire() {
        robot.openGate();
        robot.intakeIn();
        gateTimer.reset();
        state = State.OPENING;
    }

    private void ceaseFire() {
        robot.shootFeedOff();
        robot.closeGate();
        state = State.SPINNING;            // flywheel keeps running
    }
}
