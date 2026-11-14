/*
 * GateServoTuner.java
 *
 * A dedicated test OpMode for finding the ideal OPEN and CLOSED positions of the
 * control gate servo (control_gate_1). Nothing else moves -- no motors run -- so
 * you can safely watch the gate while you adjust it.
 *
 * -----------------------------------------------------------------------------
 * CONTROLS (gamepad1)
 * -----------------------------------------------------------------------------
 *   D-pad up (press) ......... move the gate UP one small step
 *   D-pad down (press) ....... move the gate DOWN one small step
 *   Hold right bumper ........ use BIG steps instead (for moving quickly)
 *   A (press) ................ save the current position as "OPEN"
 *   B (press) ................ save the current position as "CLOSED"
 *   X (press) ................ jump to the saved OPEN position
 *   Y (press) ................ jump to the saved CLOSED position
 *
 * -----------------------------------------------------------------------------
 * HOW TO USE IT
 * -----------------------------------------------------------------------------
 *   1. Press PLAY. The gate starts at the CLOSED value from RobotHardware.java.
 *   2. Use D-pad up/down until the gate is where "open" should be. Press A.
 *   3. Use D-pad up/down until the gate is where "closed" should be. Press B.
 *   4. Press X and Y a few times to check both positions look right.
 *   5. Copy the two numbers shown on the Driver Station into RobotHardware.java:
 *        GATE_OPEN_POSITION   = <OPEN value>
 *        GATE_CLOSED_POSITION = <CLOSED value>
 *      (Replace the GATE_CLOSED_POSITION formula with the plain number.)
 *
 * Saved values are NOT stored on the robot -- write them down before you stop.
 *
 * SAFETY: If the gate hits something and the servo buzzes or strains, step back
 * immediately. A stalled servo can overheat.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test: Gate Servo Tuner", group = "Test")
public class GateServoTuner extends LinearOpMode {

    // Step sizes in servo units (0.0 - 1.0). On a 300-degree servo,
    // 0.005 is about 1.5 degrees and 0.025 is about 7.5 degrees.
    private static final double SMALL_STEP = 0.005;
    private static final double BIG_STEP   = 0.025;

    // On this robot a HIGHER servo value moves the gate DOWN, so "up" subtracts.
    // If D-pad up moves the gate down, change this to +1.0.
    private static final double UP_SIGN = -1.0;

    @Override
    public void runOpMode() {
        // Only the servo is used here, so we don't call RobotHardware.init().
        // That keeps all motors untouched while tuning.
        Servo gate = hardwareMap.get(Servo.class, "control_gate_1");

        double position    = RobotHardware.GATE_CLOSED_POSITION;
        double savedOpen   = RobotHardware.GATE_OPEN_POSITION;
        double savedClosed = RobotHardware.GATE_CLOSED_POSITION;

        boolean prevUp = false, prevDown = false;
        boolean prevA = false, prevB = false, prevX = false, prevY = false;

        telemetry.addLine("Gate Servo Tuner ready. Hands clear of the gate.");
        telemetry.addLine("Press PLAY - the gate moves to the CLOSED preset.");
        telemetry.update();

        waitForStart();
        gate.setPosition(position);

        while (opModeIsActive()) {
            double step = gamepad1.right_bumper ? BIG_STEP : SMALL_STEP;

            // Nudge up / down (one step per press).
            if (gamepad1.dpad_up && !prevUp)     position += UP_SIGN * step;
            if (gamepad1.dpad_down && !prevDown) position -= UP_SIGN * step;

            // Save the current position.
            if (gamepad1.a && !prevA) savedOpen   = position;
            if (gamepad1.b && !prevB) savedClosed = position;

            // Jump to a saved position to check it.
            if (gamepad1.x && !prevX) position = savedOpen;
            if (gamepad1.y && !prevY) position = savedClosed;

            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevY = gamepad1.y;

            // Keep within the servo's legal range and send it.
            position = Math.max(0.0, Math.min(1.0, position));
            gate.setPosition(position);

            double travelDeg = Math.abs(savedClosed - savedOpen)
                    * RobotHardware.SERVO_RANGE_DEGREES;

            telemetry.addLine("=== GATE SERVO TUNER ===");
            telemetry.addData("Current position", "%.3f", position);
            telemetry.addData("Approx. angle", "%.1f deg",
                    position * RobotHardware.SERVO_RANGE_DEGREES);
            telemetry.addData("Step size", step == BIG_STEP ? "BIG (RB held)" : "small");
            telemetry.addLine();
            telemetry.addData("Saved OPEN   (A)", "%.3f", savedOpen);
            telemetry.addData("Saved CLOSED (B)", "%.3f", savedClosed);
            telemetry.addData("Travel between them", "%.1f deg", travelDeg);
            telemetry.addLine();
            telemetry.addLine("Copy into RobotHardware.java:");
            telemetry.addLine(String.format("  GATE_OPEN_POSITION   = %.3f", savedOpen));
            telemetry.addLine(String.format("  GATE_CLOSED_POSITION = %.3f", savedClosed));
            telemetry.addLine();
            telemetry.addLine("Dpad up/down=nudge  RB=big steps");
            telemetry.addLine("A=save OPEN  B=save CLOSED  X/Y=go to OPEN/CLOSED");
            telemetry.update();
        }
    }
}
