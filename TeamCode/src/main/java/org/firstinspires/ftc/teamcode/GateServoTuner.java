/*
 * GateServoTuner.java
 *
 * A dedicated test OpMode for finding and checking the OPEN (up) and CLOSED (down)
 * positions of the control gate servo (control_gate_1). No motors run, so you can
 * safely watch the gate while you adjust it.
 *
 * Button names match the REV USB PS4-style gamepad (REV-31-2983).
 *
 * -----------------------------------------------------------------------------
 * CONTROLS (gamepad1)
 * -----------------------------------------------------------------------------
 *   COMPETITION POSITIONS (the values currently in RobotHardware.java)
 *     Triangle (press) ...... full move to competition UP   (GATE_OPEN_POSITION)
 *     Cross    (press) ...... full move to competition DOWN (GATE_CLOSED_POSITION)
 *
 *   FINE ADJUSTMENT
 *     D-pad up   (press) .... move the gate UP one small step
 *     D-pad down (press) .... move the gate DOWN one small step
 *     Hold R1 ............... use BIG steps instead
 *
 *   SAVING NEW VALUES WHILE TUNING
 *     Square (press) ........ save current position as the new OPEN value
 *     Circle (press) ........ save current position as the new CLOSED value
 *     D-pad left  (press) ... jump to the saved OPEN value
 *     D-pad right (press) ... jump to the saved CLOSED value
 *
 * -----------------------------------------------------------------------------
 * HOW TO USE IT
 * -----------------------------------------------------------------------------
 *   1. Press Triangle and Cross to see how the CURRENT competition values behave.
 *   2. If they need changing, nudge with the D-pad to the right "open" spot and
 *      press Square, then to the right "closed" spot and press Circle.
 *   3. Copy the two numbers shown on the Driver Station into RobotHardware.java:
 *        GATE_OPEN_POSITION   = <OPEN value>
 *        GATE_CLOSED_POSITION = <CLOSED value>
 *      (Replace the GATE_CLOSED_POSITION formula with the plain number.)
 *   4. Redeploy, and press Triangle / Cross again to confirm.
 *
 * Saved values are NOT stored on the robot -- write them down before you stop.
 *
 * SAFETY: If the gate hits something and the servo buzzes or strains, move it
 * back immediately. A stalled servo can overheat.
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
        String lastAction  = "Start (competition DOWN)";

        boolean prevUp = false, prevDown = false, prevLeft = false, prevRight = false;
        boolean prevTriangle = false, prevCross = false, prevSquare = false, prevCircle = false;

        telemetry.addLine("Gate Servo Tuner ready. Hands clear of the gate.");
        telemetry.addLine("Press PLAY - the gate moves to competition DOWN.");
        telemetry.update();

        waitForStart();
        gate.setPosition(position);

        while (opModeIsActive()) {
            double step = gamepad1.right_bumper ? BIG_STEP : SMALL_STEP;  // R1

            // Competition positions: full move in a single press.
            if (gamepad1.triangle && !prevTriangle) {
                position = RobotHardware.GATE_OPEN_POSITION;
                lastAction = "Competition UP";
            }
            if (gamepad1.cross && !prevCross) {
                position = RobotHardware.GATE_CLOSED_POSITION;
                lastAction = "Competition DOWN";
            }

            // Fine adjustment.
            if (gamepad1.dpad_up && !prevUp) {
                position += UP_SIGN * step;
                lastAction = "Nudge up";
            }
            if (gamepad1.dpad_down && !prevDown) {
                position -= UP_SIGN * step;
                lastAction = "Nudge down";
            }

            // Save and recall tuning values.
            if (gamepad1.square && !prevSquare) {
                savedOpen = position;
                lastAction = "Saved OPEN";
            }
            if (gamepad1.circle && !prevCircle) {
                savedClosed = position;
                lastAction = "Saved CLOSED";
            }
            if (gamepad1.dpad_left && !prevLeft) {
                position = savedOpen;
                lastAction = "Jump to saved OPEN";
            }
            if (gamepad1.dpad_right && !prevRight) {
                position = savedClosed;
                lastAction = "Jump to saved CLOSED";
            }

            prevTriangle = gamepad1.triangle;
            prevCross    = gamepad1.cross;
            prevSquare   = gamepad1.square;
            prevCircle   = gamepad1.circle;
            prevUp       = gamepad1.dpad_up;
            prevDown     = gamepad1.dpad_down;
            prevLeft     = gamepad1.dpad_left;
            prevRight    = gamepad1.dpad_right;

            // Keep within the servo's legal range and send it.
            position = Math.max(0.0, Math.min(1.0, position));
            gate.setPosition(position);

            double travelDeg = Math.abs(savedClosed - savedOpen)
                    * RobotHardware.SERVO_RANGE_DEGREES;

            telemetry.addLine("=== GATE SERVO TUNER ===");
            telemetry.addData("Current position", "%.3f", position);
            telemetry.addData("Approx. angle", "%.1f deg",
                    position * RobotHardware.SERVO_RANGE_DEGREES);
            telemetry.addData("Last action", lastAction);
            telemetry.addData("Step size", step == BIG_STEP ? "BIG (R1 held)" : "small");
            telemetry.addLine();
            telemetry.addData("Competition UP / DOWN", "%.3f / %.3f",
                    RobotHardware.GATE_OPEN_POSITION, RobotHardware.GATE_CLOSED_POSITION);
            telemetry.addData("Saved OPEN   (Square)", "%.3f", savedOpen);
            telemetry.addData("Saved CLOSED (Circle)", "%.3f", savedClosed);
            telemetry.addData("Saved travel", "%.1f deg", travelDeg);
            telemetry.addLine();
            telemetry.addLine("Copy into RobotHardware.java:");
            telemetry.addLine(String.format("  GATE_OPEN_POSITION   = %.3f", savedOpen));
            telemetry.addLine(String.format("  GATE_CLOSED_POSITION = %.3f", savedClosed));
            telemetry.addLine();
            telemetry.addLine("Triangle=UP  Cross=DOWN  Dpad up/down=nudge  R1=big");
            telemetry.addLine("Square/Circle=save OPEN/CLOSED  Dpad L/R=go to saved");
            telemetry.update();
        }
    }
}
