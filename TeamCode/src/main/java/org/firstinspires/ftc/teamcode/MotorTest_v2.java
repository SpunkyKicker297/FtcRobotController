/*
 * MotorTest_v2.java
 *
 * A bench-test OpMode for operating EVERY motor and the gate servo independently,
 * with live telemetry. Put the robot on blocks (wheels off the ground) and keep
 * hands clear of the intakes and flywheel before pressing PLAY.
 *
 * Use it to confirm:
 *   1. Every device is named correctly in the Robot Configuration (a wrong name
 *      crashes at INIT -- that tells you which one).
 *   2. Every motor spins the RIGHT direction (flip setDirection in RobotHardware).
 *   3. The shooter reaches its target RPM, and the gate's open/closed positions
 *      are right.
 *
 * -----------------------------------------------------------------------------
 * CONTROLS (gamepad1)
 * -----------------------------------------------------------------------------
 *   DRIVE MOTORS (one at a time) -- hold a button, push LEFT STICK Y for power:
 *     X ... front left       Y ... front right
 *     A ... back left        B ... back right
 *
 *   PRIMARY INTAKE (intake_1)
 *     Right bumper (hold) ... IN          Left bumper (hold) ... OUT
 *
 *   SECONDARY / FEED INTAKE (intake_2)
 *     D-pad up (hold) ....... IN          D-pad down (hold) .... OUT
 *
 *   SHOOTER (shooter_1)
 *     Right trigger ......... variable POWER (open loop), 0 - 100%
 *     Start (press) ......... toggle closed-loop RPM at ShooterSequence.TARGET_RPM
 *                             (pressing the trigger overrides the RPM mode)
 *
 *   CONTROL GATE (control_gate_1)
 *     D-pad left (press) .... go to OPEN position
 *     D-pad right (press) ... go to CLOSED position
 *     Hold LEFT TRIGGER + D-pad left/right ... nudge the gate a small step,
 *                             so you can find the exact positions to copy into
 *                             GATE_OPEN_POSITION / GATE_CLOSED_POSITION.
 *
 * Releasing a hold control stops that motor.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Test v2: All Motors", group = "Test")
public class MotorTest_v2 extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();

    // Size of one gate "nudge" step (servo position units, 0.0 - 1.0).
    private static final double GATE_NUDGE_STEP = 0.005;

    // Shooter closed-loop RPM toggle.
    private boolean rpmMode = false;

    // Previous button states, for press (edge) detection.
    private boolean prevStart = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);   // gate starts at the CLOSED position

        double gatePosition = RobotHardware.GATE_CLOSED_POSITION;

        telemetry.addLine("Motor Test v2 ready. Robot on blocks, hands clear.");
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------- DRIVE MOTORS (one at a time) ----------
            double stick = -gamepad1.left_stick_y;  // push forward = positive
            double fl = gamepad1.x ? stick : 0.0;
            double fr = gamepad1.y ? stick : 0.0;
            double bl = gamepad1.a ? stick : 0.0;
            double br = gamepad1.b ? stick : 0.0;
            robot.setMotorPowers(fl, fr, bl, br);

            // ---------- PRIMARY INTAKE ----------
            double intake1Power = 0.0;
            if (gamepad1.right_bumper)     intake1Power =  RobotHardware.INTAKE_POWER;
            else if (gamepad1.left_bumper) intake1Power = -RobotHardware.INTAKE_POWER;
            robot.setIntakePower(intake1Power);

            // ---------- SECONDARY / FEED INTAKE ----------
            double intake2Power = 0.0;
            if (gamepad1.dpad_up)        intake2Power =  RobotHardware.INTAKE2_POWER;
            else if (gamepad1.dpad_down) intake2Power = -RobotHardware.INTAKE2_POWER;
            robot.setIntake2Power(intake2Power);

            // ---------- SHOOTER ----------
            if (gamepad1.start && !prevStart) {
                rpmMode = !rpmMode;
                if (!rpmMode) robot.shooterOff();
            }
            prevStart = gamepad1.start;

            String shooterMode;
            if (gamepad1.right_trigger > 0.05) {
                rpmMode = false;                         // trigger overrides RPM mode
                robot.setShooterPower(gamepad1.right_trigger);
                shooterMode = String.format("POWER %.0f%%", gamepad1.right_trigger * 100);
            } else if (rpmMode) {
                robot.setShooterRpm(ShooterSequence.TARGET_RPM);
                shooterMode = "RPM (closed loop)";
            } else {
                robot.shooterOff();
                shooterMode = "off";
            }

            // ---------- CONTROL GATE ----------
            boolean fine = gamepad1.left_trigger > 0.5;
            if (gamepad1.dpad_left && !prevDpadLeft) {
                gatePosition = fine ? gatePosition - GATE_NUDGE_STEP
                                    : RobotHardware.GATE_OPEN_POSITION;
            }
            if (gamepad1.dpad_right && !prevDpadRight) {
                gatePosition = fine ? gatePosition + GATE_NUDGE_STEP
                                    : RobotHardware.GATE_CLOSED_POSITION;
            }
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;
            gatePosition = Math.max(0.0, Math.min(1.0, gatePosition));
            robot.setGatePosition(gatePosition);

            // ---------- TELEMETRY ----------
            double shooterRpm = robot.getShooterRpm();
            double gateDegreesFromOpen = (gatePosition - RobotHardware.GATE_OPEN_POSITION)
                    * RobotHardware.SERVO_RANGE_DEGREES;

            telemetry.addLine("=== DRIVE (hold X/Y/A/B + left stick) ===");
            telemetry.addData("FL / FR", "%+.2f / %+.2f", fl, fr);
            telemetry.addData("BL / BR", "%+.2f / %+.2f", bl, br);

            telemetry.addLine("=== INTAKES ===");
            telemetry.addData("Primary (bumpers)", "%+.2f", intake1Power);
            telemetry.addData("Feed (dpad up/down)", "%+.2f", intake2Power);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", shooterMode);
            telemetry.addData("RPM actual / target", "%.0f / %.0f",
                    shooterRpm, ShooterSequence.TARGET_RPM);
            telemetry.addData("At speed?",
                    Math.abs(shooterRpm - ShooterSequence.TARGET_RPM)
                            <= ShooterSequence.RPM_TOLERANCE ? "YES" : "no");

            telemetry.addLine("=== CONTROL GATE ===");
            telemetry.addData("Servo position", "%.3f", robot.getGatePosition());
            telemetry.addData("Degrees from OPEN", "%.1f", gateDegreesFromOpen);
            telemetry.addData("OPEN / CLOSED preset", "%.3f / %.3f",
                    RobotHardware.GATE_OPEN_POSITION, RobotHardware.GATE_CLOSED_POSITION);
            telemetry.addData("Fine adjust (LT held)", fine ? "ON" : "off");
            telemetry.update();
        }

        robot.stop();
    }
}
