/*
 * AutoParkInLoadingZone.java
 *
 * This is your AUTONOMOUS OpMode. It runs during the first ~30 seconds of the
 * match, when NO driver input is allowed. The goal is to leave the starting wall
 * and navigate to the alliance loading zone / parking area for the parking points.
 *
 * -----------------------------------------------------------------------------
 * HOW THE MOVEMENT WORKS
 * -----------------------------------------------------------------------------
 * This version drives by DISTANCE (inches) using the motor encoders, and turns by
 * ANGLE (degrees) using the Control Hub's built-in IMU. That makes it far more
 * repeatable than time-based driving -- "drive 18 inches" really does drive about
 * 18 inches regardless of battery level.
 *
 * THE ROUTINE (as requested):
 *   1. Pull away from the wall ~18 inches.
 *   2. Turn 90 degrees to the LEFT.
 *   3. Drive ~18 inches (configurable -- make it longer if needed).
 *   4. Turn 90 degrees to the LEFT again.
 *   5. Park by driving forward ~16 inches.
 *
 * -----------------------------------------------------------------------------
 * BEFORE THIS WORKS WELL, CHECK TWO THINGS IN RobotHardware.java:
 * -----------------------------------------------------------------------------
 *   1. TICKS_PER_MOTOR_REV must match YOUR goBILDA motor (gear ratio).
 *   2. WHEEL_DIAMETER_INCHES must match your wheels.
 * If distances come out wrong, those two constants are what to tune first.
 * Also confirm the IMU mounting directions in RobotHardware.init().
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Park in Loading Zone", group = "Competition")
public class AutoParkInLoadingZone extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------------------------------------------------------------
    //  TUNABLE ROUTINE SETTINGS  --  adjust these for your field
    // ---------------------------------------------------------------------
    private static final double PULL_OFF_WALL_INCHES = 18.0; // step 1
    private static final double DRIVE_ACROSS_INCHES  = 18.0; // step 3 (make longer if needed)
    private static final double PARK_FORWARD_INCHES  = 16.0; // step 5
    private static final double TURN_ANGLE_DEGREES   = 90.0; // left turns

    // Power levels. Lower = slower but more accurate/repeatable.
    private static final double DRIVE_POWER = 0.4;
    private static final double TURN_POWER  = 0.4;

    // Turning tolerance (degrees) and a safety timeout (seconds) so a turn that
    // can't quite settle doesn't eat the whole autonomous period.
    private static final double HEADING_TOLERANCE_DEG = 2.0;
    private static final double MOVE_TIMEOUT_SECONDS  = 5.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized - waiting for PLAY");
        telemetry.addData("Routine", "off wall %.0f\" | L90 | across %.0f\" | L90 | park %.0f\"",
                PULL_OFF_WALL_INCHES, DRIVE_ACROSS_INCHES, PARK_FORWARD_INCHES);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        // Make sure heading starts at 0 right when the match begins.
        robot.resetHeading();

        // =====================================================================
        // AUTONOMOUS ROUTINE
        // =====================================================================

        // Step 1: pull away from the wall.
        driveInches(PULL_OFF_WALL_INCHES, "1/5: Pulling off the wall");

        // Step 2: turn 90 degrees LEFT. Left = counter-clockwise = POSITIVE angle.
        turnToHeading(+TURN_ANGLE_DEGREES, "2/5: Turning left 90");

        // Step 3: drive across toward the loading zone.
        driveInches(DRIVE_ACROSS_INCHES, "3/5: Driving across");

        // Step 4: turn another 90 degrees LEFT (now 180 total from start).
        turnToHeading(+2.0 * TURN_ANGLE_DEGREES, "4/5: Turning left 90 again");

        // Step 5: drive forward into the parking area.
        driveInches(PARK_FORWARD_INCHES, "5/5: Parking");

        robot.stop();

        telemetry.addData("Status", "Autonomous complete - parked");
        telemetry.addData("Final heading", "%.1f deg", robot.getHeading());
        telemetry.update();

        // Hold until the autonomous period ends.
        while (opModeIsActive()) {
            idle();
        }
    }

    /**
     * Drive straight forward a set distance in inches using the drive encoders.
     * A positive distance drives forward; a negative distance drives backward.
     *
     * @param inches how far to drive (forward is positive)
     * @param label  telemetry label for this step
     */
    private void driveInches(double inches, String label) {
        if (!opModeIsActive()) return;

        int targetTicks = (int) Math.round(Math.abs(inches) * RobotHardware.TICKS_PER_INCH);
        double direction = Math.signum(inches); // +1 forward, -1 backward

        // Zero the encoders, then run in plain power mode and watch the count.
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        while (opModeIsActive()
                && robot.getAverageDriveTicks() < targetTicks
                && timer.seconds() < MOVE_TIMEOUT_SECONDS) {

            robot.drive(DRIVE_POWER * direction, 0.0, 0.0);

            telemetry.addData("Step", label);
            telemetry.addData("Target / Current", "%d / %.0f ticks",
                    targetTicks, robot.getAverageDriveTicks());
            telemetry.update();
        }
        robot.stop();
        sleep(150); // brief settle so the next move starts from rest
    }

    /**
     * Turn in place until the robot reaches the target heading (absolute, measured
     * from where resetHeading() was last called). Positive = left (CCW).
     *
     * Uses simple proportional control: the turn slows down as it approaches the
     * target so it doesn't overshoot.
     *
     * @param targetHeadingDeg absolute heading to reach, in degrees
     * @param label            telemetry label for this step
     */
    private void turnToHeading(double targetHeadingDeg, String label) {
        if (!opModeIsActive()) return;

        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        double error = angleWrap(targetHeadingDeg - robot.getHeading());

        while (opModeIsActive()
                && Math.abs(error) > HEADING_TOLERANCE_DEG
                && timer.seconds() < MOVE_TIMEOUT_SECONDS) {

            // Proportional power: full TURN_POWER when far away, easing to a floor
            // of 0.12 near the target so we still have enough power to finish.
            double power = TURN_POWER * (error / 45.0);
            if (power >  TURN_POWER) power =  TURN_POWER;
            if (power < -TURN_POWER) power = -TURN_POWER;
            if (Math.abs(power) < 0.12) power = Math.copySign(0.12, power);

            // Positive yaw spins the robot; sign of error picks the direction.
            robot.drive(0.0, 0.0, power);

            telemetry.addData("Step", label);
            telemetry.addData("Target / Current", "%.0f / %.1f deg",
                    targetHeadingDeg, robot.getHeading());
            telemetry.addData("Error", "%.1f deg", error);
            telemetry.update();

            error = angleWrap(targetHeadingDeg - robot.getHeading());
        }
        robot.stop();
        sleep(150);
    }

    /** Normalize an angle to the range (-180, 180] so turns take the short way around. */
    private double angleWrap(double degrees) {
        while (degrees >  180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
}
