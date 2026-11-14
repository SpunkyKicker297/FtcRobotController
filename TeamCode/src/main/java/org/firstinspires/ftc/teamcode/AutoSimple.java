/*
 * AutoSimple.java
 *
 * A simplified AUTONOMOUS OpMode built on top of easy-to-read maneuver functions.
 *
 * The whole point of this file: the actual routine (down in runOpMode) reads like
 * plain English --
 *
 *     driveForward(18);
 *     turnLeft(90);
 *     driveForward(18);
 *     turnLeft(90);
 *     driveForward(16);
 *
 * All the encoder math and IMU turning live in the helper methods below, so you
 * can build or rearrange a routine just by calling these functions. Available
 * maneuvers:
 *
 *     driveForward(inches)    driveBackward(inches)
 *     strafeLeft(inches)      strafeRight(inches)
 *     turnLeft(degrees)       turnRight(degrees)
 *     wait(seconds)
 *
 * This produces the SAME routine as "Auto: Park in Loading Zone", just written in
 * the simpler style. Run whichever you prefer; they both appear on the Driver
 * Station. Accuracy depends on the same RobotHardware constants (TICKS_PER_INCH,
 * wheel size) and IMU mounting.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Simple (maneuvers)", group = "Competition")
public class AutoSimple extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime timer = new ElapsedTime();

    // Default speeds for the maneuvers. Lower = slower but more repeatable.
    private static final double DRIVE_POWER = 0.4;
    private static final double TURN_POWER  = 0.4;

    // How close (in degrees) a turn must get before we call it "done", plus a
    // safety timeout (seconds) per maneuver so nothing eats the whole 30 seconds.
    private static final double HEADING_TOLERANCE_DEG = 2.0;
    private static final double MOVE_TIMEOUT_SECONDS  = 5.0;

    // Tracks the heading we WANT the robot to be facing, so turns stack cleanly
    // (turnLeft(90) then turnLeft(90) ends up at 180, with no drift between them).
    private double targetHeading = 0.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized - waiting for PLAY");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        robot.resetHeading();
        targetHeading = 0.0;

        // =====================================================================
        // THE ROUTINE  --  read it top to bottom, edit freely.
        // =====================================================================
        driveForward(12);   // pull off the wall
        driveBackward(1);   // make sure it doesn't touch the wall in the parking zone.
        //turnRight(90);       // face across the field
        //driveForward(18);   // drive across (increase for a longer run)
        //turnRight(90);       // face the parking area
        //driveForward(16);   // park

        robot.stop();

        telemetry.addData("Status", "Done - parked");
        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }

    // =====================================================================
    //  MANEUVER FUNCTIONS  --  the reusable vocabulary
    // =====================================================================

    /** Drive straight forward a number of inches. */
    public void driveForward(double inches) {
        straightDrive(DRIVE_POWER, 0.0, inches, "Forward " + inches + "\"");
    }

    /** Drive straight backward a number of inches. */
    public void driveBackward(double inches) {
        straightDrive(-DRIVE_POWER, 0.0, inches, "Backward " + inches + "\"");
    }

    /** Strafe (slide sideways) to the LEFT a number of inches. */
    public void strafeLeft(double inches) {
        straightDrive(0.0, -DRIVE_POWER, inches, "Strafe left " + inches + "\"");
    }

    /** Strafe (slide sideways) to the RIGHT a number of inches. */
    public void strafeRight(double inches) {
        straightDrive(0.0, DRIVE_POWER, inches, "Strafe right " + inches + "\"");
    }

    /** Turn LEFT (counter-clockwise) a number of degrees. */
    public void turnLeft(double degrees) {
        targetHeading += degrees;           // left = positive heading
        turnToTarget("Turn left " + degrees + " deg");
    }

    /** Turn RIGHT (clockwise) a number of degrees. */
    public void turnRight(double degrees) {
        targetHeading -= degrees;           // right = negative heading
        turnToTarget("Turn right " + degrees + " deg");
    }

    /** Pause for a number of seconds (robot holds still). */
    public void wait(double seconds) {
        robot.stop();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            idle();
        }
    }

    // =====================================================================
    //  SHARED INTERNALS  --  you normally don't need to touch these
    // =====================================================================

    /**
     * Drive in a straight line (forward/back OR strafe) for a distance in inches,
     * using the drive encoders to measure how far we've gone.
     *
     * @param axial   forward power (+) / backward (-); 0 when strafing
     * @param lateral strafe power right (+) / left (-); 0 when driving straight
     * @param inches  distance to travel (always positive)
     * @param label   telemetry label
     */
    private void straightDrive(double axial, double lateral, double inches, String label) {
        if (!opModeIsActive()) return;

        int targetTicks = (int) Math.round(Math.abs(inches) * RobotHardware.TICKS_PER_INCH);

        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        while (opModeIsActive()
                && robot.getAverageDriveTicks() < targetTicks
                && timer.seconds() < MOVE_TIMEOUT_SECONDS) {

            robot.drive(axial, lateral, 0.0);

            telemetry.addData("Maneuver", label);
            telemetry.addData("Target / Current", "%d / %.0f ticks",
                    targetTicks, robot.getAverageDriveTicks());
            telemetry.update();
        }
        robot.stop();
        sleep(150);
    }

    /**
     * Turn in place until the robot reaches {@link #targetHeading}. Proportional
     * control eases the turn as it nears the target to avoid overshoot.
     */
    private void turnToTarget(String label) {
        if (!opModeIsActive()) return;

        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        double error = angleWrap(targetHeading - robot.getHeading());

        while (opModeIsActive()
                && Math.abs(error) > HEADING_TOLERANCE_DEG
                && timer.seconds() < MOVE_TIMEOUT_SECONDS) {

            double power = TURN_POWER * (error / 45.0);
            if (power >  TURN_POWER) power =  TURN_POWER;
            if (power < -TURN_POWER) power = -TURN_POWER;
            if (Math.abs(power) < 0.12) power = Math.copySign(0.12, power);

            robot.drive(0.0, 0.0, power);

            telemetry.addData("Maneuver", label);
            telemetry.addData("Target / Current", "%.0f / %.1f deg",
                    targetHeading, robot.getHeading());
            telemetry.addData("Error", "%.1f deg", error);
            telemetry.update();

            error = angleWrap(targetHeading - robot.getHeading());
        }
        robot.stop();
        sleep(150);
    }

    /** Normalize an angle to (-180, 180] so turns take the short way around. */
    private double angleWrap(double degrees) {
        while (degrees >  180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
}
