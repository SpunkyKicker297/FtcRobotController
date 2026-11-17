/*
 * AutoAdvanced.java
 *
 * AUTONOMOUS: shoot the pre-loaded pollen, then drive to the loading/parking zone.
 *
 * SEQUENCE
 *   1. Shooter STARTUP (same steps as TeleOp, via ShooterSequence):
 *        flywheel spins up -> RPM verified at target -> gate opens -> feed intake runs
 *   2. Keep shooting for SHOOT_SECONDS (default 5 s) to empty the 4 pre-loads.
 *   3. Shooter SHUTDOWN (reverse): feed off -> gate closed -> flywheel off.
 *      The flywheel then coasts down while the robot starts driving.
 *   4. The movement routine below (drive off wall, turn, drive across, park).
 *
 * If the flywheel doesn't pass the RPM check within SPINUP_TIMEOUT_SECONDS, the
 * robot FIRES ANYWAY (gate opens, feed runs) so the full sequence always happens.
 * The "Shot" line on the Driver Station stays visible during driving and says
 * whether the RPM was verified, how long spin-up took, and the peak RPM reached.
 *
 * Tune TARGET_RPM / RPM_TOLERANCE in ShooterSequence.java (shared with TeleOp).
 *
 * The movement routine (down in runOpMode) reads like plain English --
 *
 *     driveForward(12);
 *     turnLeft(90);
 *     ...
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

@Autonomous(name = "Auto: Advanced (shoot+maneuver to loading)", group = "Competition")
public class AutoAdvanced extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime timer = new ElapsedTime();
    private ShooterSequence shooter;   // created after robot.init()
    // Summary of how the shot went; stays on the Driver Station during driving.
    private String shotReport = "not run yet";

    // ---------------------------------------------------------------------
    //  SHOOTING SETTINGS  --  tune these
    // ---------------------------------------------------------------------
    // How long to keep the gate open and the feed running once at speed.
    // 5 s is a starting point for 4 pre-loaded pollen; shorten it once you
    // know how fast they actually fire, to leave more time for parking.
    private static final double SHOOT_SECONDS = 5.0;
    // Max time to wait for the flywheel to reach TARGET_RPM before giving up
    // on the shot and going to park.
    private static final double SPINUP_TIMEOUT_SECONDS = 5.0;

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
        shooter = new ShooterSequence(robot);

        telemetry.addData("Status", "Initialized - waiting for PLAY");
        telemetry.addData("Plan", "shoot %.1f s @ %.0f RPM, then park",
                SHOOT_SECONDS, ShooterSequence.TARGET_RPM);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        robot.resetHeading();
        targetHeading = 0.0;

        // =====================================================================
        // STEP 1-3: shoot the pre-loaded pollen, then shut the shooter down.
        // =====================================================================
        shootPreloads(SHOOT_SECONDS);

        // =====================================================================
        // STEP 4: THE MOVEMENT ROUTINE  --  read it top to bottom, edit freely.
        // =====================================================================
        driveBackward(12);   // pull off the wall
        strafeRight(12*3.5);     // strafe around the collector
        driveBackward(12*5);   // park

        robot.stop();

        telemetry.addData("Status", "Done - parked");
        telemetry.addData("Shot", shotReport);
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

    /**
     * Run the full shooter startup sequence, shoot for a set time, then run the
     * shutdown sequence. The robot stays still the whole time.
     *
     * @param shootSeconds how long to feed pollen once the flywheel is at speed
     */
    public void shootPreloads(double shootSeconds) {
        if (!opModeIsActive()) return;

        // Startup step 1: spin up the flywheel.
        shooter.toggle();

        // Startup step 2-3: wait until RPM is verified and feeding has started.
        double peakRpm = 0.0;
        timer.reset();
        while (opModeIsActive()
                && !shooter.isGateOpen()
                && timer.seconds() < SPINUP_TIMEOUT_SECONDS) {
            shooter.update();
            peakRpm = Math.max(peakRpm, robot.getShooterRpm());
            showShooterTelemetry("Spinning up");
        }
        double spinupSeconds = timer.seconds();

        if (shooter.isGateOpen()) {
            shotReport = String.format("RPM verified in %.1f s (peak %.0f)",
                    spinupSeconds, peakRpm);
        } else {
            // RPM check never passed in time. Shoot anyway so the full sequence
            // always runs; the report shows why the check didn't pass.
            shooter.forceFeed();
            shotReport = String.format("RPM NOT verified after %.1f s (peak %.0f of %.0f)"
                    + " - fired anyway", spinupSeconds, peakRpm, ShooterSequence.TARGET_RPM);
        }

        // Gate is up; wait GATE_SETTLE_SECONDS for the secondary intake to start.
        while (opModeIsActive()
                && shooter.getState() == ShooterSequence.State.OPENING) {
            shooter.update();
            showShooterTelemetry("Gate opening");
        }

        // Both intakes are running: keep shooting for the full window.
        if (opModeIsActive()) {
            timer.reset();
            while (opModeIsActive() && timer.seconds() < shootSeconds) {
                shooter.update();
                showShooterTelemetry(String.format("Shooting (%.1f s left)",
                        shootSeconds - timer.seconds()));
            }
        }

        // Shutdown (reverse): feed off -> gate closed -> flywheel off.
        // The flywheel coasts down while the robot starts the movement routine.
        shooter.forceStop();
    }

    /** Telemetry shown while the shooter sequence runs. */
    private void showShooterTelemetry(String phase) {
        telemetry.addData("Phase", phase);
        telemetry.addData("Shot", shotReport);
        telemetry.addData("Shooter state", shooter.getState());
        telemetry.addData("RPM actual / target", "%.0f / %.0f",
                robot.getShooterRpm(), ShooterSequence.TARGET_RPM);
        telemetry.update();
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

            telemetry.addData("Shot", shotReport);
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

            telemetry.addData("Shot", shotReport);
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
