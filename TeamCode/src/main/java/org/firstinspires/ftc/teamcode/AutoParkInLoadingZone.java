/*
 * AutoParkInLoadingZone.java
 *
 * This is your AUTONOMOUS OpMode. It runs during the first ~30 seconds of the
 * match, when NO driver input is allowed. The goal of this starter routine is
 * simple and reliable: drive off the starting wall and navigate to the alliance
 * loading zone / parking area to earn the parking points.
 *
 * -----------------------------------------------------------------------------
 * HOW THE MOVEMENT WORKS (important for a first-year team to understand)
 * -----------------------------------------------------------------------------
 * This version uses TIME-BASED driving: "drive forward at 40% power for 1.2
 * seconds", etc. This is the easiest way to get a working autonomous on day one.
 * It is NOT super precise -- battery level and floor friction change the result,
 * so you WILL need to tune the time/power numbers below on your actual field.
 *
 * A more accurate approach uses the motor encoders (built into goBILDA motors)
 * to drive exact distances, and an IMU to turn exact angles. Once this time-based
 * version works, upgrading to encoders is a great next step.
 *
 * -----------------------------------------------------------------------------
 * HOW TO TUNE IT
 * -----------------------------------------------------------------------------
 * 1. Place the robot on the field in its real starting position against the wall.
 * 2. Run this OpMode and watch where it ends up.
 * 3. Adjust the seconds / power in the "AUTONOMOUS ROUTINE" section until the
 *    robot reliably parks in your loading zone.
 * 4. Remember the field is mirrored for red vs blue alliance -- you may need a
 *    "strafe left" instead of "strafe right" depending on your side. See the
 *    ALLIANCE setting below.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto: Park in Loading Zone", group = "Competition")
public class AutoParkInLoadingZone extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime timer = new ElapsedTime();

    // Which side of the field are you starting on? This flips the strafe
    // direction so the same code works for both alliances.
    //   +1.0  -> strafe toward the RIGHT side of the field
    //   -1.0  -> strafe toward the LEFT side of the field
    // Set this to match where YOUR loading zone is from the starting position.
    private static final double ALLIANCE_STRAFE_DIRECTION = +1.0;

    // Default power used for autonomous moves. Lower = slower but more repeatable.
    private static final double DRIVE_POWER = 0.4;
    private static final double TURN_POWER  = 0.4;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized - waiting for PLAY");
        telemetry.addData("Alliance strafe", ALLIANCE_STRAFE_DIRECTION > 0 ? "RIGHT" : "LEFT");
        telemetry.update();

        // Wait for the PLAY button.
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        // =====================================================================
        // AUTONOMOUS ROUTINE  --  tune these steps for your field!
        // =====================================================================

        // Step 1: Drive forward off the wall.
        driveForSeconds(DRIVE_POWER, 0.0, 0.0, 1.2, "Driving off the wall");

        // Step 2: Strafe sideways toward the loading zone / parking area.
        driveForSeconds(0.0, DRIVE_POWER * ALLIANCE_STRAFE_DIRECTION, 0.0, 1.0,
                "Strafing to loading zone");

        // Step 3: Nudge forward to settle fully into the parking area.
        driveForSeconds(DRIVE_POWER, 0.0, 0.0, 0.5, "Settling into park");

        // Make sure the robot is stopped at the end.
        robot.stop();

        telemetry.addData("Status", "Autonomous complete - parked");
        telemetry.update();

        // Hold here (doing nothing) until autonomous time runs out.
        while (opModeIsActive()) {
            idle();
        }
    }

    /**
     * Helper: drive with the given motion for a set number of seconds, then stop.
     * This checks opModeIsActive() continuously so the robot stops immediately if
     * the match ends or STOP is pressed.
     *
     * @param axial    forward (+) / backward (-)
     * @param lateral  strafe right (+) / left (-)
     * @param yaw      turn clockwise (+) / counter-clockwise (-)
     * @param seconds  how long to run this move
     * @param label    text shown on the Driver Station telemetry
     */
    private void driveForSeconds(double axial, double lateral, double yaw,
                                 double seconds, String label) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            robot.drive(axial, lateral, yaw);

            telemetry.addData("Step", label);
            telemetry.addData("Time left", "%.1f s", seconds - timer.seconds());
            telemetry.update();
        }
        robot.stop();
    }
}
