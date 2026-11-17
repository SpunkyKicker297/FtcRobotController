/*
 * TeleOpDrive.java
 *
 * This is your DRIVER-CONTROLLED OpMode. During the "TeleOp" period of a match,
 * this reads the gamepad (your goBILDA controller plugged into the Driver Station)
 * and drives the robot.
 *
 * DRIVER controls (gamepad1 = first controller):
 *   Left stick  Y  -> drive forward / backward
 *   Left stick  X  -> strafe left / right
 *   Right stick X  -> turn (yaw) left / right
 *   Right trigger  -> hold for "turbo" (full speed); otherwise speed is reduced
 *                     for easier, more precise control (great for new drivers).
 *   Right bumper (hold) -> run PRIMARY intake IN
 *   Left bumper  (hold) -> run PRIMARY intake OUT (eject)
 *
 * SHOOTER controls (gamepad2 = second controller):
 *   The FLYWHEEL spins up at PLAY and runs for the whole TeleOp.
 *   Cross (press)       -> FIRE: opens the control gate and runs the secondary
 *                          intake. If the flywheel isn't at speed yet, the shot is
 *                          armed and fires automatically once it is.
 *   Cross (press again) -> CEASE FIRE: feed off, gate closed. Flywheel keeps going.
 *   See ShooterSequence.java to tune TARGET_RPM / RPM_TOLERANCE.
 *
 * The driver collects pollen with the intake while driving; the second person
 * runs the shooter. If you only have one driver, change the "gamepad2"
 * references below to "gamepad1".
 *
 * To make this appear on the Driver Station OpMode list, this class is NOT marked
 * @Disabled. If you ever want to hide it, add @Disabled above the class.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp Drive", group = "Competition")
public class TeleOpDrive extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    // When NOT holding turbo, multiply all drive power by this so the robot is
    // easier to control. 0.6 = 60% speed. Tune to taste.
    private static final double NORMAL_SPEED = 0.6;

    // Flywheel runs continuously; the fire button only works the gate + feed.
    private ContinuousShooter shooter;
    // Remembers last loop's fire-button state so we only react to a NEW press
    // (the rising edge), not every loop the button is held down.
    private boolean previousFireButton = false;

    @Override
    public void runOpMode() {
        // Set up all the motors.
        robot.init(hardwareMap);
        shooter = new ContinuousShooter(robot);

        telemetry.addData("Status", "Initialized - press PLAY to drive");
        telemetry.update();

        // Wait here until the driver presses the PLAY (start) button.
        waitForStart();
        runtime.reset();

        // Spin the flywheel up right away; it stays running for the whole TeleOp.
        shooter.start();

        // Loop until the match ends or the driver presses STOP.
        while (opModeIsActive()) {

            // Read the joysticks.
            // Note: pushing a stick forward gives a NEGATIVE y value, so we flip it.
            double axial   = -gamepad1.left_stick_y;   // forward / back
            double lateral =  gamepad1.left_stick_x;    // strafe
            double yaw     =  gamepad1.right_stick_x;    // turn

            // Hold the right trigger for full speed; otherwise drive at NORMAL_SPEED.
            double speedScale = (gamepad1.right_trigger > 0.5) ? 1.0 : NORMAL_SPEED;

            robot.drive(axial * speedScale, lateral * speedScale, yaw * speedScale);

            // ---------- PRIMARY INTAKE (gamepad1 bumpers, hold to run) ----------
            // While the shooter is firing, the primary intake also runs IN to push
            // pollen through to the shooter. The driver's left bumper (eject)
            // still overrides it, e.g. to clear a jam.
            boolean firing = shooter.isFiring();
            if (gamepad1.left_bumper) {
                robot.intakeOut();
            } else if (gamepad1.right_bumper || firing) {
                robot.intakeIn();
            } else {
                robot.stopIntake();
            }

            // ---------- SHOOTER (gamepad2 Cross, press to fire / cease fire) ----------
            // The flywheel is always spinning. Cross opens the gate and runs the
            // feed intake (only once the flywheel is at speed); press again to stop.
            if (gamepad2.cross && !previousFireButton) {
                shooter.toggleFire();
            }
            previousFireButton = gamepad2.cross;

            // Track flywheel speed and fire an armed shot once at speed.
            shooter.update();

            // Show useful info on the Driver Station screen.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mode", (speedScale == 1.0) ? "TURBO" : "Normal");
            telemetry.addData("Axial / Lateral / Yaw", "%.2f / %.2f / %.2f",
                    axial, lateral, yaw);
            telemetry.addData("Shooter state", shooter.getState());
            telemetry.addData("Flywheel RPM", "%.0f / %.0f",
                    robot.getShooterRpm(), ShooterSequence.TARGET_RPM);
            telemetry.addData("Ready to fire", shooter.isAtSpeed() ? "YES" : "no");
            telemetry.update();
        }

        shooter.stop();
        robot.stop();
    }
}
