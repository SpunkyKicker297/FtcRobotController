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
 *   A button (press)    -> start/stop the SHOOTER SEQUENCE. First press: flywheel
 *                          spins up to target RPM, then the control gate opens and
 *                          the secondary intake feeds pollen. Second press runs it
 *                          in reverse (feed off, gate closed, flywheel off).
 *                          See ShooterSequence.java to tune the target RPM.
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

    // Runs the shooter startup/shutdown sequence (created after robot.init()).
    private ShooterSequence shooter;
    // Remembers last loop's A-button state so we only react to a NEW press
    // (the rising edge), not every loop the button is held down.
    private boolean previousAButton = false;

    @Override
    public void runOpMode() {
        // Set up all the motors.
        robot.init(hardwareMap);
        shooter = new ShooterSequence(robot);

        telemetry.addData("Status", "Initialized - press PLAY to drive");
        telemetry.update();

        // Wait here until the driver presses the PLAY (start) button.
        waitForStart();
        runtime.reset();

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
            if (gamepad1.right_bumper) {
                robot.intakeIn();
            } else if (gamepad1.left_bumper) {
                robot.intakeOut();
            } else {
                robot.stopIntake();
            }

            // ---------- SHOOTER (gamepad2 A, press to start/stop the sequence) ----------
            // Edge detection: only react when A goes from up to down.
            if (gamepad2.a && !previousAButton) {
                shooter.toggle();
            }
            previousAButton = gamepad2.a;

            // Advance the startup sequence (spin up -> open gate -> feed).
            shooter.update();

            // Show useful info on the Driver Station screen.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mode", (speedScale == 1.0) ? "TURBO" : "Normal");
            telemetry.addData("Axial / Lateral / Yaw", "%.2f / %.2f / %.2f",
                    axial, lateral, yaw);
            telemetry.addData("Shooter state", shooter.getState());
            telemetry.addData("Shooter RPM", "%.0f / %.0f",
                    robot.getShooterRpm(), ShooterSequence.TARGET_RPM);
            telemetry.update();
        }

        shooter.forceStop();
        robot.stop();
    }
}
