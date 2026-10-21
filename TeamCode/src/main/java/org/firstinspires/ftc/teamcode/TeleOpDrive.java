/*
 * TeleOpDrive.java
 *
 * This is your DRIVER-CONTROLLED OpMode. During the "TeleOp" period of a match,
 * this reads the gamepad (your goBILDA controller plugged into the Driver Station)
 * and drives the robot.
 *
 * Controls (gamepad1 = first controller):
 *   Left stick  Y  -> drive forward / backward
 *   Left stick  X  -> strafe left / right
 *   Right stick X  -> turn (yaw) left / right
 *   Right trigger  -> hold for "turbo" (full speed); otherwise speed is reduced
 *                     for easier, more precise control (great for new drivers).
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

    @Override
    public void runOpMode() {
        // Set up all the motors.
        robot.init(hardwareMap);

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

            // Show useful info on the Driver Station screen.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mode", (speedScale == 1.0) ? "TURBO" : "Normal");
            telemetry.addData("Axial / Lateral / Yaw", "%.2f / %.2f / %.2f",
                    axial, lateral, yaw);
            telemetry.update();
        }

        robot.stop();
    }
}
