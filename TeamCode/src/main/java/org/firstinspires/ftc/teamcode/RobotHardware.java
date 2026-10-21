/*
 * RobotHardware.java
 *
 * A shared "hardware class" for your robot. Instead of copy/pasting the same
 * motor setup into every OpMode, both your TeleOp and Autonomous programs
 * create one of these and call its helper methods.
 *
 * This assumes a 4-wheel mecanum (omni-directional) drive base. If your robot
 * uses a simple 2-motor drive, you can delete the two "right"/"left" extras and
 * simplify, but mecanum is by far the most common FTC starter drivetrain.
 *
 * IMPORTANT: The motor names below ("front_left_drive", etc.) MUST exactly match
 * the names you type into the Robot Configuration on the Driver Station / Robot
 * Controller phone. If they don't match, the app will crash on INIT with a
 * "cannot find a hardware device" error.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

    // The four drive motors.
    private DcMotor frontLeftDrive  = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive  = null;

    /**
     * Call this once from runOpMode() (before waitForStart) to grab all the
     * hardware from the configuration and get it ready to use.
     *
     * @param hardwareMap the hardwareMap object from your OpMode
     */
    public void init(HardwareMap hardwareMap) {
        // Look up each motor by the name set in the Robot Configuration.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Most robots need one side reversed so that positive power drives forward.
        // If your robot drives backward on the first test, flip these directions.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake (instead of coast) when power is zero. This makes the robot stop
        // more precisely, which is especially helpful during autonomous.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start stopped.
        stop();
    }

    /**
     * Drive the mecanum base using three motion axes. This is the math that lets
     * a mecanum robot move in any direction and spin at the same time.
     *
     * @param axial   forward (+) / backward (-)
     * @param lateral strafe right (+) / left (-)
     * @param yaw     turn clockwise (+) / counter-clockwise (-)
     */
    public void drive(double axial, double lateral, double yaw) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize: if any wheel is asked for more than 100% power, scale them all
        // down proportionally so the robot still moves in the intended direction.
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /** Set each wheel's power directly (handy for autonomous moves). */
    public void setMotorPowers(double frontLeft, double frontRight,
                               double backLeft, double backRight) {
        frontLeftDrive.setPower(frontLeft);
        frontRightDrive.setPower(frontRight);
        backLeftDrive.setPower(backLeft);
        backRightDrive.setPower(backRight);
    }

    /** Stop all drive motors. */
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }
}
