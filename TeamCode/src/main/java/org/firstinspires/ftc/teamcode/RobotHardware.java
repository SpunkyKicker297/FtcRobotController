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
 * In addition to the drivetrain, this robot has two mechanism motors wired to a
 * REV Expansion Hub:
 *   - an INTAKE motor  (pulls game elements into the robot)
 *   - a  SHOOTER motor (launches/scores game elements)
 *
 * IMPORTANT: The motor names below ("front_left_drive", "intake", etc.) MUST
 * exactly match the names you type into the Robot Configuration on the Driver
 * Station / Robot Controller phone. If they don't match, the app will crash on
 * INIT with a "cannot find a hardware device" error. The drive motors live on
 * the Control Hub; the intake and shooter live on the Expansion Hub -- but in
 * code you still just look them up by name, the SDK handles which hub they're on.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    // The four drive motors.
    private DcMotor frontLeftDrive  = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive  = null;

    // Mechanism motors on the REV Expansion Hub.
    private DcMotor intakeMotor  = null;
    private DcMotor intake2Motor = null;   // secondary intake: feeds pollen into the shooter
    // The shooter is a DcMotorEx so we can read its velocity from the encoder.
    private DcMotorEx shooterMotor = null;

    // Servo that opens/closes the gate letting pollen into the flywheel.
    private Servo controlGate = null;

    // The REV Control Hub has a built-in IMU (orientation sensor). We use it to
    // make accurate turns (e.g. exactly 90 degrees).
    private IMU imu = null;

    // Default power levels for the mechanisms. Tune these for your robot.
    // Positive intake power should pull game elements IN; flip the sign (or the
    // motor direction below) if it spits them out instead.
    public static final double INTAKE_POWER  = 0.35;
    public static final double SHOOTER_POWER = 0.52;
    public static final double INTAKE2_POWER = 0.6;   // secondary (feed) intake

    // ------------------------------------------------------------------
    //  Shooter flywheel speed  --  TUNE THESE
    // ------------------------------------------------------------------
    // Encoder ticks per flywheel revolution. Depends on your motor:
    //   goBILDA 6000 RPM Yellow Jacket (1:1)  -> 28
    //   goBILDA 5202 19.2:1                   -> 537.7
    // Check your motor's spec sheet. A wrong value makes the RPM reading wrong.
    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    // ------------------------------------------------------------------
    //  Control gate servo  --  TUNE THESE
    // ------------------------------------------------------------------
    // Total rotation the servo covers from position 0.0 to 1.0. goBILDA torque /
    // speed servos are commonly 300 degrees; some are 180 or 270. Check yours.
    public static final double SERVO_RANGE_DEGREES = 300.0;
    // How far the gate swings between closed and open (15-20 degrees).
    public static final double GATE_TRAVEL_DEGREES = 18.0;
    // Servo positions can only be 0.0 to 1.0 (no negative values), so we place the
    // OPEN position slightly above 0.0 and put CLOSED one full travel beyond it.
    // On this robot a HIGHER servo value LOWERS the gate.
    //
    // Servo position (0.0-1.0) when the gate is OPEN. Adjust to your mounting.
    public static final double GATE_OPEN_POSITION = 0.06;
    // Servo position when the gate is CLOSED: GATE_TRAVEL_DEGREES further from open.
    // Opening the gate moves it back exactly GATE_TRAVEL_DEGREES.
    public static final double GATE_CLOSED_POSITION =
            GATE_OPEN_POSITION + (GATE_TRAVEL_DEGREES / SERVO_RANGE_DEGREES);

    // ------------------------------------------------------------------
    //  Encoder / drivetrain geometry  --  TUNE THESE FOR YOUR ROBOT
    // ------------------------------------------------------------------
    // How many encoder "ticks" the motor reports per one full output-shaft
    // revolution. This depends on your goBILDA motor. Common values:
    //   goBILDA 5202/5203 Yellow Jacket 19.2:1 -> 537.7
    //   goBILDA 5202/5203 Yellow Jacket 13.7:1 -> 383.6
    //   goBILDA 5202/5203 Yellow Jacket 26.9:1 -> 753.2
    // Check the sticker on your motor / goBILDA's site and set the right one.
    public static final double TICKS_PER_MOTOR_REV = 537.7;

    // Extra gearing between the motor and the wheel. 1.0 means the wheel is
    // driven directly by the motor's output shaft (most common).
    public static final double DRIVE_GEAR_REDUCTION = 1.0;

    // Wheel diameter in inches. goBILDA mecanum wheels are 96mm = ~3.78 in.
    public static final double WHEEL_DIAMETER_INCHES = 3.78;

    // Derived: how many encoder ticks equal one inch of forward travel.
    public static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

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

        // ----- Mechanism motors on the REV Expansion Hub -----
        // These names must match your Robot Configuration (Expansion Hub ports).
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake_1");
        intake2Motor = hardwareMap.get(DcMotor.class, "intake_2");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_1");
        controlGate  = hardwareMap.get(Servo.class, "control_gate_1");

        // Set directions so positive power does the "useful" thing. Flip any of
        // these to REVERSE if the motor runs the wrong way during testing.
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intake2Motor.setDirection(DcMotor.Direction.FORWARD);
        // REVERSE = correct shooting direction on this robot (FORWARD ran it backward).
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // The shooter now has an encoder cable, so run it in closed-loop velocity
        // mode. The SDK then adjusts power to hold the requested speed, which
        // keeps shots consistent as the battery drains or pollen loads the wheel.
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start with the gate closed.
        controlGate.setPosition(GATE_CLOSED_POSITION);

        // Intake: BRAKE so game elements don't slip back out when it stops.
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Shooter: FLOAT (coast). A heavy spinning flywheel shouldn't be hard-braked.
        // If your shooter isn't a flywheel, BRAKE may be fine instead.
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ----- IMU (built into the REV Control Hub) -----
        // The IMU must know how the Control Hub is physically mounted so it can
        // report the correct heading. The defaults below assume the hub is lying
        // flat (logo facing UP) with the USB ports pointing toward the FRONT of
        // the robot. If your hub is mounted differently, change these two
        // directions -- see the REV / FTC docs on "Universal IMU".
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParameters);
        imu.resetYaw();

        // Start stopped.
        stop();
    }

    // ------------------------------------------------------------------
    //  Sensors / encoders
    // ------------------------------------------------------------------

    /** Current robot heading in degrees. Positive = counter-clockwise (left turn). */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Reset the IMU heading so "straight ahead" becomes 0 degrees. */
    public void resetHeading() {
        imu.resetYaw();
    }

    /** Switch all four drive motors into the given RunMode. */
    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    /** Average absolute encoder position across the four drive motors (ticks). */
    public double getAverageDriveTicks() {
        return (Math.abs(frontLeftDrive.getCurrentPosition())
                + Math.abs(backLeftDrive.getCurrentPosition())
                + Math.abs(frontRightDrive.getCurrentPosition())
                + Math.abs(backRightDrive.getCurrentPosition())) / 4.0;
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

    // ------------------------------------------------------------------
    //  Intake controls
    // ------------------------------------------------------------------

    /** Run the intake at an arbitrary power (-1.0 to 1.0). */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    /** Run the intake inward at the default power. */
    public void intakeIn() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    /** Run the intake outward (eject) at the default power. */
    public void intakeOut() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    /** Stop the intake motor. */
    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    // ------------------------------------------------------------------
    //  Shooter controls
    // ------------------------------------------------------------------

    /** Run the shooter at an arbitrary power (-1.0 to 1.0). */
    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
    }

    /** Spin the shooter up to the default firing power. */
    public void shooterOn() {
        shooterMotor.setPower(SHOOTER_POWER);
    }

    /** Stop the shooter motor. */
    public void shooterOff() {
        shooterMotor.setPower(0);
    }

    /** Spin the shooter flywheel to a target speed in RPM (closed-loop). */
    public void setShooterRpm(double rpm) {
        shooterMotor.setVelocity(rpm * SHOOTER_TICKS_PER_REV / 60.0);
    }

    /** Current shooter flywheel speed in RPM, measured from the encoder. */
    public double getShooterRpm() {
        return shooterMotor.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    // ------------------------------------------------------------------
    //  Secondary intake (feeds the shooter) and control gate
    // ------------------------------------------------------------------

    /** Run the secondary intake to push pollen into the shooter. */
    public void feedOn() {
        intake2Motor.setPower(INTAKE2_POWER);
    }

    /** Stop the secondary intake. */
    public void feedOff() {
        intake2Motor.setPower(0);
    }

    /** Run the secondary intake at an arbitrary power (-1.0 to 1.0). */
    public void setIntake2Power(double power) {
        intake2Motor.setPower(power);
    }

    /** Move the control gate to any servo position (clamped to 0.0 - 1.0). Handy for tuning. */
    public void setGatePosition(double position) {
        controlGate.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    /** The last position commanded to the control gate servo (0.0 - 1.0). */
    public double getGatePosition() {
        return controlGate.getPosition();
    }

    /** Open the control gate so pollen can reach the flywheel. */
    public void openGate() {
        controlGate.setPosition(GATE_OPEN_POSITION);
    }

    /** Close the control gate. */
    public void closeGate() {
        controlGate.setPosition(GATE_CLOSED_POSITION);
    }

    /** Stop all motors: drivetrain, intakes, and shooter. */
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
        if (intakeMotor != null)  intakeMotor.setPower(0);
        if (intake2Motor != null) intake2Motor.setPower(0);
        if (shooterMotor != null) shooterMotor.setPower(0);
    }
}
