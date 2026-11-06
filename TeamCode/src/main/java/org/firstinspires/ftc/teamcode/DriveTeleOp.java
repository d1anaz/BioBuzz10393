package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Drive", group = "TeleOp")
public class DriveTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private GoBildaPinpointDriver pinpoint;

    // Heading hold: when the turn stick is released, the Pinpoint keeps the robot pointed the same way
    private static final double HEADING_KP = 1.0; // raise if it still drifts, lower if it wobbles
    private double targetHeading = 0;

    private double intakePower = 0;  // 0.9 = in, -0.9 = out, 0 = off
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Arm servos (Expansion Hub servo ports 0 and 1)
    private Servo armRight, armLeft;
    // Each servo has its own down position because the horns don't line up the same.
    // Find these with the "Arm Servo Tuner" OpMode (tests/ArmServoTuner.java).
    // Down position (D-pad left, and where the arm goes when PLAY is pressed)
    private static final double ARM_RIGHT_DOWN = 0.125; // 0-1
    private static final double ARM_LEFT_DOWN  = 0.125; // 0-1
    // Up position (D-pad right)
    private static final double ARM_RIGHT_UP = 0.761; // 0-1
    private static final double ARM_LEFT_UP  = 0.761; // 0-1
    private boolean armUp = false;
    // Both servos move together over this many seconds so they stay level.
    // Must be slower than the slower servo. Lower = faster; raise if left still lags.
    private static final double ARM_MOVE_TIME = 0.8;
    private double armProgress = 0; // 0 = down, 1 = up
    private double lastArmTime = 0;

    @Override
    public void init() {//here maybe switch  some of lf and rbs to get it not to go right oct2026
        // Names must match the robot configuration on the Driver Station
        // Config names don't match the physical corners; mapped from the wheel test (Oct 2026)
        frontLeft  = hardwareMap.get(DcMotor.class, "RF"); // Control Hub port 0
        frontRight = hardwareMap.get(DcMotor.class, "RB"); // Control Hub port 1
        backLeft   = hardwareMap.get(DcMotor.class, "LB"); // Control Hub port 3
        backRight  = hardwareMap.get(DcMotor.class, "LF"); // Control Hub port 2

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake"); // Expansion Hub
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRight = hardwareMap.get(Servo.class, "armRight"); // Expansion Hub servo port 0
        armLeft  = hardwareMap.get(Servo.class, "armLeft");  // Expansion Hub servo port 1
        // The two servos face each other, so one is reversed to make both lift the arm together
        armRight.setDirection(Servo.Direction.FORWARD);
        armLeft.setDirection(Servo.Direction.REVERSE); // mirrored side: must be opposite of armRight

        // Name must match the I2C device in the robot configuration
        // Offsets and directions match pedroPathing/Constants.java
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-0.245, -3.697, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // robot must be still during init
    }

    @Override
    public void start() {
        // Move the arm to its down position when PLAY is pressed (not during INIT)
        lastArmTime = getRuntime();
        setArm();
    }

    private void setArm() {
        // Step progress toward the target, then send both servos the same fraction of their travel
        double now = getRuntime();
        double step = (now - lastArmTime) / ARM_MOVE_TIME;
        lastArmTime = now;
        double target = armUp ? 1 : 0;
        if (armProgress < target) armProgress = Math.min(target, armProgress + step);
        else                      armProgress = Math.max(target, armProgress - step);

        armRight.setPosition(ARM_RIGHT_DOWN + (ARM_RIGHT_UP - ARM_RIGHT_DOWN) * armProgress);
        armLeft.setPosition(ARM_LEFT_DOWN + (ARM_LEFT_UP - ARM_LEFT_DOWN) * armProgress);
    }

    @Override
    public void loop() {
        double forward =  gamepad1.right_trigger - gamepad1.left_trigger; // R2 = forward, L2 = backward
        double strafe  = -gamepad1.left_stick_x * 1.1; // left stick left/right (negated: was inverted)
        double turn    =  gamepad1.right_stick_x;        // right stick right/right

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);

        if (Math.abs(turn) > 0.05) {
            // Driver is turning: follow them and remember the new direction
            targetHeading = heading;
        } else if (Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05) {
            // Driving without turning: push back toward the remembered direction
            double error = Math.atan2(Math.sin(targetHeading - heading), Math.cos(targetHeading - heading));
            turn = -error * HEADING_KP; // flip to +error if it spins away instead of correcting
        } else {
            targetHeading = heading;
        }

        // D-pad up = intake forward, D-pad down = reverse; press the same button again to stop
        if (gamepad1.dpad_up && !lastDpadUp)     intakePower = (intakePower == 0.9) ? 0 : 0.9;
        if (gamepad1.dpad_down && !lastDpadDown) intakePower = (intakePower == -0.9) ? 0 : -0.9;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        intake.setPower(intakePower);

        // D-pad right = arm to up position, D-pad left = arm back down
        if (gamepad1.dpad_right) armUp = true;
        if (gamepad1.dpad_left)  armUp = false;
        setArm();

        // Wheel test: hold a face button to spin ONE motor forward at 30%.
        // Every wheel should roll toward the front of the robot.
        if (gamepad1.y || gamepad1.b || gamepad1.x || gamepad1.a) {
            frontLeft.setPower(gamepad1.y ? 0.3 : 0);  // Y / triangle
            frontRight.setPower(gamepad1.b ? 0.3 : 0); // B / circle
            backLeft.setPower(gamepad1.x ? 0.3 : 0);   // X / square
            backRight.setPower(gamepad1.a ? 0.3 : 0);  // A / cross
            telemetry.addData("Testing", gamepad1.y ? "frontLeft (RF)" : gamepad1.b ? "frontRight (RB)"
                    : gamepad1.x ? "backLeft (LB)" : "backRight (LF)");
            telemetry.update();
            return;
        }

        // Scale so no motor power goes above 1
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        frontLeft.setPower((forward + strafe + turn) / max);
        backLeft.setPower((forward - strafe + turn) / max);
        frontRight.setPower((forward - strafe - turn) / max);
        backRight.setPower((forward + strafe - turn) / max);

        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.addData("Intake", intakePower > 0 ? "FORWARD" : intakePower < 0 ? "REVERSE" : "OFF");
        telemetry.addData("Arm", armUp ? "UP" : "DOWN");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Target (deg)", "%.1f", Math.toDegrees(targetHeading));
        telemetry.addData("X / Y (in)", "%.1f / %.1f", pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.update();
    }
    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
    }
}
