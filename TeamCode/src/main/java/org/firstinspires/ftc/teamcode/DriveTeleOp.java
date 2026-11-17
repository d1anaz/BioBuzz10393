package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Drive", group = "TeleOp")
public class DriveTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    // Pedro Pathing follower: uses the autotuned Pinpoint + heading values from pedro/Constants.java
    private Follower follower;

    // Heading hold: when the turn stick is released, the robot keeps pointing the same way
    // so strafing goes straight instead of curving. (Position hold was removed; it fought strafing.)
    private double targetHeading = 0;
    private boolean headingLocked = false;
    private static final double SETTLE_TURN_SPEED = 0.5; // rad/s; lock starts once the robot stops spinning
    // Turn power per radian of heading error. Too low = still curves; too high = robot wiggles.
    private static final double HEADING_KP = 1.5;
    // Max turn power heading hold may use
    private static final double MAX_HEADING_CORRECTION = 0.4;

    // Turning: the robot needs some power before it starts to rotate, so any stick push
    // starts at TURN_MIN power and rises to TURN_MAX at full stick.
    private static final double TURN_MIN = 0.2;  // raise if small stick pushes still don't turn
    private static final double TURN_MAX = 1.0;  // lower if full-stick turning is too fast

    // Driving speed for R2/L2 and the left stick: 1.0 = full speed, 0.5 = half speed
    private static final double DRIVE_SPEED = 0.7;//HERE SPEED YOU CAN CHANGE LATER GOES 0-1

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

        // Pedro sets up the drive motors and the Pinpoint from pedro/Constants.java
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0)); // robot must be still during init
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
        // Wheel test: hold a face button to spin ONE motor forward at 30%.
        // Runs before Pedro so Pedro doesn't fight the test.
        // Every wheel should roll toward the front of the robot.
        if (gamepad1.y || gamepad1.b || gamepad1.x || gamepad1.a) {
            follower.stop(); // Pedro lets go of the motors during the wheel test (no follower.update() this loop)
            frontLeft.setPower(gamepad1.y ? 0.3 : 0);  // Y / triangle
            frontRight.setPower(gamepad1.b ? 0.3 : 0); // B / circle
            backLeft.setPower(gamepad1.x ? 0.3 : 0);   // X / square
            backRight.setPower(gamepad1.a ? 0.3 : 0);  // A / cross
            telemetry.addData("Testing", gamepad1.y ? "frontLeft (RF)" : gamepad1.b ? "frontRight (RB)"
                    : gamepad1.x ? "backLeft (LB)" : "backRight (LF)");
            telemetry.update();
            return;
        }

        // Pedro directions: forward +, strafe left +, turn left (counter-clockwise) +
        double forward = (gamepad1.right_trigger - gamepad1.left_trigger) * DRIVE_SPEED; // R2 = forward, L2 = backward
        double strafe  = -gamepad1.left_stick_x * DRIVE_SPEED;  // left stick: push right = strafe right
        double turnIn  = -gamepad1.right_stick_x; // right stick: push right = turn right

        follower.update(); // reads the Pinpoint and drives the motors
        Pose pose = follower.pose();
        double heading = pose.heading();
        Velocity vel = follower.velocity();

        boolean turning = Math.abs(turnIn) > 0.05;
        String driveMode;
        double turn;

        if (turning) {
            // Driver is turning: boost so it starts turning right away
            turn = Math.signum(turnIn) * (TURN_MIN + (TURN_MAX - TURN_MIN) * (Math.abs(turnIn) - 0.05) / 0.95);
            headingLocked = false;
            driveMode = "TURNING";
        } else if (!headingLocked && Math.abs(vel.omega) > SETTLE_TURN_SPEED) {
            // Just let go of the turn stick and still spinning: let it stop before locking
            turn = 0;
            driveMode = "SETTLING";
        } else {
            // Not turning: lock this heading and gently correct any drift
            if (!headingLocked) {
                targetHeading = heading;
                headingLocked = true;
            }
            double error = Math.atan2(Math.sin(targetHeading - heading), Math.cos(targetHeading - heading));
            turn = Math.max(-MAX_HEADING_CORRECTION, Math.min(MAX_HEADING_CORRECTION, HEADING_KP * error));
            driveMode = "HEADING HOLD";
        }
        follower.manual(forward, strafe, turn);

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

        telemetry.addData("Drive", driveMode);
        telemetry.addData("Forward / Strafe / Turn", "%.2f / %.2f / %.2f", forward, strafe, turnIn);
        telemetry.addData("Intake", intakePower > 0 ? "FORWARD" : intakePower < 0 ? "REVERSE" : "OFF");
        telemetry.addData("Arm", armUp ? "UP" : "DOWN");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Target (deg)", "%.1f", Math.toDegrees(targetHeading));
        telemetry.addData("X / Y (in)", "%.1f / %.1f", pose.x(), pose.y());
        telemetry.update();
    }
    @Override
    public void stop() {
        follower.stop();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
    }
}
