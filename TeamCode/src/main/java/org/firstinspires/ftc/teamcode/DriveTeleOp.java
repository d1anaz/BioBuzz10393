package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive", group = "TeleOp")
public class DriveTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;

    private boolean intakeOn = false;
    private boolean lastDpadUp = false;

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
    }

    @Override
    public void loop() {
        double forward =  gamepad1.right_trigger - gamepad1.left_trigger; // R2 = forward, L2 = backward
        double strafe  =  gamepad1.left_stick_x * 1.1; // left stick right/right
        double turn    =  gamepad1.right_stick_x;        // right stick right/right

        // D-pad up toggles the intake on/off
        if (gamepad1.dpad_up && !lastDpadUp) intakeOn = !intakeOn;
        lastDpadUp = gamepad1.dpad_up;
        intake.setPower(intakeOn ? 0.9 : 0);

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
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
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
