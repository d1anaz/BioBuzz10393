package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive", group = "TeleOp")
public class DriveTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        // Names must match the robot configuration on the Driver Station
        frontLeft  = hardwareMap.get(DcMotor.class, "LF"); // Control Hub port 2
        frontRight = hardwareMap.get(DcMotor.class, "RF"); // Control Hub port 0
        backLeft   = hardwareMap.get(DcMotor.class, "LB"); // Control Hub port 3
        backRight  = hardwareMap.get(DcMotor.class, "RB"); // Control Hub port 1

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;     // left stick up/down
        double strafe  =  gamepad1.left_stick_x * 1.1; // left stick left/right
        double turn    =  gamepad1.right_stick_x;    // right stick left/right

        // Scale so no motor power goes above 1
        double max = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        frontLeft.setPower((forward + strafe + turn) / max);
        backLeft.setPower((forward - strafe + turn) / max);
        frontRight.setPower((forward - strafe - turn) / max);
        backRight.setPower((forward + strafe - turn) / max);

        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
