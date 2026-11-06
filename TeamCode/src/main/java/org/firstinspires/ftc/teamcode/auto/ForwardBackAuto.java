package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Forward/Back 2ft", group = "Test")
public class ForwardBackAuto extends LinearOpMode {

    // Change these to match your motors and wheels
    static final double TICKS_PER_REV  = 537.7; // goBILDA 312 RPM Yellow Jacket
    static final double WHEEL_DIAMETER = 4.094; // inches (goBILDA 104mm mecanum)
    static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);

    static final double DRIVE_POWER = 0.4;
    static final double TIMEOUT_SEC = 4.0; // safety stop if a move never finishes

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;

    @Override
    public void runOpMode() {
        // Same mapping as DriveTeleOp: config names don't match the physical corners
        frontLeft  = hardwareMap.get(DcMotor.class, "RF");
        frontRight = hardwareMap.get(DcMotor.class, "RB");
        backLeft   = hardwareMap.get(DcMotor.class, "LB");
        backRight  = hardwareMap.get(DcMotor.class, "LF");
        intake     = hardwareMap.get(DcMotor.class, "intake");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Status", "Ready - forward 24in, then back 24in");
        telemetry.update();
        waitForStart();

        driveInches(24);   // forward 2 feet
        sleep(500);
        driveInches(-24);  // back 2 feet
    }

    /** Drives straight; positive = forward, negative = backward. */
    private void driveInches(double inches) {
        int move = (int) Math.round(inches * TICKS_PER_INCH);
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};

        for (DcMotor m : motors) {
            m.setTargetPosition(m.getCurrentPosition() + move);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(DRIVE_POWER);
        }

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < TIMEOUT_SEC
                && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Target", "%.1f in", inches);
            telemetry.addData("FL / FR", "%d / %d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.addData("BL / BR", "%d / %d", backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
        }

        for (DcMotor m : motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
