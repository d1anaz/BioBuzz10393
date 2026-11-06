package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Finds the "down" position for each arm servo separately.
 * Left stick up/down  = move armRight
 * Right stick up/down = move armLeft
 * When both sides sit level in the down spot, copy the numbers into
 * ARM_RIGHT_DOWN and ARM_LEFT_DOWN in DriveTeleOp.java.
 */
@TeleOp(name = "Arm Servo Tuner", group = "Testing")
public class ArmServoTuner extends LinearOpMode {

    private static final double STEP = 0.002; // small step per loop for fine control

    @Override
    public void runOpMode() {
        Servo armRight = hardwareMap.get(Servo.class, "armRight");
        Servo armLeft  = hardwareMap.get(Servo.class, "armLeft");
        // Must match the directions in DriveTeleOp.java
        armRight.setDirection(Servo.Direction.FORWARD);
        armLeft.setDirection(Servo.Direction.REVERSE);

        double rightPos = 0.5;
        double leftPos  = 0.5;

        telemetry.addLine("Press PLAY. Left stick = armRight, right stick = armLeft");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_y) > 0.1)  rightPos -= gamepad1.left_stick_y * STEP;
            if (Math.abs(gamepad1.right_stick_y) > 0.1) leftPos  -= gamepad1.right_stick_y * STEP;
            rightPos = Math.max(0, Math.min(1, rightPos));
            leftPos  = Math.max(0, Math.min(1, leftPos));

            armRight.setPosition(rightPos);
            armLeft.setPosition(leftPos);

            telemetry.addData("ARM_RIGHT_DOWN", "%.3f", rightPos);
            telemetry.addData("ARM_LEFT_DOWN", "%.3f", leftPos);
            telemetry.addLine("Left stick = armRight, right stick = armLeft");
            telemetry.update();
        }
    }
}
