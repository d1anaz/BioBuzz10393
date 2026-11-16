package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "Test: Shooting While Moving", group = "Tests")
public class TestSWM extends OpMode {

    private Drivetrain drivetrain;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;
    private ShootingWhileMoving swm;
    private Follower follower;

    private boolean lastOptions = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(50, 20, Math.toRadians(113.5)));

        limelight = new Limelight(hardwareMap, false);
        drivetrain = new Drivetrain(hardwareMap, follower, false);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, limelight, false);
        swm = new ShootingWhileMoving(follower, shooter, turret, false);

        telemetry.addLine("SWM Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Options: Toggle SWM");
        telemetry.addLine("  Drive around shoot zones");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.manual(0, 0, 0);
        limelight.start();
        shooter.spin(); // Enable shooter for testing
    }

    @Override
    public void loop() {
        follower.update();

        // Toggle SWM
        if (gamepad1.options && !lastOptions) {
            swm.toggle();
            gamepad1.rumble(swm.isEnabled() ? 500 : 200);
        }
        lastOptions = gamepad1.options;

        // Drive
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        drivetrain.drive(forward, strafe, turn);

        // Update subsystems
        shooter.periodic();
        turret.setMode(Turret.Mode.ODOMETRY);
        turret.update(follower.pose());
        swm.update();

        Pose pose = follower.pose();
        Pose future = swm.getFuturePose();

        telemetry.addLine("╔═══ SWM TEST ═══╗");
        telemetry.addData("│ SWM Enabled", swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ H-Lock Active", swm.isHeadingLockActive() ? "YES" : "NO");
        telemetry.addLine("╠═══ CURRENT POSE ═══╣");
        telemetry.addData("│ X", "%.1f", pose.x());
        telemetry.addData("│ Y", "%.1f", pose.y());
        telemetry.addLine("╠═══ FUTURE POSE ═══╣");
        telemetry.addData("│ X", "%.1f", future.x());
        telemetry.addData("│ Y", "%.1f", future.y());
        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ At Speed", shooter.isAtSpeed() ? "YES" : "NO");
        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Aligned", turret.isAligned() ? "YES" : "NO");
        telemetry.addLine("╚═══════════════════╝");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        drivetrain.stop();
        limelight.stop();
    }
}