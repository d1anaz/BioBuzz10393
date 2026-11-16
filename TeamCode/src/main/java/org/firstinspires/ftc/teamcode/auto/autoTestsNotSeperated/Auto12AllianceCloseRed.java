package org.firstinspires.ftc.teamcode.auto.autoTestsNotSeperated;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Auto12AllianceCloseRed — 12-ball alliance close-side autonomous, RED.
 * Starts at (28.211, 132.895, 143deg). 3 intake/shoot cycles each preceded by a gate open.
 * NOTE: Doc 6 had duplicate Java field names "gateopen" — renamed gateopen1/2/3 here.
 */
@Autonomous(name = "PDxRD 12 ball close red (goal)", group = "PDxRD")
@Configurable
public class Auto12AllianceCloseRed extends OpMode {

    private static final boolean IS_RED = true;

    private static final double GOAL_X = 144.0;
    private static final double GOAL_Y  = 144.0;

    // =========================================================================
    //  TURRET CONSTANTS
    // =========================================================================
    private static final double TURRET_MAX_ANGLE       = 50.0;
    private static final double TURRET_CENTER_POS      = 0.5;
    private static final double TURRET_ALIGN_TOLERANCE = 2.0;
    private static final double TURRET_P               = 0.012;
    private static final double TURRET_D               = 0.001;

    // =========================================================================
    //  SHOOTER RPMs
    // =========================================================================
    private static final double SHOOT_0_RPM = 3150.0;   // close preset
    private static final double SHOOT_1_RPM = 3150.0;
    private static final double SHOOT_2_RPM = 3150.0;
    private static final double SHOOT_3_RPM = 3150.0;

    // =========================================================================
    //  TIMING CONSTANTS
    // =========================================================================
    private static final double INITIAL_SPINUP_S    = 1.0;
    private static final double SPINUP_MS            = 50.0;
    private static final double SHOOT_MS             = 1500.0;
    /** Max time to wait for alignment before shooting anyway. */
    private static final double ALIGN_TIMEOUT_S        = 1.0;
    private static final double INTAKE_END_DWELL_MS  = 120.0;
    /** Pause after gate-open path completes before sweeping inward. */
    private static final double GATE_OPEN_DWELL_MS   = 120.0;

    // =========================================================================
    //  PATH SPEEDS
    // =========================================================================
    private static final double SHOOT_PATH_SPEED    = 0.9;
    private static final double INTAKE_PATH_SPEED   = 0.85;
    private static final double GATE_OPEN_SPEED     = 0.8;   // slower for gate push

    // =========================================================================
    //  PER-PATH TIMEOUTS (seconds)
    // =========================================================================
    private static final double T_SHOOT0    = 4.0;
    private static final double T_INTAKE1   = 3.5;
    private static final double T_GATEOPEN1 = 2.0;
    private static final double T_SHOOT1    = 3.0;
    private static final double T_INTAKE2   = 4.0;
    private static final double T_GATEOPEN2 = 2.0;
    private static final double T_SHOOT2    = 3.0;
    private static final double T_INTAKE3   = 4.0;
    private static final double T_GATEOPEN3 = 2.0;
    private static final double T_SHOOT3    = 3.0;

    // =========================================================================
    //  HARDWARE
    // =========================================================================
    private TelemetryManager panelsTelemetry;
    private Follower         follower;
    private Paths            paths;
    private Intake           intake;
    private Shooter          shooter;
    private Turret           turret;
    private Limelight        limelight;
    private Lights           lights;

    // =========================================================================
    //  TIMERS
    // =========================================================================
    private final ElapsedTime pathTimer      = new ElapsedTime();
    private final ElapsedTime alignTimer    = new ElapsedTime();
    private final ElapsedTime spinupTimer    = new ElapsedTime();
    private final ElapsedTime shootTimer     = new ElapsedTime();
    private final ElapsedTime intakeEndTimer = new ElapsedTime();
    private final ElapsedTime gateOpenTimer  = new ElapsedTime();

    // =========================================================================
    //  TURRET PD STATE
    // =========================================================================
    private double            turretLastError = 0.0;
    private final ElapsedTime turretPidTimer  = new ElapsedTime();
    private double            turretServoPos  = TURRET_CENTER_POS;

    private double currentPathTimeout = 4.0;

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================
    private enum AutoState {
        INITIAL_SPINUP,
        // Preload shoot
        SHOOT0_PATH, SHOOT0_SPINUP, SHOOT0_FIRING,
        // Cycle 1: intake → gate → shoot
        INTAKE1_PATH, INTAKE1_DWELL,
        GATEOPEN1_PATH, GATEOPEN1_DWELL,
        SHOOT1_PATH, SHOOT1_SPINUP, SHOOT1_FIRING,
        // Cycle 2
        INTAKE2_PATH, INTAKE2_DWELL,
        GATEOPEN2_PATH, GATEOPEN2_DWELL,
        SHOOT2_PATH, SHOOT2_SPINUP, SHOOT2_FIRING,
        // Cycle 3
        INTAKE3_PATH, INTAKE3_DWELL,
        GATEOPEN3_PATH, GATEOPEN3_DWELL,
        SHOOT3_PATH, SHOOT3_SPINUP, SHOOT3_FIRING,
        DONE
    }
    private AutoState state = AutoState.INITIAL_SPINUP;

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(115.789, 132.895, Math.toRadians(37)));
        limelight = new Limelight(hardwareMap, IS_RED);
        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        turret    = new Turret(hardwareMap, limelight, IS_RED);
        lights    = new Lights(hardwareMap);
        paths     = new Paths(follower);
        panelsTelemetry.debug("Status", "Auto 12 Alliance Close Red - Ready");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.spin();
        shooter.setTargetRPM(SHOOT_0_RPM);
        shooter.setCloseMode();
        intake.setMode(Intake.Mode.INTAKE);
        turretServoPos  = TURRET_CENTER_POS;
        turretLastError = 0.0;
        turretPidTimer.reset();
        spinupTimer.reset();
        state = AutoState.INITIAL_SPINUP;
        AutoToTeleTransfer.finalPose = follower.pose();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        turret.update(follower.pose());
        updateTurretOdom();
        lights.update(shooter.isActive(), shooter.getRPMMode(), isAligned());
        autonomousUpdate();
        AutoToTeleTransfer.finalPose = follower.pose();
        panelsTelemetry.debug("State",        state.name());
        panelsTelemetry.debug("T",            String.format(Locale.US, "%.3f", follower.parametricCompletion()));
        panelsTelemetry.debug("PathTimer",    String.format(Locale.US, "%.2f / %.1f", pathTimer.seconds(), currentPathTimeout));
        panelsTelemetry.debug("Shoot",        String.format(Locale.US, "%.0f / %.0f ms", shootTimer.milliseconds(), SHOOT_MS));
        panelsTelemetry.debug("GateTimer ms", String.format(Locale.US, "%.0f / %.0f", gateOpenTimer.milliseconds(), GATE_OPEN_DWELL_MS));
        panelsTelemetry.debug("RPM Tgt",      shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",     shooter.getReadRPM());
        panelsTelemetry.debug("AtSpeed",      shooter.isAtSpeed());
        panelsTelemetry.debug("Intake",       intake.getCurrentMode().toString());
        panelsTelemetry.debug("TurretSrv",    String.format(Locale.US, "%.3f", turretServoPos));
        panelsTelemetry.debug("Aligned",      isTurretAligned());
        panelsTelemetry.debug("X",            follower.pose().x());
        panelsTelemetry.debug("Y",            follower.pose().y());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.stop(); intake.stop(); lights.off();
        AutoToTeleTransfer.finalPose = follower.pose();
    }

    // =========================================================================
    //  TURRET ODOM TRACKING
    // =========================================================================
    private void updateTurretOdom() {
        Pose p = follower.pose();
        double angle = Math.toDegrees(Math.atan2(GOAL_Y - p.y(), GOAL_X - p.x()))
                - Math.toDegrees(p.heading());
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        angle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
        double current    = servoToAngle(turretServoPos);
        double error      = angle - current;
        double dt         = turretPidTimer.seconds();
        double derivative = (dt > 0.001) ? (error - turretLastError) / dt : 0.0;
        turretServoPos    = Math.max(0.0, Math.min(1.0,
                turretServoPos + error * TURRET_P + derivative * TURRET_D));
        turretLastError   = error;
        turretPidTimer.reset();
        turret.setManualAngle(servoToAngle(turretServoPos));
        turret.setMode(Turret.Mode.ODOMETRY);
    }

    private double servoToAngle(double pos) {
        return (pos - TURRET_CENTER_POS) / 0.5 * TURRET_MAX_ANGLE;
    }

    private boolean isTurretAligned() {
        Pose p = follower.pose();
        double rel = Math.toDegrees(Math.atan2(GOAL_Y - p.y(), GOAL_X - p.x()))
                - Math.toDegrees(p.heading());
        while (rel >  180) rel -= 360;
        while (rel < -180) rel += 360;
        double desired = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, rel));
        return Math.abs(desired - servoToAngle(turretServoPos)) < TURRET_ALIGN_TOLERANCE;
    }

    // =========================================================================
    //  PATH HELPERS
    // =========================================================================
    private void followShoot(Path p, double t)    { Constants.setMaxPathPower(SHOOT_PATH_SPEED);  follower.holdEnd.set(true); follower.follow(p);  pathTimer.reset(); currentPathTimeout = t; }
    private void followIntake(Path p, double t)   { Constants.setMaxPathPower(INTAKE_PATH_SPEED); follower.holdEnd.set(false); follower.follow(p); pathTimer.reset(); currentPathTimeout = t; }
    private void followGateOpen(Path p, double t) { Constants.setMaxPathPower(GATE_OPEN_SPEED);   follower.holdEnd.set(false); follower.follow(p); pathTimer.reset(); currentPathTimeout = t; }

    private boolean pathDone()      { return follower.parametricCompletion() >= 0.95 || !follower.isBusy() || pathTimer.seconds() >= currentPathTimeout; }
    private boolean shootPathDone() { return !follower.isBusy() || pathTimer.seconds() >= currentPathTimeout; }

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================

    private boolean isAligned() {
        return isTurretAligned() || shooter.isAtSpeed();
    }
    private void autonomousUpdate() {
        switch (state) {

            case INITIAL_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= INITIAL_SPINUP_S) {
                    followShoot(paths.shoot0, T_SHOOT0); state = AutoState.SHOOT0_PATH;
                }
                break;

            case SHOOT0_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT0_SPINUP; }
                break;

            case SHOOT0_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT0_FIRING;
                }
                break;

            case SHOOT0_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_1_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    followIntake(paths.intake1, T_INTAKE1);
                    state = AutoState.INTAKE1_PATH;
                }
                break;

            // ── CYCLE 1 ──────────────────────────────────────────────────────
            case INTAKE1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { intakeEndTimer.reset(); state = AutoState.INTAKE1_DWELL; }
                break;

            case INTAKE1_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    followGateOpen(paths.gateopen1, T_GATEOPEN1); state = AutoState.GATEOPEN1_PATH;
                }
                break;

            case GATEOPEN1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.stop();
                    gateOpenTimer.reset();
                    state = AutoState.GATEOPEN1_DWELL;
                }
                break;

            case GATEOPEN1_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenTimer.milliseconds() >= GATE_OPEN_DWELL_MS) {
                    followShoot(paths.shoot1, T_SHOOT1); state = AutoState.SHOOT1_PATH;
                }
                break;

            case SHOOT1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT1_SPINUP; }
                break;

            case SHOOT1_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT1_FIRING;
                }
                break;

            case SHOOT1_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_2_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    followIntake(paths.intake2, T_INTAKE2);
                    state = AutoState.INTAKE2_PATH;
                }
                break;

            // ── CYCLE 2 ──────────────────────────────────────────────────────
            case INTAKE2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { intakeEndTimer.reset(); state = AutoState.INTAKE2_DWELL; }
                break;

            case INTAKE2_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    followGateOpen(paths.gateopen2, T_GATEOPEN2); state = AutoState.GATEOPEN2_PATH;
                }
                break;

            case GATEOPEN2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.stop();
                    gateOpenTimer.reset();
                    state = AutoState.GATEOPEN2_DWELL;
                }
                break;

            case GATEOPEN2_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenTimer.milliseconds() >= GATE_OPEN_DWELL_MS) {
                    followShoot(paths.shoot2, T_SHOOT2); state = AutoState.SHOOT2_PATH;
                }
                break;

            case SHOOT2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT2_SPINUP; }
                break;

            case SHOOT2_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT2_FIRING;
                }
                break;

            case SHOOT2_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_3_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    followIntake(paths.intake3, T_INTAKE3);
                    state = AutoState.INTAKE3_PATH;
                }
                break;

            // ── CYCLE 3 ──────────────────────────────────────────────────────
            case INTAKE3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { intakeEndTimer.reset(); state = AutoState.INTAKE3_DWELL; }
                break;

            case INTAKE3_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    followGateOpen(paths.gateopen3, T_GATEOPEN3); state = AutoState.GATEOPEN3_PATH;
                }
                break;

            case GATEOPEN3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.stop();
                    gateOpenTimer.reset();
                    state = AutoState.GATEOPEN3_DWELL;
                }
                break;

            case GATEOPEN3_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenTimer.milliseconds() >= GATE_OPEN_DWELL_MS) {
                    followShoot(paths.shoot3, T_SHOOT3); state = AutoState.SHOOT3_PATH;
                }
                break;

            case SHOOT3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT3_SPINUP; }
                break;

            case SHOOT3_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT3_FIRING;
                }
                break;

            case SHOOT3_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    intake.setMode(Intake.Mode.INTAKE); state = AutoState.DONE;
                }
                break;

            case DONE:
                intake.setMode(Intake.Mode.INTAKE);
                break;
        }
    }

    // =========================================================================
    //  PATHS — Red (x = 144 - x_blue, heading = 180 - heading_blue)
    // =========================================================================
    public static class Paths {
        public Path shoot0;
        public Path intake1, gateopen1, shoot1;
        public Path intake2, gateopen2, shoot2;
        public Path intake3, gateopen3, shoot3;

        public Paths(Follower f) {
            shoot0 = line(
                            new Pose(115.789, 132.895), new Pose(88.316, 88.737)).linear(Math.toRadians(37), Math.toRadians(37));

            intake1 = curve(
                            new Pose(88.316, 88.737), new Pose(99.474, 83.711), new Pose(127.474, 83.737)).tangent();

            gateopen1 = curve(
                            new Pose(127.474, 83.737), new Pose(110.763, 75.500), new Pose(127.632, 75.842)).linear(Math.toRadians(0), Math.toRadians(0));

            shoot1 = curve(
                            new Pose(110.632, 75.842), new Pose(88.184, 94.079), new Pose(88.316, 88.316)).linear(Math.toRadians(0), Math.toRadians(30));

            intake2 = curve(
                            new Pose(88.316, 88.316), new Pose(85.737, 47.105), new Pose(94.053, 61.263),
                            new Pose(124.868, 59.605), new Pose(128.000, 59.474)).tangent();

            gateopen2 = curve(
                            new Pose(128.000, 59.474), new Pose(111.263, 75.579), new Pose(127.526, 64.684)).linear(Math.toRadians(0), Math.toRadians(0));

            shoot2 = line(
                            new Pose(111.526, 75.684), new Pose(88.000, 88.211)).linear(Math.toRadians(0), Math.toRadians(30));

            intake3 = curve(
                            new Pose(88.000, 88.211), new Pose(85.184, 49.500),
                            new Pose(82.816, 50.816), new Pose(134.579, 48.263)).tangent();

            gateopen3 = curve(
                            new Pose(134.579, 48.263), new Pose(111.605, 73.789), new Pose(127.579, 64.105)).linear(Math.toRadians(0), Math.toRadians(0));

            shoot3 = line(
                            new Pose(111.579, 73.105), new Pose(86.632, 109.316)).linear(Math.toRadians(0), Math.toRadians(10));
        }
    }
}