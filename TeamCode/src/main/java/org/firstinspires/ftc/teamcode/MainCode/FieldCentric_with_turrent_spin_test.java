package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name = "FieldCentricTurrent", group = "Test")

public class FieldCentric_with_turrent_spin_test extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearRight = null;

    private DcMotorEx TurretSpin = null;
    private DcMotorEx TurretShoot = null;
    private DcMotorEx Intake = null;
    private DcMotorEx TurretFeed = null;

    double lowPower;

    private static final double velConst = DriveConstants.MAX_RPM / 60.0 * DriveConstants.TICKS_PER_REV;

    // ---- Turret aiming ----
    // Motor revs per 1 turret rev (e.g. 20T motor gear -> 100T turret gear = 5.0)
    private static final double TURRET_GEAR_RATIO = 3.0;
    private static final double TURRET_TICKS_PER_DEG =
            DriveConstants.TICKS_PER_REV * TURRET_GEAR_RATIO / 360.0;

    // ---- Shooter / feed / intake (ticks per second) ----
    private static final double SHOOTER_TARGET_TPS = 1500;        // <-- put your shooter velocity here
    private static final double SHOOTER_READY_TOLERANCE_TPS = 50; // how close counts as "at speed"
    private static final double FEED_TPS = 1000;                  // <-- feed motor velocity (negative to reverse)
    private static final double INTAKE_TPS = 1000;                // <-- intake velocity (negative to reverse)

    private boolean shooterOn = false;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearright");
        // init turrent
        TurretSpin = hardwareMap.get(DcMotorEx.class, "turretspin");
        TurretShoot = hardwareMap.get(DcMotorEx.class, "turretshoot");
        //init intake+turrent feed
        TurretFeed = hardwareMap.get(DcMotorEx.class, "turrentfeed");
        Intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Motor directions drive (dont change these ever these are correct for coding bot)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        // turret motors
        TurretShoot.setDirection(DcMotor.Direction.REVERSE); //robbie you need to configure these right
        TurretSpin.setDirection(DcMotor.Direction.REVERSE);
        //intake + feed motors
        Intake.setDirection(DcMotor.Direction.REVERSE);
        TurretFeed.setDirection(DcMotor.Direction.REVERSE);

        // Turret spin encoder: angle at INIT becomes 0 (point it straight forward before init)
        TurretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter, feed, and intake need encoders for velocity control
        TurretShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretFeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretFeed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Change these if your Control Hub is mounted differently
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        lowPower = 1;

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        logoDirection,
                        usbDirection
                )
        ));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Joystick controls
            double y = gamepad1.left_stick_y * lowPower;
            double x = gamepad1.left_stick_x * lowPower;
            double rx = gamepad1.right_stick_x * 3 * lowPower;

            // Get robot heading
            double botHeading = -imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            // Convert robot-centric controls to field-centric
            double rotX = x * Math.cos(-botHeading)
                    - y * Math.sin(-botHeading);

            double rotY = x * Math.sin(-botHeading)
                    + y * Math.cos(-botHeading);

            // Compensate for imperfect strafing
            rotX *= 1.1;

            // ---------------- turrent aim (hold field zero) ----------------
            double robotHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double robotTurnRateDegPerSec = Math.toDegrees(
                    imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);

            // Turret angle relative to the robot
            double turretAngleDeg = TurretSpin.getCurrentPosition() / TURRET_TICKS_PER_DEG;

            // Turret angle needed to hold field zero, wrapped to [-180, 180] and limited to +/-170 for wires
            double desiredTurretDeg = -robotHeadingDeg;
            desiredTurretDeg = ((desiredTurretDeg + 180) % 360 + 360) % 360 - 180;
            desiredTurretDeg = Math.max(-170, Math.min(170, desiredTurretDeg));

            double errorDeg = desiredTurretDeg - turretAngleDeg;

            // Feedforward (cancel robot rotation) + P correction, in ticks/sec
            double turretVel = (-robotTurnRateDegPerSec + 6.0 * errorDeg) * TURRET_TICKS_PER_DEG;

            // Cap at the same max velocity as the drive motors
            turretVel = Math.max(-velConst, Math.min(velConst, turretVel));

            TurretSpin.setVelocity(turretVel);
            // ---------------------------------------------------------------

            // ---------------- shooter + feed + intake ----------------
            if (gamepad1.aWasPressed()) {
                shooterOn = !shooterOn;
            }
            TurretShoot.setVelocity(shooterOn ? SHOOTER_TARGET_TPS : 0);

            // Hold B to run the feed AND the intake together
            boolean loading = gamepad1.b;
            TurretFeed.setVelocity(loading ? FEED_TPS : 0);
            Intake.setVelocity(loading ? INTAKE_TPS : 0);

            double shooterVel = TurretShoot.getVelocity();
            boolean shooterReady = shooterOn
                    && Math.abs(shooterVel - SHOOTER_TARGET_TPS) <= SHOOTER_READY_TOLERANCE_TPS;
            // ---------------------------------------------------------

            // here is the previous code, it seems to be doing something similar to the correct version
            /*
            double frontLeftVel = rotY + rotX + rx;
            double frontRightVel = rotY - rotX - rx;
            double rearLeftVel = rotY - rotX + rx;
            double rearRightVel = rotY + rotX - rx;

            // Normalize velocities
            double max = Math.max(
                    Math.abs(frontLeftVel),
                    Math.abs(frontRightVel)
            );

            max = Math.max(max, Math.abs(rearLeftVel));
            max = Math.max(max, Math.abs(rearRightVel));

            if (max > velConst) {
                frontLeftVel /= max;
                frontRightVel /= max;
                rearLeftVel /= max;
                rearRightVel /= max;
            }
             */
            // here is the new code that should do this properly
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftVel = velConst * (rotY + rotX + rx) / denominator;
            double rearLeftVel = velConst * (rotY - rotX + rx) / denominator;
            double frontRightVel = velConst * (rotY - rotX - rx) / denominator;
            double rearRightVel = velConst * (rotY + rotX - rx) / denominator;

            // Send velocity to motors
            frontLeft.setVelocity(frontLeftVel);
            frontRight.setVelocity(frontRightVel);
            rearLeft.setVelocity(rearLeftVel);
            rearRight.setVelocity(rearRightVel);

            // Slow mode toggle
            if (gamepad1.rightBumperWasPressed()) {
                lowPower = (lowPower == 0.5) ? 1 : 0.5;
            }

            // Start button: re-zero heading
            if (gamepad1.startWasPressed()) {
                imu.resetYaw();
            }

            // ---------------- Telemetry ----------------
            telemetry.addData(
                    "Heading",
                    "%.1f°",
                    Math.toDegrees(botHeading)
            );

            telemetry.addData(
                    "Front Left/Right",
                    "%.2f, %.2f",
                    frontLeftVel,
                    frontRightVel
            );

            telemetry.addData(
                    "Rear Left/Right",
                    "%.2f, %.2f",
                    rearLeftVel,
                    rearRightVel
            );

            telemetry.addData("Turret rel deg", "%.1f", turretAngleDeg);
            telemetry.addData("Turret field deg", "%.1f", turretAngleDeg + robotHeadingDeg);
            telemetry.addData("Turret vel (ticks/s)", "%.0f", turretVel);

            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Shooter vel / target", "%.0f / %.0f", shooterVel, SHOOTER_TARGET_TPS);
            telemetry.addData("Shooter ready", shooterReady);
            telemetry.addData("Feed + Intake", loading);

            telemetry.update();
        }
    }
}