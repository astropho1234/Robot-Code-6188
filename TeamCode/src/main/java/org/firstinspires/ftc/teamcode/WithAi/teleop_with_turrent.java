package org.firstinspires.ftc.teamcode.WithAi;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Drive_Turret", group="Test")
public class teleop_with_turrent extends LinearOpMode {

    // motors
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotorEx turret;
    private IMU imu;

    // drive
    private static final double MAX_TPS = 2400.0;

    // Turret
    private static final double MOTOR_TICKS_PER_REV = 537.7;
    private static final double TURRET_RATIO = 5.11;
    private static final double TURRET_TICKS_PER_RAD =
            (MOTOR_TICKS_PER_REV * TURRET_RATIO) / (2 * Math.PI);
    private static final double TURRET_MIN = Math.toRadians(-90);
    private static final double TURRET_MAX = Math.toRadians(90);

    // ---- Field frame: inches, heading in degrees, 0 = facing +X, CCW positive ----
    // Where you physically place the robot (center of robot). Change these or use presets in init.
    private double startX = 0, startY = 0, startHeadingDeg = 0;

    // What the turret aims at (same field frame). Nudge with dpad in init.
    private double targetX = 0, targetY = 0;

    // Odometry (calibrate these)
    private static final double TICKS_PER_INCH = (28 * 15.1) / (Math.PI * 96 / 25.4);
    private static final double STRAFE_SCALE = 1.1;

    private double robotX, robotY;
    private int lfPrev, lrPrev, rfPrev, rrPrev;

    @Override
    public void runOpMode() {

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Drive motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "frontleft");
        leftRear   = hardwareMap.get(DcMotorEx.class, "rearleft");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rearright");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontright");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Turret: must be pointing straight forward at init
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);

        // ---- Init loop: pick start pose and target ----
        while (!isStarted() && !isStopRequested()) {
            // Start presets (EDIT these to your real start spots)
            if (gamepad1.x) { startX = -36; startY = -60; startHeadingDeg = 90; }  // preset A
            if (gamepad1.b) { startX =  36; startY = -60; startHeadingDeg = 90; }  // preset B
            if (gamepad1.a) { startX =   0; startY =   0; startHeadingDeg = 0;  }  // origin, facing +X

            // Nudge target
            if (gamepad1.dpad_up)    targetY += 0.5;
            if (gamepad1.dpad_down)  targetY -= 0.5;
            if (gamepad1.dpad_right) targetX += 0.5;
            if (gamepad1.dpad_left)  targetX -= 0.5;

            telemetry.addData("Start (x, y, heading)", "%.1f, %.1f, %.0f deg", startX, startY, startHeadingDeg);
            telemetry.addData("Target (x, y)", "%.1f, %.1f", targetX, targetY);
            telemetry.addLine("X = preset A, B = preset B, A = origin, dpad = nudge target");
            telemetry.addLine("Turret must be pointing straight forward now.");
            telemetry.update();
            sleep(100); // limits dpad repeat rate
        }
        if (isStopRequested()) return;

        imu.resetYaw();
        robotX = startX;
        robotY = startY;

        // Start odometry from the current encoder readings
        lfPrev = leftFront.getCurrentPosition();
        lrPrev = leftRear.getCurrentPosition();
        rfPrev = rightFront.getCurrentPosition();
        rrPrev = rightRear.getCurrentPosition();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Re-zero: treats the direction the robot is facing now as the start heading
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Heading in the field frame (IMU yaw + start heading offset)
            double botHeading = AngleUnit.normalizeRadians(
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                            + Math.toRadians(startHeadingDeg));

            // ---- Field-centric drive ----
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            leftFront.setVelocity(frontLeftPower * MAX_TPS);
            leftRear.setVelocity(backLeftPower * MAX_TPS);
            rightFront.setVelocity(frontRightPower * MAX_TPS);
            rightRear.setVelocity(backRightPower * MAX_TPS);

            // ---- Odometry ----
            int lf = leftFront.getCurrentPosition();
            int lr = leftRear.getCurrentPosition();
            int rf = rightFront.getCurrentPosition();
            int rr = rightRear.getCurrentPosition();

            double dLf = lf - lfPrev, dLr = lr - lrPrev, dRf = rf - rfPrev, dRr = rr - rrPrev;
            lfPrev = lf; lrPrev = lr; rfPrev = rf; rrPrev = rr;

            double fwd   = (dLf + dLr + dRf + dRr) / 4.0 / TICKS_PER_INCH;
            double right = (dLf - dLr - dRf + dRr) / 4.0 / TICKS_PER_INCH / STRAFE_SCALE;

            robotX += fwd * Math.cos(botHeading) + right * Math.sin(botHeading);
            robotY += fwd * Math.sin(botHeading) - right * Math.cos(botHeading);

            // ---- Turret tracking ----
            double fieldAngle  = Math.atan2(targetY - robotY, targetX - robotX);
            double turretAngle = AngleUnit.normalizeRadians(fieldAngle - botHeading);
            turretAngle = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretAngle));

            turret.setTargetPosition((int) Math.round(turretAngle * TURRET_TICKS_PER_RAD));

            // ---- Telemetry ----
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("X / Y (in)", "%.1f / %.1f", robotX, robotY);
            telemetry.addData("Target", "%.1f / %.1f", targetX, targetY);
            telemetry.addData("Turret target (deg)", Math.toDegrees(turretAngle));
            telemetry.addData("Turret pos (ticks)", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}