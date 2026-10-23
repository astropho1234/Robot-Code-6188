package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name = "FieldCentric", group = "Test")

public class FieldCentric extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearRight = null;
    private DcMotorEx outerIntake = null;
    private DcMotorEx innerIntake = null;
    private DcMotorEx turretSpin = null;
    private DcMotorEx turretShoot = null;
    private SparkFunOTOS odometry = null;

    private double lowPower;
    private static final double VELCONST = DriveConstants.MAX_RPM / 60.0 * DriveConstants.TICKS_PER_REV;
    private static final double TURRETGEARRATIO = 3.61 * 3.61 * 3.61 * 28;
    private double outerIntakeVel;
    private double innerIntakeVel;
    private boolean manualTurret;
    private int activeLauncher;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearright");

        // Intake motors
        outerIntake = hardwareMap.get(DcMotorEx.class, "intake1");
        innerIntake = hardwareMap.get(DcMotorEx.class, "intake2");

        // Turret motors
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretspin");
        turretShoot = hardwareMap.get(DcMotorEx.class, "turretshoot");

        // Odometry
        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        turretSpin.setDirection(DcMotor.Direction.REVERSE);
        turretShoot.setDirection(DcMotor.Direction.REVERSE);

        // motor modes
        // assumes initial position is straight forwards
        turretSpin.setTargetPosition(0);
        turretSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretSpin.setVelocity(0);

        // Initialize IMU and odometry
        IMU imu = hardwareMap.get(IMU.class, "imu");

         odometry.setLinearUnit(DistanceUnit.METER);
         odometry.setAngularUnit(AngleUnit.DEGREES);
         // all of these below are temp values that need to be changed
         odometry.setOffset(new SparkFunOTOS.Pose2D(0,0,0));
         odometry.setLinearScalar(1.463);
         odometry.setAngularScalar(1.0);

        // Change these if your Control Hub is mounted differently
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        lowPower = 1;
        outerIntakeVel = 0.0;
        innerIntakeVel = 0.0;
        manualTurret = true;
        activeLauncher = 0;

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
            double x = -gamepad1.left_stick_x * lowPower;
            double rx = -gamepad1.right_stick_x * lowPower;

            // intake
            if (gamepad1.aWasPressed()) {
                outerIntakeVel = VELCONST * 0.9;
            } else if (gamepad1.bWasPressed()) {
                outerIntakeVel = VELCONST * -0.9;
            }

            if (gamepad2.bWasPressed()) {
                innerIntakeVel = VELCONST;
            }
            if (gamepad2.yWasPressed()) {
                innerIntakeVel = -VELCONST;
            }

            if (gamepad1.rightBumperWasPressed()){
                if (lowPower == 1) {
                    lowPower = 0.5;
                } else {
                    lowPower = 1;
                }
            }

            if (gamepad1.leftBumperWasPressed()){
                imu.resetYaw();
                odometry.setPosition(new SparkFunOTOS.Pose2D(1.0266,0.02,0));
            }

            // launcher
            if (gamepad2.dpadUpWasPressed()) {
                if (activeLauncher == 1) activeLauncher = 0;
                else activeLauncher = 1;
            }

            // if (gamepad2.aWasPressed()) manualTurret = !manualTurret;

            /*
            Important numbers to aim
            the center of an upwards hive is 0.5686 m out, 0.32375 m across, and 1.51257 m up
            from the center of the field

            assume position starts from the left corner
             */

            SparkFunOTOS.Pose2D odometryPos = odometry.getPosition();

            // checking which hive to aim for
            double xDistToTarget;
            if (odometryPos.x < 1.83) xDistToTarget = 1.5575 - odometryPos.x;
            else xDistToTarget = 2.1025 - odometryPos.x;

            double yDistToTarget = 1.50625 - odometryPos.y;

            // specifically amount to rotate, takes into account robot rotation
            double angleToTarget = Math.atan2(xDistToTarget, yDistToTarget) - Math.toRadians(odometryPos.h); // in radians
            // Convert angle from -pi to pi to encoder ticks
            int turretTicksToRotate = (int) (angleToTarget * TURRETGEARRATIO / Math.PI);

            // checking if it will rotate outside of the range
            if (turretTicksToRotate > 0.777 * TURRETGEARRATIO) {
                // setting turret position to the closest point outside the limits
                // there will be some cursed behavior with setting directly to -pi position if the robot is rotated, fix later
                if (turretTicksToRotate > 0.166 * TURRETGEARRATIO) turretTicksToRotate = (int) (-1 * TURRETGEARRATIO);
                else turretTicksToRotate = (int) (0.777 * TURRETGEARRATIO);
            }
            else if (turretTicksToRotate < -1 * TURRETGEARRATIO) {
                // same cursed behavior here too with setting directly instead of math
                // maybe try Math.clamp()?
                turretTicksToRotate = (int) (0.777 * TURRETGEARRATIO);
            }

            double launchAngle = Math.toRadians(45);
            // speed is only the speed of the ball, not the launcher vel
            double launchSpeed = Math.sqrt(Math.abs((4.7 * xDistToTarget * xDistToTarget) /
                    (Math.cos(launchAngle) * Math.cos(launchAngle) * (xDistToTarget * Math.tan(launchAngle) - yDistToTarget))));
            double launcherVel = activeLauncher * launchSpeed * DriveConstants.TICKS_PER_REV / (.045 * 2 * Math.PI); // .045 is radius of wheel

            if (manualTurret) {
                if (gamepad2.dpadRightWasPressed()) {
                    turretTicksToRotate = turretSpin.getCurrentPosition() + 20;
                }
                else if (gamepad2.dpadLeftWasPressed()) {
                    turretTicksToRotate = turretSpin.getCurrentPosition() - 20;
                }
                if (gamepad2.x) {
                    launcherVel = 0.9 * VELCONST;
                }
                else launcherVel = 0;
            }


            // Get robot heading
            double botHeading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            // Convert robot-centric controls to field-centric
            double rotX = x * Math.cos(-botHeading)
                    - y * Math.sin(-botHeading);

            double rotY = x * Math.sin(-botHeading)
                    + y * Math.cos(-botHeading);

            // Compensate for imperfect strafing
            rotX *= 1.1;

            // Calculate mecanum velocities
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftVel = VELCONST * (rotY + rotX + rx) / denominator;
            double rearLeftVel = VELCONST * (rotY - rotX + rx) / denominator;
            double frontRightVel = VELCONST * (rotY - rotX - rx) / denominator;
            double rearRightVel = VELCONST * (rotY + rotX - rx) / denominator;

            // Send velocity to motors
            frontLeft.setVelocity(frontLeftVel);
            frontRight.setVelocity(frontRightVel);
            rearLeft.setVelocity(rearLeftVel);
            rearRight.setVelocity(rearRightVel);

            outerIntake.setVelocity(outerIntakeVel);
            innerIntake.setVelocity(innerIntakeVel);

            turretSpin.setTargetPosition(turretTicksToRotate);
            turretSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // might want pid for this later instead
            // turretSpin.setVelocity(0.25 * VELCONST);

            turretShoot.setVelocity(launcherVel);

            // Telemetry
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

            telemetry.addData("Inner Intake vel: ",innerIntakeVel);
            telemetry.addData("Outer Intake vel: ",outerIntakeVel);
            telemetry.addData("turret spinning: ",activeLauncher);
            telemetry.addData("turret vel: ",launcherVel);
            telemetry.addData("ball launch speed: ",launchSpeed);
            telemetry.addData("turret target pos: ",turretTicksToRotate);
            telemetry.addData("turret target angle: ",angleToTarget);

            telemetry.addData("robot x: ",odometryPos.x);
            telemetry.addData("robot y: ",odometryPos.y);
            telemetry.addData("robot x dist: ",xDistToTarget);
            telemetry.addData("robot y dist: ",yDistToTarget);
            telemetry.addData("robot h: ",odometryPos.h);
            telemetry.addData("atan math: ",xDistToTarget/yDistToTarget);
            telemetry.addData("manual turret controls: ",manualTurret);

            telemetry.update();
        }
    }
}