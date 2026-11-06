package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name = "LaunchDataGather" +
        "", group = "Test")

public class LaunchDataGather extends LinearOpMode {

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

    private static final double VELCONST = DriveConstants.MAX_RPM / 60.0 * DriveConstants.TICKS_PER_REV;
    // remove negative when it has a turret
    private double lowPower;
    private double outerIntakeVel;
    private double innerIntakeVel;
    private int activeIntake; // 0 = off, 1 = in, 2 = out
    private double prevBotHeading;
    private double launcherVel;

    private static final double[][][] launcherVelPoints = { // formated as {x,y,vel} in a 2d array
            {{0,0,0}, {0,1,0}},
            {{0,1,0}, {1,1,0}}
    };

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
        innerIntake.setDirection(DcMotor.Direction.REVERSE);

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
         odometry.setOffset(new SparkFunOTOS.Pose2D(0.1885,0.035,180));
         odometry.setLinearScalar(1.463);
         odometry.setAngularScalar(1.0);
         odometry.resetTracking();







        // Change these if your Control Hub is mounted differently
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        lowPower = 1;
        outerIntakeVel = 0.0;
        innerIntakeVel = 0.0;
        activeIntake = 0;
        prevBotHeading = 0.0;

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

            // Get robot heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            SparkFunOTOS.Pose2D odometryPos = odometry.getPosition();

            double xDistToTarget;
            if (odometryPos.x < 1.83) xDistToTarget = 1.5575 - odometryPos.x;
            else xDistToTarget = 2.1025 - odometryPos.x;

            double yDistToTarget = 1.50625 - odometryPos.y;

            // saving new target imu pos if rotated
            boolean hasRotated;
            if (Math.abs(rx) > 0.05) prevBotHeading = botHeading;

            // intake
            if (gamepad1.aWasPressed()) {
                if (activeIntake == 1) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = VELCONST * 0.5;
                    activeIntake = 1;
                }

            } else if (gamepad1.bWasPressed()) {
                if (activeIntake == 2) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = -VELCONST * 0.5;
                    activeIntake = 2;
                }

            }

            if (gamepad2.b) {
                innerIntakeVel = VELCONST * 0.2;
            }
            else if (gamepad2.a) {
                innerIntakeVel = -VELCONST * 0.2;
            }

            if (gamepad1.rightBumperWasPressed()){
                if (lowPower == 1) {
                    lowPower = 0.25;
                } else {
                    lowPower = 1;
                }
            }

            if (gamepad1.leftBumperWasPressed()){
                imu.resetYaw();
                odometry.setPosition(new SparkFunOTOS.Pose2D(1.0266,0.02,180));
            }

            // launcher
            if (gamepad2.dpadLeftWasPressed()) {
                launcherVel = 0;
            }
            else if (gamepad2.dpadDownWasPressed()) {
                launcherVel -= 0.05 * VELCONST;
            }
            else if (gamepad2.dpadUpWasPressed()) {
                launcherVel += 0.01 * VELCONST;
            }
            else if (gamepad2.dpadRightWasPressed()) {
                launcherVel += 0.1 * VELCONST;
            }
            if (launcherVel > VELCONST) launcherVel = VELCONST;
            if (launcherVel < -VELCONST) launcherVel = -VELCONST;

            // Convert robot-centric controls to field-centric
            double rotX = x * Math.cos(-botHeading)
                    - y * Math.sin(-botHeading);

            double rotY = x * Math.sin(-botHeading)
                    + y * Math.cos(-botHeading);

            // Compensate for imperfect strafing
            rotX *= 1.1;

            // adjust for unintentional rotation

            // Calculate mecanum velocities
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftVel = VELCONST * (rotY + rotX + rx) / denominator;
            double rearLeftVel = VELCONST * (rotY - rotX + rx) / denominator;
            double frontRightVel = VELCONST * (rotY - rotX - rx) / denominator;
            double rearRightVel = VELCONST * (rotY + rotX - rx) / denominator;

            // correcting for misaligned imu movements
            frontLeftVel += Math.toDegrees(prevBotHeading - botHeading) * VELCONST / 50;
            frontRightVel -= Math.toDegrees(prevBotHeading - botHeading) * VELCONST / 50;
            // accounting for a velocity greater than the max by reducing the opposite wheel instead
            if (frontLeftVel > VELCONST) {
                rearRightVel -= frontLeftVel - VELCONST;
                frontLeftVel = VELCONST;
            }
            if (frontRightVel < -VELCONST) {
                rearLeftVel += frontRightVel + VELCONST;
                frontRightVel = -VELCONST;
            }

            // Send velocity to motors
            frontLeft.setVelocity(frontLeftVel);
            frontRight.setVelocity(frontRightVel);
            rearLeft.setVelocity(rearLeftVel);
            rearRight.setVelocity(rearRightVel);

            outerIntake.setVelocity(outerIntakeVel);
            innerIntake.setVelocity(innerIntakeVel);

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

            // telemetry.addData("Inner Intake vel: ",innerIntakeVel);
            // telemetry.addData("Outer Intake vel: ",outerIntakeVel);
            telemetry.addData("turret vel: ",launcherVel);
            // telemetry.addData("ball launch speed: ",launchSpeed);

            telemetry.addData("robot x: ",odometryPos.x);
            telemetry.addData("robot y: ",odometryPos.y);
            telemetry.addData("robot x dist: ",xDistToTarget);
            telemetry.addData("robot y dist: ",yDistToTarget);
            telemetry.addData("robot h: ",odometryPos.h);
            telemetry.addData("low power mode: ",lowPower);

            telemetry.addData("frontLeft power draw: ", frontLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rearLeft power draw: ", rearLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("frontRight power draw: ", frontRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rearRight power draw: ", rearRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("outerIntake power draw: ", outerIntake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("innerIntake power draw: ", innerIntake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("turretSpin power draw: ", turretSpin.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("turretShoot power draw: ", turretShoot.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }
    }
}