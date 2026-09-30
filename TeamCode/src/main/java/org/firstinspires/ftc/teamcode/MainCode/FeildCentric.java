package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FeildCentric", group = "Test")

public class FeildCentric extends LinearOpMode { // robbie make sure to change motor directions becuase you dident push your old code rahhhh and now i dont have a clue the motor directions

    // Drive motors
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        rearRight = hardwareMap.get(DcMotor.class, "rearright");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Change these if your Control Hub is mounted differently
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        logoDirection,
                        usbDirection
                )
        ));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Joystick controls
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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

            // Calculate mecanum powers
            double frontLeftPower = rotY + rotX + rx;
            double frontRightPower = rotY - rotX - rx;
            double rearLeftPower = rotY - rotX + rx;
            double rearRightPower = rotY + rotX - rx;

            // Normalize powers
            double max = Math.max(
                    Math.abs(frontLeftPower),
                    Math.abs(frontRightPower)
            );

            max = Math.max(max, Math.abs(rearLeftPower));
            max = Math.max(max, Math.abs(rearRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                rearLeftPower /= max;
                rearRightPower /= max;
            }

            // Send power to motors
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            rearLeft.setPower(rearLeftPower);
            rearRight.setPower(rearRightPower);

            // Telemetry
            telemetry.addData(
                    "Heading",
                    "%.1f°",
                    Math.toDegrees(botHeading)
            );

            telemetry.addData(
                    "Front Left/Right",
                    "%.2f, %.2f",
                    frontLeftPower,
                    frontRightPower
            );

            telemetry.addData(
                    "Rear Left/Right",
                    "%.2f, %.2f",
                    rearLeftPower,
                    rearRightPower
            );

            telemetry.update();
        }
    }
}