package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name = "FeildCentric", group = "Test")

public class FieldCentric extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearRight = null;
    private static final double velConst = DriveConstants.MAX_RPM / 60.0 * DriveConstants.TICKS_PER_REV;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearright");

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

            // Calculate mecanum velocities
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

            // Send velocity to motors
            frontLeft.setVelocity(frontLeftVel);
            frontRight.setVelocity(frontRightVel);
            rearLeft.setVelocity(rearLeftVel);
            rearRight.setVelocity(rearRightVel);

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

            telemetry.update();
        }
    }
}