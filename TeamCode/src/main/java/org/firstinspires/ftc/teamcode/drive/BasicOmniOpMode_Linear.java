package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")

public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare the 4 drive motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");

        // Set motor directions.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the driver presses STOP.
        while (opModeIsActive()) {

            // Joystick controls.
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Calculate power for each wheel.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double rearLeftPower = axial - lateral + yaw;
            double rearRightPower = axial + lateral - yaw;

            // Normalize powers so none exceeds 1.0.
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

            // Send power to the motors.
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);

            // Telemetry.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData(
                    "Front Left/Right",
                    "%4.2f, %4.2f",
                    frontLeftPower,
                    frontRightPower
            );
            telemetry.addData(
                    "Rear Left/Right",
                    "%4.2f, %4.2f",
                    rearLeftPower,
                    rearRightPower
            );
            telemetry.update();
        }
    }
}

