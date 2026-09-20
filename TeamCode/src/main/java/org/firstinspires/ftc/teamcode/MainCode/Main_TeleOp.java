package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//HARRO ROBBIE THIS IS MAIN TELEOP

@TeleOp(name="Drive", group="Test")

public class Main_TeleOp extends LinearOpMode {

    // Declare the 4 drive motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;

    private DcMotor Launcher= null;

    @Override
    public void runOpMode() {

        // Initialize the hardware.
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        rearRight = hardwareMap.get(DcMotor.class, "rearright"); //pretty sure the config isn't named right but motor prob broke
        Launcher = hardwareMap.get(DcMotor.class, "launcher_flywheel");

        // Set motor directions.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

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




            //luncher

            if (gamepad1.a) {
                Launcher.setPower(1);



            }

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
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            rearLeft.setPower(rearLeftPower);
            rearRight.setPower(rearRightPower);

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

