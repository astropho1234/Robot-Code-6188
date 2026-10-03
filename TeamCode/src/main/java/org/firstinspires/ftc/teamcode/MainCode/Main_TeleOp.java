package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

//HARRO ROBBIE THIS IS MAIN TELEOP

@TeleOp(name="Drive", group="Test")

public class Main_TeleOp extends LinearOpMode {

    // Declare the 4 drive motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;

    private DcMotor Launcher= null;

    private static final double velConst = DriveConstants.MAX_RPM / 60.0 * DriveConstants.TICKS_PER_REV;

    private IMU imu;

    @Override
    public void runOpMode() {

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters =
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                DriveConstants.LOGO_FACING_DIR,
                                DriveConstants.USB_FACING_DIR
                        )
                );

        imu.initialize(parameters);


        // Initialize the hardware.
        // Motors
        leftFront =
                hardwareMap.get(DcMotorEx.class, "frontleft");

        leftRear =
                hardwareMap.get(DcMotorEx.class, "rearleft");

        rightRear =
                hardwareMap.get(DcMotorEx.class, "rearright");

        rightFront =
                hardwareMap.get(DcMotorEx.class, "frontright");


        Launcher = hardwareMap.get(DcMotor.class, "launcher_flywheel");

        // Set motor directions.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the driver presses STOP.
        while (opModeIsActive()) {

            // Joystick controls.
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Calculate power for each wheel.
            double frontLeftVel = (axial + lateral + yaw) * velConst;
            double frontRightVel = (axial - lateral - yaw) * velConst;
            double rearLeftVel = (axial - lateral + yaw) * velConst;
            double rearRightVel = (axial + lateral - yaw) * velConst;

            //launcher
            if (gamepad1.a) {
                Launcher.setPower(0.65);

            }

            // Normalize powers so none exceeds 1.0.
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

            // Send power to the motors.
            leftFront.setVelocity(frontLeftVel);
            rightFront.setVelocity(frontRightVel);
            leftRear.setVelocity(rearLeftVel);
            rightRear.setVelocity(rearRightVel);

            // Telemetry.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData(
                    "Front Left/Right",
                    "%4.2f, %4.2f",
                    frontLeftVel,
                    frontRightVel
            );
            telemetry.addData(
                    "Rear Left/Right",
                    "%4.2f, %4.2f",
                    rearLeftVel,
                    rearRightVel
            );
            telemetry.update();
        }
    }
}

