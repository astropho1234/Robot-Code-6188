package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// HARRO ROBBIE THIS IS MAIN TELEOP

@TeleOp(name="Drive", group="Test")
public class Special extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;

    // Launcher
    private DcMotor Launcher = null;

    // Maximum motor velocity in ticks/sec
    private static final double velConst =
            DriveConstants.MAX_RPM / 60.0
                    * DriveConstants.TICKS_PER_REV;

    // IMU
    private IMU imu;

    @Override
    public void runOpMode() {

        // =========================
        // IMU SETUP
        // =========================

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters =
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                DriveConstants.LOGO_FACING_DIR,
                                DriveConstants.USB_FACING_DIR
                        )
                );

        imu.initialize(parameters);

        // =========================
        // MOTOR SETUP
        // =========================

        leftFront =
                hardwareMap.get(DcMotorEx.class, "frontleft");

        leftRear =
                hardwareMap.get(DcMotorEx.class, "rearleft");

        rightRear =
                hardwareMap.get(DcMotorEx.class, "rearright");

        rightFront =
                hardwareMap.get(DcMotorEx.class, "frontright");

        Launcher =
                hardwareMap.get(DcMotor.class, "launcher_flywheel");

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // =========================
        // RUN USING ENCODERS
        // =========================

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // =========================
        // INITIALIZATION
        // =========================

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "FIELD CENTRIC");
        telemetry.addData("Max Velocity", "%.0f ticks/sec", velConst);
        telemetry.update();

        waitForStart();

        runtime.reset();

        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {

            // ---------------------------------
            // GET DRIVER INPUT
            // ---------------------------------

            // FTC joystick Y is negative when pushed forward,
            // so invert it.
            double joystickY = -gamepad1.left_stick_y;

            // Left/right
            double joystickX = gamepad1.left_stick_x;

            // Rotation
            double yaw = gamepad1.right_stick_x;

            // ---------------------------------
            // GET ROBOT HEADING
            // ---------------------------------

            double heading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            // ---------------------------------
            // FIELD-CENTRIC TRANSFORMATION
            // ---------------------------------

            double rotX =
                    joystickX * Math.cos(-heading)
                            - joystickY * Math.sin(-heading);

            double rotY =
                    joystickX * Math.sin(-heading)
                            + joystickY * Math.cos(-heading);

            // ---------------------------------
            // MECANUM CALCULATIONS
            // ---------------------------------

            double frontLeftPower =
                    rotY + rotX + yaw;

            double frontRightPower =
                    rotY - rotX - yaw;

            double rearLeftPower =
                    rotY - rotX + yaw;

            double rearRightPower =
                    rotY + rotX - yaw;

            // ---------------------------------
            // NORMALIZE POWER VALUES
            // ---------------------------------

            double maxPower = Math.max(
                    Math.abs(frontLeftPower),
                    Math.abs(frontRightPower)
            );

            maxPower = Math.max(
                    maxPower,
                    Math.abs(rearLeftPower)
            );

            maxPower = Math.max(
                    maxPower,
                    Math.abs(rearRightPower)
            );

            if (maxPower > 1.0) {

                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                rearLeftPower /= maxPower;
                rearRightPower /= maxPower;
            }

            // ---------------------------------
            // CONVERT POWER TO VELOCITY
            // ---------------------------------

            double frontLeftVel =
                    frontLeftPower * velConst;

            double frontRightVel =
                    frontRightPower * velConst;

            double rearLeftVel =
                    rearLeftPower * velConst;

            double rearRightVel =
                    rearRightPower * velConst;

            // ---------------------------------
            // SEND VELOCITY TO MOTORS
            // ---------------------------------

            leftFront.setVelocity(frontLeftVel);
            rightFront.setVelocity(frontRightVel);
            leftRear.setVelocity(rearLeftVel);
            rightRear.setVelocity(rearRightVel);

            // ---------------------------------
            // LAUNCHER
            // ---------------------------------

            if (gamepad1.a) {
                Launcher.setPower(0.65);
            } else {
                Launcher.setPower(0);
            }

            // ---------------------------------
            // TELEMETRY
            // ---------------------------------

            telemetry.addData(
                    "Status",
                    "Run Time: " + runtime.toString()
            );

            telemetry.addData(
                    "Drive Mode",
                    "FIELD CENTRIC"
            );

            telemetry.addData(
                    "Heading",
                    "%.1f degrees",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Max Velocity",
                    "%.0f ticks/sec",
                    velConst
            );

            telemetry.addData(
                    "FL / FR",
                    "%.0f / %.0f",
                    frontLeftVel,
                    frontRightVel
            );

            telemetry.addData(
                    "RL / RR",
                    "%.0f / %.0f",
                    rearLeftVel,
                    rearRightVel
            );

            telemetry.update();
        }
    }
}
