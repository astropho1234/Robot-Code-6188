package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name = "AutoLaunch", group = "Test")

public class AutoLaunch extends LinearOpMode {

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

    private static final double velConst = DriveConstants.MAX_RPM / 60.0 * DriveConstants.TICKS_PER_REV;
    private double outerIntakeVel;
    private double innerIntakeVel;

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

        // motor modes
        // assumes initial position is straight forwards
        turretSpin.setTargetPosition(0);
        turretSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretSpin.setVelocity(0);

        // Initialize IMU and odometry
        IMU imu = hardwareMap.get(IMU.class, "imu");

         odometry.setLinearUnit(DistanceUnit.METER);
         odometry.setAngularUnit(AngleUnit.DEGREES);
         // all of these below are temp values that need to be changed
         odometry.setOffset(new SparkFunOTOS.Pose2D(.17,.1885,0));
         odometry.setLinearScalar(1.0);
         odometry.setAngularScalar(1.0);

        // Change these if your Control Hub is mounted differently
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        outerIntakeVel = 0.0;
        innerIntakeVel = 0.0;

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
            //so you just need to copy the launching math over and make it move right slightly

            // launcher
            int activeLauncher = 1;
            //if (gamepad2.x) activeLauncher = 1;

            /*
            Important numbers to aim
            the center of an upwards hive is 56.86 cm out, 32.375 cm across, and 151.257 cm up
            from the center of the field

            assume position starts from the left corner (offset by 17 cm forwards 18.85 sideways)
             */

            SparkFunOTOS.Pose2D odometryPos = odometry.getPosition();
            double xDistToTarget = 1.2614 - odometryPos.x;
            double yDistToTarget = 1.50625 - odometryPos.y;

            // specifically amount to rotate, takes into account robot rotation
            double angleToTarget = Math.atan2(yDistToTarget, xDistToTarget) - odometryPos.h; // in radians
            // multiply gear ratio in with angleToTarget
            int turretTicksToRotate = (int) ((angleToTarget) * 28 / Math.PI);

            double launchAngle = Math.toRadians(45);
            // speed is only the speed of the ball, not the launcher vel
            double launchSpeed = Math.sqrt((4.7 * xDistToTarget * xDistToTarget) /
                    (Math.cos(launchAngle) * Math.cos(launchAngle) * (xDistToTarget * Math.tan(launchAngle) - yDistToTarget)));
            double launcherVel = activeLauncher * launchSpeed * DriveConstants.TICKS_PER_REV / (.045 * 2 * Math.PI); // .045 is radius of wheel



        }
    }
}