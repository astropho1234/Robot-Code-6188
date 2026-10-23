package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

//HARRO ROBBIE THIS IS MAIN TELEOP

@Autonomous(name="BasicAuto", group="Test")

public class BasicAuto extends LinearOpMode {

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

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            frontLeft.setTargetPosition(100);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setVelocity(VELCONST * 0.25);

            rearLeft.setTargetPosition(100);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setVelocity(VELCONST * 0.25);

            frontRight.setTargetPosition(100);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setVelocity(VELCONST * 0.25);

            rearRight.setTargetPosition(100);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setVelocity(VELCONST * 0.25);
            if (timer.seconds() > 2) {
                frontLeft.setTargetPosition(300);
                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeft.setVelocity(VELCONST * 0.25);

                rearLeft.setTargetPosition(-100);
                rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearLeft.setVelocity(VELCONST * 0.25);

                frontRight.setTargetPosition(300);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setVelocity(VELCONST * 0.25);

                rearRight.setTargetPosition(-100);
                rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearRight.setVelocity(VELCONST * 0.25);
            }
        }
    }
}
