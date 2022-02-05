package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PIDTest", group = "Tests")
public class OldPID extends LinearOpMode {

    private DcMotorEx LF = null;
    private DcMotorEx LB = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;

    private double encoderConstant = 45.2847909695;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");

        LF.setDirection(DcMotorEx.Direction.FORWARD);
        LB.setDirection(DcMotorEx.Direction.FORWARD);
        // reverse right side motors
        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);

        LF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            PID(100, LF);
            PID(100, LB);
            PID(100, RF);
            PID(100, RB);


        }
    }

    double integral = 0;
    double lastError = 0;
    public void PID(double targetVelocity, DcMotorEx motor) {
        PIDTimer.reset();
        double currentVelocity = motor.getVelocity();

        double error = targetVelocity - currentVelocity;

        integral += error * PIDTimer.time();

        double deltaError = error - lastError;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integral;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;
        motor.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
    }

    public void quitDriving() {
        // Sets all motor speeds to 0
        LF.setVelocity(0);
        LB.setVelocity(0);
        RF.setVelocity(0);
        RB.setVelocity(0);

        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


}
