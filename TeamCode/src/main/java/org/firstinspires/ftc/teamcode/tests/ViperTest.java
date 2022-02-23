
package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="ViperTest",group="Tests")
public class ViperTest extends LinearOpMode {
    private DcMotorEx viper_direct = null;
    private DcMotorEx viper_indirect = null;

    FtcDashboard ftcdashboard;

    private final double speed = 1500;

    public static PIDCoefficients pidCoeffsDir = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsDir = new PIDCoefficients(0, 0, 0);

    ElapsedTime PIDTimerDir = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static PIDCoefficients pidCoeffsIndir = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsIndir = new PIDCoefficients(0, 0, 0);

    ElapsedTime PIDTimerIndir = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private boolean viperextended = false;
    private double encoderConstant = 294;

    private double directPower = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        viper_direct = hardwareMap.get(DcMotorEx.class, "viper_direct");
        viper_indirect = hardwareMap.get(DcMotorEx.class, "viper_indirect");

        viper_direct.setDirection(DcMotorEx.Direction.REVERSE);
        viper_indirect.setDirection(DcMotorEx.Direction.FORWARD);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)

        waitForStart();
        if (opModeIsActive()) {
            viper_extend();
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                telemetry.addData("Error: ", e);
                telemetry.update();
            }
            viper_lower();
        }
        }

    public void viper_extend() {
        viper_direct.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        viper_direct.setTargetPosition(1569);
        viper_indirect.setTargetPosition(1569);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(speed);
        viper_indirect.setVelocity(speed);

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {
            PIDdirect(speed);
            directPower = viper_direct.getVelocity();
            PIDindirect(directPower);

            telemetry.addData("Direct: ", viper_direct.getVelocity());
            telemetry.addData("Indirect: ", viper_indirect.getVelocity());
            telemetry.update();

        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    public void viper_lower() {
        /*viper_direct.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/

        viper_direct.setTargetPosition(-10);
        viper_indirect.setTargetPosition(-10);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(-speed);
        viper_indirect.setVelocity(-speed);

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {
            PIDdirect(-speed);
            directPower = viper_direct.getVelocity();
            PIDindirect(directPower);
        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    double integralDir = 0;
    double lastErrorDir = 0;
    public void PIDdirect(double targetVelocity) {
        PIDTimerDir.reset();

        double currentVelocity = viper_direct.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralDir += error * PIDTimerDir.time();

        double deltaError = error - lastErrorDir;
        double derivative = deltaError / PIDTimerDir.time();

        pidGainsDir.p = pidCoeffsDir.p * error;
        pidGainsDir.i = pidCoeffsDir.i * integralDir;
        pidGainsDir.d = pidCoeffsDir.d * derivative;

        viper_direct.setVelocity(targetVelocity + pidGainsDir.p + pidGainsDir.i + pidGainsDir.d);

        lastErrorDir = error;
    }

    double integralIndir = 0;
    double lastErrorIndir = 0;
    public void PIDindirect(double targetVelocity) {
        PIDTimerIndir.reset();

        double currentVelocity = viper_indirect.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralDir += error * PIDTimerIndir.time();

        double deltaError = error - lastErrorDir;
        double derivative = deltaError / PIDTimerIndir.time();

        pidGainsIndir.p = pidCoeffsIndir.p * error;
        pidGainsIndir.i = pidCoeffsIndir.i * integralDir;
        pidGainsIndir.d = pidCoeffsIndir.d * derivative;

        viper_indirect.setVelocity(targetVelocity + pidGainsIndir.p + pidGainsIndir.i + pidGainsIndir.d);

        lastErrorDir = error;
    }

}
