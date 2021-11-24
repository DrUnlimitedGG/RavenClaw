
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp  ")
public class MecanumDrive extends OpMode {

    public DcMotorEx right_front;
    public DcMotorEx left_front;
    public DcMotorEx right_back;
    public DcMotorEx left_back;
    public DcMotorEx carousel;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double encoderConstant = 89.1267682333;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    private ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    // code to run when driver hits INIT
    public void init() {

        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        carousel = hardwareMap.get(DcMotorEx.class, "sustainable");

        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);
        left_front.setDirection(DcMotorEx.Direction.FORWARD);
        left_back.setDirection(DcMotorEx.Direction.FORWARD);

        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);




    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        carousel.setPower(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        // double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

        /* max power is is 0.7 cuz testing and don't wan to go crazy;  change if necessary
        divide by denominator to correct for imperfect strafing */
        double left_front_power = (ly + lx - rx);
        double left_back_power = (ly - lx - rx);
        double right_front_power = (ly - lx + rx);
        double right_back_power = (ly + lx + rx);

        PIDLF(left_front_power);
        PIDLB(left_back_power);
        PIDRF(right_front_power);
        PIDRB(right_back_power);

        while (gamepad1.dpad_up == true) {
            carousel.setVelocity(200);
            if (gamepad1.dpad_up == false) {
                carousel.setPower(0);
                break;
            }
        }

        if (gamepad1.a == true) {
            right_front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            left_front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            left_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            right_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            /*
            right_front.setTargetPosition(-TK);
            right_back.setTargetPosition(-TK);
            left_front.setTargetPosition(TK);
            left_back.setTargetPosition(TK);
             */

            right_front.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            left_front.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            right_front.setPower(-0.1);
            right_back.setPower(-0.1);
            left_front.setPower(0.1);
            left_back.setPower(0.1);

            while (right_front.isBusy() || right_back.isBusy() || left_front.isBusy() || left_back.isBusy()) {
            }

            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);

            left_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        }
        telemetry.addData("Runtime: ", runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {

    }

    double integralLF = 0;
    double lastErrorLF = 0;
    public void PIDLF(double targetVelocity) {
        PIDTimer.reset();
        double currentVelocity = left_front.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralLF += error * PIDTimer.time();

        double deltaError = error - lastErrorLF;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralLF;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;


        left_front.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

    double integralLB = 0;
    double lastErrorLB = 0;
    public void PIDLB(double targetVelocity) {
        PIDTimer.reset();
        double currentVelocity = left_back.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralLB += error * PIDTimer.time();

        double deltaError = error - lastErrorLB;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralLB;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;


        left_back.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

    double integralRF = 0;
    double lastErrorRF = 0;
    public void PIDRF(double targetVelocity) {
        PIDTimer.reset();
        double currentVelocity = right_front.getPower();

        double error = targetVelocity - currentVelocity;

        integralRF += error * PIDTimer.time();

        double deltaError = error - lastErrorRF;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralRF;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;


        right_front.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

    double integralRB = 0;
    double lastErrorRB = 0;
    public void PIDRB(double targetVelocity) {
        PIDTimer.reset();
        double currentVelocity = right_back.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralRB += error * PIDTimer.time();

        double deltaError = error - lastErrorRB;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralRB;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;

        right_back.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

}
