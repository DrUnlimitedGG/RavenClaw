
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Red_TeleOp")
public class MecanumDriveRed extends OpMode {

    public DcMotorEx right_front;
    public DcMotorEx left_front;
    public DcMotorEx right_back;
    public DcMotorEx left_back;
    public DcMotorEx carousel;
    public DcMotorEx cascadingLift;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double encoderConstant = 89.1267682333;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    private ElapsedTime PIDTimerLF = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerLB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerRF = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerRB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerCarousel = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerCascade = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    // code to run when driver hits INIT
    public void init() {

        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        carousel = hardwareMap.get(DcMotorEx.class, "sustainable");
        cascadingLift = hardwareMap.get(DcMotorEx.class, "cascading_lift");

        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);
        left_front.setDirection(DcMotorEx.Direction.FORWARD);
        left_back.setDirection(DcMotorEx.Direction.FORWARD);

        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        cascadingLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

        left_front.setPower(left_front_power);
        left_back.setPower(left_back_power);
        right_front.setPower(right_front_power);
        right_back.setPower(right_back_power);

        while (gamepad2.right_bumper == true) {
            PIDcarousel(-200);
            if (gamepad2.right_bumper == false) {
                carousel.setPower(0);
                break;
            }
        }

        while (gamepad2.left_bumper == true) {
            PIDcarousel(200);
            if (gamepad2.left_bumper == false) {
                carousel.setPower(0);
                break;
            }
        }

        if (gamepad1.b == true) {
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

            right_front.setVelocity(-200);
            right_back.setVelocity(-200);
            left_front.setVelocity(200);
            left_back.setVelocity(200);

            while (right_front.isBusy() || right_back.isBusy() || left_front.isBusy() || left_back.isBusy()) {
                PIDRF(-200);
                PIDRB(-200);
                PIDLF(200);
                PIDLB(200);
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

        if (gamepad1.dpad_right == true) {
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);

            boolean xAxisLockLoop = true;

            while (xAxisLockLoop == true) {
                left_front.setPower(lx);
                left_back.setPower(-lx);
                right_front.setPower(-lx);
                right_back.setPower(lx);

                if (gamepad1.dpad_left == true && gamepad1.dpad_right == false) {
                    left_front.setPower(0);
                    left_back.setPower(0);
                    right_front.setPower(0);
                    right_back.setPower(0);

                    break;
                }
            }
        }

        if (gamepad1.dpad_up == true) {
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);

            boolean yAxisLockLoop = true;

            while (yAxisLockLoop == true) {
                left_front.setPower(ly);
                left_back.setPower(ly);
                right_front.setPower(ly);
                right_back.setPower(ly);

                if (gamepad1.dpad_down == true && gamepad1.dpad_up == false) {
                    left_front.setPower(0);
                    left_back.setPower(0);
                    right_front.setPower(0);
                    right_back.setPower(0);

                    break;
                }
            }
        }

        if (gamepad2.dpad_up == true) {
            cascadingLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascadingLift.setTargetPosition(30);
            cascadingLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            cascadingLift.setVelocity(20);
            while (cascadingLift.isBusy()) {
                PIDcascade(20);
            }

            cascadingLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad2.dpad_down == true) {
            cascadingLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascadingLift.setTargetPosition(-30);
            cascadingLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            cascadingLift.setVelocity(20);
            while (cascadingLift.isBusy()) {
                PIDcascade(20);
            }

            cascadingLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        telemetry.addData("Runtime: ", runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    double integralLF = 0;
    double lastErrorLF = 0;

    public void PIDLF(double targetVelocity) {
        PIDTimerLF.reset();
        double currentVelocity = left_front.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralLF += error * PIDTimerLF.time();

        double deltaError = error - lastErrorLF;
        double derivative = deltaError / PIDTimerLF.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralLF;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;


        left_front.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

    double integralLB = 0;
    double lastErrorLB = 0;

    public void PIDLB(double targetVelocity) {
        PIDTimerLB.reset();
        double currentVelocity = left_back.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralLB += error * PIDTimerLB.time();

        double deltaError = error - lastErrorLB;
        double derivative = deltaError / PIDTimerLB.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralLB;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;


        left_back.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

    double integralRF = 0;
    double lastErrorRF = 0;

    public void PIDRF(double targetVelocity) {
        PIDTimerRF.reset();
        double currentVelocity = right_front.getPower();

        double error = targetVelocity - currentVelocity;

        integralRF += error * PIDTimerRF.time();

        double deltaError = error - lastErrorRF;
        double derivative = deltaError / PIDTimerRF.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralRF;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;


        right_front.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

    double integralRB = 0;
    double lastErrorRB = 0;

    public void PIDRB(double targetVelocity) {
        PIDTimerRB.reset();
        double currentVelocity = right_back.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralRB += error * PIDTimerRB.time();

        double deltaError = error - lastErrorRB;
        double derivative = deltaError / PIDTimerRB.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralRB;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;

        right_back.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }

    double integralCarousel = 0;
    double lastErrorCarousel = 0;

    public void PIDcarousel(double targetVelocity) {
        PIDTimerCarousel.reset();
        double currentVelocity = carousel.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralRB += error * PIDTimerCarousel.time();

        double deltaError = error - lastErrorRB;
        double derivative = deltaError / PIDTimerCarousel.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralRB;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;

        carousel.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);
    }

    double integralCascade = 0;
    double lastErrorCascade = 0;

    public void PIDcascade(double targetVelocity) {
        PIDTimerCascade.reset();
        double currentVelocity = cascadingLift.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralCascade += error * PIDTimerCascade.time();

        double deltaError = error - lastErrorCascade;
        double derivative = deltaError / PIDTimerCascade.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integralRB;
        pidGains.d = pidCoeffs.d = pidCoeffs.d * derivative;

        cascadingLift.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

    }
}
