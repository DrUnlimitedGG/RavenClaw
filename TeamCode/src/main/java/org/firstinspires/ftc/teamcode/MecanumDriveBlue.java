
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Blue_TeleOp", group = "TeleOps")
public class MecanumDriveBlue extends OpMode {

    public DcMotorEx right_front;
    public DcMotorEx left_front;
    public DcMotorEx right_back;
    public DcMotorEx left_back;
    public DcMotorEx carousel;
    // public DcMotorEx intake;
    //public DcMotorEx cascadingLift;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double encoderConstant = 89.1267682333;

    public static PIDCoefficients pidCoeffsLF = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsLF = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsLB = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsLB = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsRF = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsRF = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsRB = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsRB = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsCarousel = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsCarousel = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffsCascade = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsCascade = new PIDCoefficients(0, 0, 0);
    
    public static PIDCoefficients pidCoeffsIntake = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsIntake = new PIDCoefficients(0, 0, 0);


    private ElapsedTime PIDTimerLF = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerLB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerRF = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerRB = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerCarousel = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerCascade = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime PIDTimerIntake = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public boolean xAxisLockLoop = false;
    public boolean yAxisLockLoop = false;

    boolean current_DpadUP = false;
    boolean last_DpadUP = false;

    boolean current_DpadRIGHT = false;
    boolean last_DpadRIGHT = false;

    @Override
    // code to run when driver hits INIT
    public void init() {

        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        carousel = hardwareMap.get(DcMotorEx.class, "sustainable");
        // intake = hardwareMap.get(DcMotorEx.class, "intake");
        //cascadingLift = hardwareMap.get(DcMotorEx.class, "cascading_lift");

        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);
        left_front.setDirection(DcMotorEx.Direction.REVERSE);
        left_back.setDirection(DcMotorEx.Direction.FORWARD);

        carousel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //cascadingLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
        //intake.setPower(0);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.dpad_right == true) {
            if (yAxisLockLoop == true) {
                telemetry.addData("Error: ", "Please disengage the Y-Axis lock!");
                telemetry.update();
            } else if (yAxisLockLoop == false) {
                xAxisLockLoop = true;
                telemetry.addData("X Axis Lock: ", "Activated!");
            }
        } else if (gamepad1.dpad_left == true && xAxisLockLoop == true) {
            xAxisLockLoop = false;
            telemetry.addData("X Axis Lock: ", "Disabled!");

        }

        if (gamepad1.dpad_up == true) {
            if (xAxisLockLoop == true) {
                telemetry .addData("Error: ", "Please disengage the X-Axis lock!");
                telemetry.update();
            } else if (xAxisLockLoop == false) {
                yAxisLockLoop = true;
                telemetry.addData("Y Axis Lock: ", "Activated!");

            }
        } else if (gamepad1.dpad_down == true && yAxisLockLoop == true) {
            yAxisLockLoop = false;
            telemetry.addData("Y Axis Lock: ", "Disabled!");

        }


        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double intakePower = gamepad2.left_stick_x;

        if(xAxisLockLoop == true) {
            ly = 0;
            rx = 0;
        }

        if(yAxisLockLoop == true) {
            lx = 0;
            rx = 0;
        }


        /*if (gamepad1.x == true) {
            xAxisLockLoop = true;
            telemetry.addData("X-Axis Lock:", "Activated!");
            telemetry.update();
        } else if (gamepad1.x == false) {
            xAxisLockLoop = false;
        }

        if (xAxisLockLoop == true) {
            ly = 0;
            rx = 0;
        }

        if (gamepad1.y == true) {
            yAxisLockLoop = true;
        } else if (gamepad1.y == false) {
            yAxisLockLoop = false;
        }

        if (yAxisLockLoop == true) {
            lx = 0;
            rx = 0;
        } */

        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

        /* max power is is 0.7 cuz testing and don't want to go crazy;  change if necessary
        divide by denominator to correct for imperfect strafing */
        double left_front_power = (ly + lx - rx) / denominator;
        double left_back_power = (ly - lx - rx) / denominator;
        double right_front_power = (ly - lx + rx) / denominator;
        double right_back_power = (ly + lx + rx) / denominator;

        left_front.setPower(-1 * (0.15 * left_front_power));
        left_back.setPower(-1 * (0.15 * left_back_power));
        right_front.setPower(-1 * (0.15 * right_front_power));
        right_back.setPower(-1 * (0.15 * right_back_power));

        telemetry.addData("Left Front ", left_front_power);
        telemetry.addData("Left Back ", left_back_power);
        telemetry.addData("Right Front ", right_front_power);
        telemetry.addData("Right Back ", right_back_power);
        telemetry.update();

        while (gamepad2.right_bumper == true) {
            carousel.setPower(200);
            if (gamepad2.right_bumper == false) {
                carousel.setPower(0);
                break;
            }
        }

        while (gamepad2.left_bumper == true) {
            carousel.setPower(-200);
            if (gamepad2.left_bumper == false) {
                carousel.setPower(0);
                break;
            }
        }
        
        //intake.setPower(intakePower);

        /*
        if (gamepad1.b == true) {

            right_front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            left_front.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            left_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            right_back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            right_front.setTargetPosition(-TK);
            right_back.setTargetPosition(-TK);
            left_front.setTargetPosition(TK);
            left_back.setTargetPosition(TK);


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
        */

        /*if (gamepad1.dpad_right == true) {
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);

            while (xAxisLockLoop  == true) {
                double left_x = gamepad1.left_stick_x;

                left_front.setPower(left_x);
                left_back.setPower(-left_x);
                right_front.setPower(-left_x);
                right_back.setPower(left_x);

                telemetry.addData("Axis Lock: ", "X Activated");
                telemetry.addData("Left Front Power: ", left_front.getPower());
                telemetry.addData("Right Front Power: ", right_front.getPower());
                telemetry.addData("Left Back Power: ", left_back.getPower());
                telemetry.addData("Right Back Power: ", right_back.getPower());
                telemetry.update();

                if (gamepad1.dpad_left == true && gamepad1.dpad_right == false) {
                    left_front.setPower(0);
                    left_back.setPower(0);
                    right_front.setPower(0);
                    right_back.setPower(0);

                    break;
                }
            }
        } */

        /*if (gamepad1.dpad_up == true) {
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);

            while (yAxisLockLoop == true) {
                double left_y = gamepad1.left_stick_y;

                left_front.setPower(left_y);
                left_back.setPower(left_y);
                right_front.setPower(left_y);
                right_back.setPower(left_y);

                telemetry.addData("Axis Lock: ", "Y Activated");
                telemetry.addData("Left Front Power: ", left_front.getPower());
                telemetry.addData("Right Front Power: ", right_front.getPower());
                telemetry.addData("Left Back Power: ", left_back.getPower());
                telemetry.addData("Right Back Power: ", right_back.getPower());
                telemetry.update();

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
        } */



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

        pidGainsLF.p = pidCoeffsLF.p * error;
        pidGainsLF.i = pidCoeffsLF.i * integralLF;
        pidGainsLF.d = pidCoeffsLF.d = pidCoeffsLF.d * derivative;


        left_front.setVelocity(pidGainsLF.p + pidGainsLF.i + pidGainsLF.d + targetVelocity);

        lastErrorLF = error;
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

        pidGainsLB.p = pidCoeffsLB.p * error;
        pidGainsLB.i = pidCoeffsLB.i * integralLB;
        pidGainsLB.d = pidCoeffsLB.d = pidCoeffsLB.d * derivative;


        left_back.setVelocity(pidGainsLB.p + pidGainsLB.i + pidGainsLB.d + targetVelocity);

        lastErrorLB = error;

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

        pidGainsRF.p = pidCoeffsRF.p * error;
        pidGainsRF.i = pidCoeffsRF.i * integralRF;
        pidGainsRF.d = pidCoeffsRF.d = pidCoeffsRF.d * derivative;


        right_front.setVelocity(pidGainsRF.p + pidGainsRF.i + pidGainsRF.d + targetVelocity);

        lastErrorRF = error;
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

        pidGainsRB.p = pidCoeffsRB.p * error;
        pidGainsRB.i = pidCoeffsRB.i * integralRB;
        pidGainsRB.d = pidCoeffsRB.d = pidCoeffsRB.d * derivative;

        right_back.setVelocity(pidGainsRB.p + pidGainsRB.i + pidGainsRB.d + targetVelocity);

        lastErrorRB = error;
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

        pidGainsCarousel.p = pidCoeffsCarousel.p * error;
        pidGainsCarousel.i = pidCoeffsCarousel.i * integralRB;
        pidGainsCarousel.d = pidCoeffsCarousel.d = pidCoeffsCarousel.d * derivative;

        carousel.setVelocity(pidGainsCarousel.p + pidGainsCarousel.i + pidGainsCarousel.d + targetVelocity);

        lastErrorCarousel = error;
    }

    double integralCascade = 0;
    double lastErrorCascade = 0;

    /*public void PIDcascade(double targetVelocity) {
        PIDTimerCascade.reset();
        double currentVelocity = cascadingLift.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralCascade += error * PIDTimerCascade.time();

        double deltaError = error - lastErrorCascade;
        double derivative = deltaError / PIDTimerCascade.time();

        pidGainsCascade.p = pidCoeffsCascade.p * error;
        pidGainsCascade.i = pidCoeffsCascade.i * integralCascade;
        pidGainsCascade.d = pidCoeffsCascade.d = pidCoeffsCascade.d * derivative;

        cascadingLift.setVelocity(pidGainsCascade.p + pidGainsCascade.i + pidGainsCascade.d + targetVelocity);

        lastErrorCascade = error;
    } */
}
