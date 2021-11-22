
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends OpMode {

    public DcMotorEx right_front;
    public DcMotorEx left_front;
    public DcMotorEx right_back;
    public DcMotorEx left_back;
    public DcMotorEx carousel;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //private double encoderConstant = 89.1267682333;

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


        telemetry.addData("Connected\n", "Left Front: ", left_front.getPortNumber());
        telemetry.addData("Right Front: ", right_front.getPortNumber());
        telemetry.addData("Left Back: ", left_back.getPortNumber());
        telemetry.addData("Right Front: ", right_front.getPortNumber());
        telemetry.update();

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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double carouselPower = gamepad1.right_trigger;

        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

        double left_front_power = (ly + lx + rx) / denominator;
        double left_back_power = (ly - lx + rx) / denominator;
        double right_front_power = (ly - lx - rx) / denominator;
        double right_back_power = (ly + lx - rx) / denominator;

        left_front.setPower(-1 * left_front_power);
        left_back.setPower(-1 * left_back_power);
        right_front.setPower(-1 * right_front_power);
        right_back.setPower(-1 * right_back_power);



        carousel.setPower(carouselPower);

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

            left_front.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            left_back.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            right_front.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            right_back.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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

    }

