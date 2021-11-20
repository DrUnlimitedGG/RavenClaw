
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
    //public Servo carousel;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
            telemetry.addData("Status", "Initialized");

            right_front = hardwareMap.get(DcMotorEx.class, "right_front");
            left_front = hardwareMap.get(DcMotorEx.class, "left_front");
            right_back = hardwareMap.get(DcMotorEx.class, "right_back");
            left_back = hardwareMap.get(DcMotorEx.class, "left_back");
            //carousel = hardwareMap.get(DcMotorEx.class, "carousel");

            right_front.setDirection(DcMotorEx.Direction.REVERSE);
            right_back.setDirection(DcMotorEx.Direction.REVERSE);
            left_front.setDirection(DcMotorEx.Direction.FORWARD);
            left_back.setDirection(DcMotorEx.Direction.FORWARD);




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
        telemetry.addData("Status", "More dog than a golden retriever");
        telemetry.addData("Status", "More free than a costco sample");
        telemetry.update();

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double lx = gamepad2.left_stick_x;
        double ly = -gamepad2.left_stick_y;
        double rx = gamepad2.right_stick_x;


        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

        double left_front_power = (ly + lx + rx)/denominator;
        double left_back_power = (ly - lx + rx)/denominator;
        double right_front_power = (ly - lx - rx)/denominator;
        double right_back_power = (ly + lx - rx)/denominator;

        left_front.setPower(left_front_power);
        left_back.setPower(left_back_power);
        right_front.setPower(right_front_power);
        right_back.setPower(right_back_power);

        /*
        left_front.setPower(ly + lx + rx);
        left_back.setPower(ly - lx + rx);
        right_front.setPower(ly - lx - rx);
        right_back.setPower(ly + lx - rx);
        */


        telemetry.addData("Left Front: ", left_front_power);
        telemetry.addData("Left Back: ", left_back_power);
        telemetry.addData("Right Front: ", right_front_power);
        telemetry.addData("Right Back: ", right_back_power);
        telemetry.addData("Runtime: ", runtime.toString());
        telemetry.update();




    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}



