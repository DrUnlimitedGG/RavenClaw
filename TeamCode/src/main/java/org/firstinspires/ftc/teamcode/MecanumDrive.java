
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

    private ElapsedTime runtime;


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
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        telemetry.addData("LX: ", lx);
        telemetry.addData("LY: ", ly);
        telemetry.addData("RX: ", rx);
        telemetry.update();

        left_front.setPower(ly + lx + rx);
        left_back.setPower(ly - lx + rx);
        right_front.setPower(ly - lx - rx);
        right_back.setPower(ly + lx - rx);

        telemetry.addData("Left Front: ", ly + lx + rx);
        telemetry.addData("Left Back: ", ly - lx + rx);
        telemetry.addData("Right Front: ", ly - lx - rx);
        telemetry.addData("Right Back: ", ly + lx - rx);
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



