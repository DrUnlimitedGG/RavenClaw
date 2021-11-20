package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {

    private DcMotorEx LF = null;
    private DcMotorEx LB = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;

    private final double encoderConstant = 45.2847909695;

    public void runOpMode() {
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");

        LF.setDirection(DcMotorEx.Direction.FORWARD);
        LB.setDirection(DcMotorEx.Direction.FORWARD);
        // reverse right side
        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);

        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

        }
    }

    public void quitDriving() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void driveForward(double distance, double speed) {
        LF.setTargetPosition((int) (distance * encoderConstant));
        LB.setTargetPosition((int) (distance * encoderConstant));
        RF.setTargetPosition((int) (distance * encoderConstant));
        RB.setTargetPosition((int) (distance * encoderConstant));

        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LF.setPower(speed);
        LB.setPower(speed);
        RF.setPower(speed);
        RB.setPower(speed);

        while (LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy()) {

        }

        quitDriving();

    }

    public void driveBack(double distance, double speed) {
        LF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        LB.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RB.setTargetPosition((int) (-1 * (distance * encoderConstant)));

        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LF.setPower(speed);
        LB.setPower(speed);
        RF.setPower(speed);
        RB.setPower(speed);

        while (LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy()) {

        }

        quitDriving();
    }


}

